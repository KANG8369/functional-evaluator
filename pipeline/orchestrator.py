import json
import os
import shutil
from datetime import datetime, timezone
from pathlib import Path

from agents.coder_agent import build_coder_chain, generate_package_spec
from agents.build_evaluator_agent import build_build_evaluator_chain, evaluate_build_log
from agents.functional_evaluator_agent import build_functional_evaluator_chain, evaluate_functional

from pipeline.workspace_manager import ensure_pkg_created, write_files
from pipeline.build_runner import build_workspace
from pipeline.criteria_loader import load_criteria
from pipeline.runtime_executor import RuntimeExecutor


def _log(msg, verbose):
    if verbose:
        print(msg, flush=True)


def _utc_ts():
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _read_text(path):
    p = Path(path)
    if not p.exists():
        return "(missing file: {})".format(p)
    return p.read_text(encoding="utf-8", errors="replace")


def run_pipeline(
    project_root,
    ros_distro="humble",
    max_attempts=4,
    evaluator_max_chars=20000,
    stop_on_first_fail=True,
    verbose=False,
    model_name=None,
):
    if "OPENAI_API_KEY" not in os.environ:
        raise RuntimeError("Set OPENAI_API_KEY env variable.")

    project_root = Path(project_root)
    description_dir = project_root / "description"
    criteria_dir = project_root / "criteria"
    ws_root = project_root / "generated_ros2_packages"
    artifacts_root = project_root / "runtime_artifacts"
    failed_root = project_root / "failed_ros2_packages"
    reports_root = project_root / "reports"

    txt_files = sorted(description_dir.glob("*.txt"))
    if not txt_files:
        raise RuntimeError("No description .txt files found")

    coder_chain = build_coder_chain() if model_name is None else build_coder_chain(model_name=model_name)
    be_chain = build_build_evaluator_chain() if model_name is None else build_build_evaluator_chain(model_name=model_name)
    fe_chain = build_functional_evaluator_chain()

    build_report = {"run_started_at": _utc_ts(), "items": []}
    functional_report = {"run_started_at": _utc_ts(), "items": []}

    _log("=== ROS2 Package Generator Pipeline ===", verbose)
    _log("- ROS_DISTRO: {}".format(ros_distro), verbose)
    _log("- Workspace: {}".format(ws_root), verbose)
    _log("- Runtime artifacts: {}".format(artifacts_root), verbose)
    _log("- Failed archive: {}".format(failed_root), verbose)
    _log("- Tasks: {}".format(len(txt_files)), verbose)
    _log("", verbose)

    for txt in txt_files:
        task_id = txt.stem
        pkg_name = task_id.strip()

        _log("=== Task: {} (pkg={}) ===".format(txt.name, pkg_name), verbose)

        desc = txt.read_text(encoding="utf-8", errors="replace").strip()
        criteria_path = criteria_dir / (task_id + ".yaml")
        criteria = load_criteria(criteria_path, task_id)

        feedback = ""
        success = False
        last_error = ""

        for attempt in range(1, int(max_attempts) + 1):
            _log("[Orchestrator] Attempt {}/{} pkg={}".format(attempt, max_attempts, pkg_name), verbose)
            _log("[Orchestrator] 1) Coder generating package spec...", verbose)

            spec = generate_package_spec(coder_chain, desc, feedback=feedback)

            # enforce package name
            if getattr(spec, "package_name", "").strip() != pkg_name:
                last_error = "package_name_mismatch: expected={} got={}".format(pkg_name, getattr(spec, "package_name", ""))
                _log("[Orchestrator] ❌ " + last_error, verbose)
                feedback = "PACKAGE NAME MUST EQUAL '{}'. Regenerate.".format(pkg_name)
                continue

            files = getattr(spec, "files", [])
            _log("[Orchestrator]   ↳ Coder returned package_name={} files={}".format(spec.package_name, len(files)), verbose)

            _log("[Orchestrator] 2) Materializing workspace/package...", verbose)
            ws_root.mkdir(parents=True, exist_ok=True)
            pkg_root = ensure_pkg_created(ws_root=ws_root, package_name=pkg_name, ros_distro=ros_distro, pkg_name=pkg_name)
            write_files(pkg_root=pkg_root, files=files)

            _log("[Orchestrator] 3) Building workspace...", verbose)
            rc, build_log_path = build_workspace(ws_root, ros_distro)
            build_log_text = _read_text(build_log_path)

            if rc != 0:
                _log("[Orchestrator] ❌ BUILD FAILED rc={}. Log: {}".format(rc, build_log_path), verbose)
                tail = build_log_text[-3000:] if len(build_log_text) > 3000 else build_log_text
                _log("[Orchestrator] --- build log tail ---\n{}\n--- end ---".format(tail), verbose)

                # build evaluator feedback
                log_for_eval = build_log_text[-int(evaluator_max_chars):] if len(build_log_text) > int(evaluator_max_chars) else build_log_text
                try:
                    be_res = evaluate_build_log(be_chain, log_for_eval)
                except Exception as e:
                    be_res = {"root_cause": str(e), "actionable_fixes": "Fix build issues using log."}

                # archive failed attempt
                attempt_dir = failed_root / task_id / ("attempt_{:02d}_{}".format(attempt, _utc_ts()))
                attempt_dir.mkdir(parents=True, exist_ok=True)
                # move package
                pkg_src = ws_root / "src" / pkg_name
                if pkg_src.exists():
                    shutil.move(str(pkg_src), str(attempt_dir / "package"))
                (attempt_dir / "spec.json").write_text(json.dumps({"package_name": spec.package_name}, indent=2))
                (attempt_dir / "build_log.txt").write_text(build_log_text)
                (attempt_dir / "evaluator_outputs.json").write_text(json.dumps({"build_evaluator": be_res}, indent=2))

                feedback = "BUILD FAILED. ROOT CAUSE: {}. FIXES: {}".format(
                    getattr(be_res, "root_cause", be_res.get("root_cause", "")),
                    getattr(be_res, "actionable_fixes", be_res.get("actionable_fixes", "")),
                )
                last_error = "build failed"
                _log("[Orchestrator] Retrying...\n", verbose)
                continue

            _log("[Orchestrator] ✅ BUILD OK. Proceeding to runtime + scenarios...", verbose)

            scenario_failed = False
            last_runtime = None
            last_scenario = None

            for scen in criteria.scenarios:
                scen_id = scen.get("id", "unknown_scenario")
                _log("[Orchestrator] 4) Runtime session start: scenario_id={}".format(scen_id), verbose)

                executor = RuntimeExecutor(
                    package_name=pkg_name,
                    scenario=scen,
                    workspace_root=ws_root,
                    artifacts_root=artifacts_root,
                    ros_distro=ros_distro,
                    startup_grace_sec=2,
                    runtime_budget_sec=10,
                    teardown_grace_sec=3,
                    verbose=verbose,
                )
                runtime_report, scenario_report = executor.run()
                last_runtime, last_scenario = runtime_report, scenario_report

                run_status = runtime_report.get("runtime_status", {}).get("status", "UNKNOWN")
                scen_status = scenario_report.get("scenario_status", {}).get("status", "UNKNOWN")
                _log("[Orchestrator]   ↳ runtime_status={} scenario_status={}".format(run_status, scen_status), verbose)

                if run_status != "OK" or scen_status != "PASS":
                    scenario_failed = True
                    last_error = "runtime_or_scenario_failed: runtime={} scenario={} scenario_id={}".format(run_status, scen_status, scen_id)
                    _log("[Orchestrator] ❌ " + last_error, verbose)
                    if stop_on_first_fail:
                        break

            if scenario_failed:
                # archive runtime artifacts
                attempt_dir = failed_root / task_id / ("attempt_{:02d}_{}".format(attempt, _utc_ts()))
                attempt_dir.mkdir(parents=True, exist_ok=True)
                pkg_src = ws_root / "src" / pkg_name
                if pkg_src.exists():
                    shutil.move(str(pkg_src), str(attempt_dir / "package"))
                # copy latest runtime session dir
                if last_runtime is not None:
                    run_id = last_runtime.get("run_id")
                    scen_id = last_runtime.get("scenario_id")
                    session_dir = artifacts_root / run_id / pkg_name / scen_id
                    if session_dir.exists():
                        shutil.copytree(session_dir, attempt_dir / "runtime_artifacts")
                (attempt_dir / "build_log.txt").write_text(_read_text(build_log_path))
                (attempt_dir / "evaluator_outputs.json").write_text(json.dumps({"runtime_failure": last_error}, indent=2))
                feedback = "RUNTIME/SCENARIO FAILED. Fix runtime behavior. " + last_error
                _log("[Orchestrator] Retrying...\n", verbose)
                continue

            # Functional evaluation (stub)
            _log("[Orchestrator] 5) Functional evaluation...", verbose)
            func_out = evaluate_functional(
                fe_chain,
                criteria_yaml=_read_text(criteria_path),
                runtime_report_json=json.dumps(last_runtime, indent=2) if last_runtime else "",
                scenario_report_json=json.dumps(last_scenario, indent=2) if last_scenario else "",
                probes_text="",
                node_logs_text="",
            )

            verdict = func_out.get("verdict", "FAIL")
            _log("[Orchestrator]   ↳ Functional verdict={}".format(verdict), verbose)

            if verdict != "PASS":
                last_error = "functional_failed"
                feedback = "FUNCTIONAL FAILED. Fix behavior."
                _log("[Orchestrator] Retrying...\n", verbose)
                continue

            success = True
            last_error = ""
            _log("[Orchestrator] ✅ Task SUCCESS.", verbose)
            break

        build_report["items"].append({"task_id": task_id, "package_name": pkg_name, "success": success, "last_error": last_error})
        functional_report["items"].append({"task_id": task_id, "package_name": pkg_name, "verdict": "PASS" if success else "FAIL", "last_error": last_error})

    reports_root.mkdir(parents=True, exist_ok=True)
    (reports_root / "build_report.json").write_text(json.dumps(build_report, indent=2))
    (reports_root / "functional_report.json").write_text(json.dumps(functional_report, indent=2))
    return build_report, functional_report

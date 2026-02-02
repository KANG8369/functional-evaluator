from __future__ import annotations

import json
import re
import shutil
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

from agents.coder_agent import CODER_SYSTEM_PROMPT, build_coder_chain, generate_package_spec
from agents.build_evaluator_agent import build_evaluator_chain, evaluate_build_log
from agents.functional_evaluator_agent import build_functional_evaluator_chain, evaluate_functional

from pipeline.artifacts import make_run_id, session_paths, write_json, write_text
from pipeline.criteria_loader import load_criteria
from pipeline.workspace_manager import ensure_pkg_created, write_files
from pipeline.build_runner import build_workspace
from pipeline.runtime_executor import RuntimeExecutor


def sanitize_pkg_name(stem: str) -> str:
    """
    Enforce: package name == description file stem (lowercase with underscores).
    """
    s = (stem or "").strip().lower()
    s = re.sub(r"[^a-z0-9_]+", "_", s)
    s = re.sub(r"_+", "_", s).strip("_")
    if not s:
        s = "generated_pkg"
    if s[0].isdigit():
        s = f"pkg_{s}"
    return s


def _log(msg: str, verbose: bool) -> None:
    if verbose:
        print(msg, flush=True)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _read_text(path: Path) -> str:
    if not path.exists():
        return f"(missing: {path})"
    return path.read_text(encoding="utf-8", errors="replace")


def _attempt_dir(failed_root: Path, task_id: str, attempt: int, pkg: str) -> Path:
    ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    d = failed_root / task_id / f"attempt_{attempt:02d}_{pkg}_{ts}"
    d.mkdir(parents=True, exist_ok=True)
    return d


def archive_failed_attempt(
    *,
    failed_root: Path,
    task_id: str,
    attempt: int,
    ws_root: Path,
    package_name: str,
    spec_blob: Dict[str, Any],
    coder_feedback: str,
    build_log_text: Optional[str],
    evaluator_outputs: Dict[str, Any],
    runtime_session_dir: Optional[Path],
) -> Path:
    """
    Requirement:
      - Do NOT delete failed packages. Move them out of generated workspace.
      - Save spec, prompts/feedback, build log, evaluator outputs, runtime artifacts (if any).
    """
    d = _attempt_dir(failed_root, task_id, attempt, package_name)

    # Move package out of workspace
    pkg_src = ws_root / "src" / package_name
    pkg_dst = d / "package"
    if pkg_dst.exists():
        shutil.rmtree(pkg_dst)
    if pkg_src.exists():
        shutil.move(str(pkg_src), str(pkg_dst))

    write_json(d / "spec.json", spec_blob)
    write_json(
        d / "coder_prompt.json",
        {"system": CODER_SYSTEM_PROMPT, "description_task_id": task_id, "feedback": coder_feedback},
    )
    if build_log_text is not None:
        (d / "build_log.txt").write_text(build_log_text, encoding="utf-8")
    write_json(d / "evaluator_outputs.json", evaluator_outputs)

    if runtime_session_dir is not None and runtime_session_dir.exists():
        shutil.copytree(runtime_session_dir, d / "runtime_artifacts", dirs_exist_ok=True)

    return d


def run_pipeline(
    *,
    project_root: Path,
    ros_distro: str = "humble",
    max_attempts: int = 4,
    evaluator_max_chars: int = 20000,
    stop_on_first_fail: bool = True,
    verbose: bool = False,
) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    """
    Pipeline:
      description -> coder -> materialize -> build -> runtime_smoke(scenarios) -> functional_eval -> retry

    Writes:
      reports/build_report.json
      reports/functional_report.json

    Returns:
      (build_report, functional_report)
    """
    description_dir = project_root / "description"
    criteria_dir = project_root / "criteria"
    ws_root = project_root / "generated_ros2_packages"
    artifacts_root = project_root / "runtime_artifacts"
    failed_root = project_root / "failed_ros2_packages"
    build_script = project_root / "scripts" / "build_ros2_ws.sh"

    ws_root.mkdir(parents=True, exist_ok=True)
    (ws_root / "src").mkdir(parents=True, exist_ok=True)
    artifacts_root.mkdir(parents=True, exist_ok=True)
    failed_root.mkdir(parents=True, exist_ok=True)
    (project_root / "reports").mkdir(parents=True, exist_ok=True)

    coder_chain = build_coder_chain(model_name="gpt-5.2")
    build_eval_chain = build_evaluator_chain(model_name="gpt-5.2")
    func_chain = build_functional_evaluator_chain(model_name="gpt-5.2")

    build_report: Dict[str, Any] = {"schema_version": "build_report.v1", "run_started_at": _utc_now(), "items": []}
    functional_report: Dict[str, Any] = {"schema_version": "functional_report.v1", "run_started_at": _utc_now(), "items": []}

    txt_files = sorted(description_dir.glob("*.txt"))
    if not txt_files:
        raise RuntimeError(f"No description .txt files in {description_dir}")

    for txt in txt_files:
        task_id = txt.stem
        expected_pkg = sanitize_pkg_name(task_id)

        _log(f"\n=== Task: {txt.name} (pkg={expected_pkg}) ===", verbose)

        desc_text = txt.read_text(encoding="utf-8", errors="replace").strip()
        if not desc_text:
            build_report["items"].append({"task_id": task_id, "package_name": expected_pkg, "success": False, "reason": "empty_description"})
            functional_report["items"].append({"task_id": task_id, "package_name": expected_pkg, "verdict": "FAIL", "reason": "empty_description"})
            continue

        criteria_path = criteria_dir / f"{task_id}.yaml"
        criteria = load_criteria(criteria_path, task_id=task_id)
        criteria_yaml_text = criteria_path.read_text(encoding="utf-8", errors="replace")

        feedback = ""
        success = False
        last_error = ""
        attempts_used = 0
        final_verdict = "FAIL"

        for attempt in range(1, max_attempts + 1):
            attempts_used = attempt
            evaluator_outputs: Dict[str, Any] = {}

            _log(f"[Orchestrator] Attempt {attempt}/{max_attempts} pkg={expected_pkg}", verbose)

            # 1) Coder
            _log("[Orchestrator] 1) Coder generating package spec...", verbose)
            try:
                spec = generate_package_spec(coder_chain, desc_text, feedback=feedback)
            except Exception as e:
                last_error = f"coder_failed: {e}"
                feedback = "Coder failed to produce a valid structured spec. Output must match schema."
                _log(f"[Orchestrator] ❌ {last_error}", verbose)
                continue

            _log(f"[Orchestrator]   ↳ Coder returned package_name={spec.package_name} files={len(spec.files)}", verbose)

            # Enforce package name == task_id stem
            if spec.package_name.strip() != expected_pkg:
                last_error = f"package_name_mismatch: expected={expected_pkg} got={spec.package_name}"
                feedback = f"PACKAGE NAME MUST EQUAL '{expected_pkg}'. You returned '{spec.package_name}'. Fix."
                _log(f"[Orchestrator] ❌ {last_error}", verbose)
                continue

            # 2) Materialize
            _log("[Orchestrator] 2) Materializing workspace/package...", verbose)
            try:
                pkg_root = ensure_pkg_created(ws_root=ws_root, package_name=expected_pkg, ros_distro=ros_distro)
                tuple_files = [(f.path, f.content) for f in spec.files]
                write_files(pkg_root=pkg_root, files=tuple_files)
            except Exception as e:
                last_error = f"materialize_failed: {e}"
                feedback = f"Workspace writing failed: {e}. Ensure file paths are valid relative POSIX paths."
                _log(f"[Orchestrator] ❌ {last_error}", verbose)
                continue

            # 3) Build
            _log("[Orchestrator] 3) Building workspace...", verbose)
            rc, build_log_path = build_workspace(ws_root=ws_root, ros_distro=ros_distro, build_script=build_script)
            build_log_text = _read_text(build_log_path)

            if rc != 0:
                last_error = f"build_failed_rc={rc}"
                _log(f"[Orchestrator] ❌ BUILD FAILED rc={rc}. Log: {build_log_path}", verbose)
                tail = build_log_text[-3000:] if len(build_log_text) > 3000 else build_log_text
                _log("[Orchestrator] --- build log tail (last ~3k chars) ---\n" + tail + "\n--- end ---", verbose)

                # Build evaluator
                _log("[Orchestrator] 3.1) Build Evaluator analyzing log...", verbose)
                log_tail = build_log_text[-evaluator_max_chars:] if len(build_log_text) > evaluator_max_chars else build_log_text
                try:
                    be_res = evaluate_build_log(build_eval_chain, log_tail)
                    evaluator_outputs["build_evaluator"] = be_res.model_dump() if hasattr(be_res, "model_dump") else asdict(be_res)  # type: ignore
                    feedback = (
                        "BUILD FAILED. Apply these fixes strictly.\n"
                        f"ROOT CAUSE:\n{be_res.root_cause}\n\n"
                        f"ACTIONABLE FIXES:\n{be_res.actionable_fixes}\n"
                    )
                except Exception as e:
                    evaluator_outputs["build_evaluator_error"] = str(e)
                    feedback = "Build failed. Fix compile/CMake/package.xml issues using the build log."

                # Archive failed attempt (moves package out of ws)
                _log("[Orchestrator] Archiving failed build attempt...", verbose)
                spec_blob = spec.model_dump() if hasattr(spec, "model_dump") else asdict(spec)  # type: ignore
                archive_failed_attempt(
                    failed_root=failed_root,
                    task_id=task_id,
                    attempt=attempt,
                    ws_root=ws_root,
                    package_name=expected_pkg,
                    spec_blob=spec_blob,
                    coder_feedback=feedback,
                    build_log_text=build_log_text,
                    evaluator_outputs=evaluator_outputs,
                    runtime_session_dir=None,
                )
                _log("[Orchestrator] Retrying...\n", verbose)
                continue

            _log("[Orchestrator] ✅ BUILD OK. Proceeding to runtime + scenarios...", verbose)

            # 4) Runtime per scenario (1 scenario = 1 runtime session)
            run_id = make_run_id()
            scenario_failed = False
            last_session_dir: Optional[Path] = None
            last_runtime_report: Optional[Dict[str, Any]] = None
            last_scenario_report: Optional[Dict[str, Any]] = None

            for scenario in criteria.scenarios:
                scenario_id = scenario["id"]
                _log(f"[Orchestrator] 4) Runtime session start: scenario_id={scenario_id}", verbose)

                paths = session_paths(
                    artifacts_root=artifacts_root,
                    run_id=run_id,
                    package_name=expected_pkg,
                    scenario_id=scenario_id,
                )
                last_session_dir = paths.session_dir

                # snapshots for reproducibility
                write_text(paths.session_dir / "criteria_snapshot.yaml", criteria_yaml_text)
                write_text(paths.session_dir / "description_snapshot.txt", desc_text)

                executor = RuntimeExecutor(
                    package_name=expected_pkg,
                    scenario=scenario,
                    workspace_root=ws_root,
                    artifacts_root=artifacts_root,
                    run_id=run_id,
                    ros_distro=ros_distro,
                    startup_grace_sec=2,
                    teardown_grace_sec=3,
                    verbose=verbose,
                )
                runtime_report, scenario_report = executor.run()
                last_runtime_report, last_scenario_report = runtime_report, scenario_report

                run_status = (runtime_report.get("runtime_status", {}) or {}).get("status", "UNKNOWN")
                scen_status = (scenario_report.get("scenario_status", {}) or {}).get("status", "UNKNOWN")

                _log(f"[Orchestrator]   ↳ runtime_status={run_status} scenario_status={scen_status}", verbose)

                if run_status != "OK" or scen_status != "PASS":
                    scenario_failed = True
                    last_error = f"runtime_or_scenario_failed: runtime={run_status} scenario={scen_status} scenario_id={scenario_id}"
                    evaluator_outputs["runtime_executor"] = {"runtime_status": run_status, "scenario_status": scen_status}
                    _log(f"[Orchestrator] ❌ {last_error}", verbose)

                    if stop_on_first_fail:
                        break

            if scenario_failed:
                # Archive failed runtime attempt (moves package out of ws)
                _log("[Orchestrator] Archiving failed runtime/scenario attempt...", verbose)
                spec_blob = spec.model_dump() if hasattr(spec, "model_dump") else asdict(spec)  # type: ignore
                archive_failed_attempt(
                    failed_root=failed_root,
                    task_id=task_id,
                    attempt=attempt,
                    ws_root=ws_root,
                    package_name=expected_pkg,
                    spec_blob=spec_blob,
                    coder_feedback=feedback,
                    build_log_text=build_log_text,
                    evaluator_outputs=evaluator_outputs,
                    runtime_session_dir=last_session_dir,
                )
                feedback = f"RUNTIME/SCENARIO FAILED: {last_error}. Fix node runtime behavior and/or interfaces."
                _log("[Orchestrator] Retrying...\n", verbose)
                continue

            # 5) Functional evaluation (LLM) using LAST scenario session evidence
            if last_session_dir is None:
                last_error = "no_session_dir_for_functional_eval"
                _log(f"[Orchestrator] ❌ {last_error}", verbose)
                feedback = "No runtime artifacts were produced; cannot do functional evaluation."
                continue

            _log("[Orchestrator] 5) Functional evaluation: collecting evidence...", verbose)

            runtime_report_json = _read_text(last_session_dir / "runtime_report.json")
            scenario_report_json = _read_text(last_session_dir / "scenario_report.json")

            # probes
            probes_dir = last_session_dir / "probes"
            probes_text_parts = []
            if probes_dir.exists():
                for p in sorted(probes_dir.glob("*.txt")):
                    probes_text_parts.append(f"## {p.name}\n{_read_text(p)}")
            probes_text = "\n\n".join(probes_text_parts) if probes_text_parts else "(no probes)"

            # node logs
            node_logs_dir = last_session_dir / "node_logs"
            node_logs_parts = []
            if node_logs_dir.exists():
                for p in sorted(node_logs_dir.glob("*.txt")):
                    t = _read_text(p)
                    if len(t) > 6000:
                        t = "[NOTE] truncated to last 6000 chars\n\n" + t[-6000:]
                    node_logs_parts.append(f"## {p.name}\n{t}")
            node_logs_text = "\n\n".join(node_logs_parts) if node_logs_parts else "(no node logs)"

            _log("[Orchestrator] 5.1) Calling Functional Evaluator (LLM)...", verbose)
            try:
                func_res = evaluate_functional(
                    func_chain,
                    criteria_yaml=criteria_yaml_text,
                    runtime_report_json=runtime_report_json,
                    scenario_report_json=scenario_report_json,
                    probes_text=probes_text,
                    node_logs_text=node_logs_text,
                )
                func_out = func_res.model_dump() if hasattr(func_res, "model_dump") else asdict(func_res)  # type: ignore
            except Exception as e:
                func_out = {
                    "verdict": "FAIL",
                    "failed_criteria": ["functional_evaluator_crash"],
                    "root_cause": str(e),
                    "actionable_fixes": ["Fix functional evaluator implementation or evidence formatting."],
                }

            write_json(last_session_dir / "functional_report.json", func_out)
            verdict = func_out.get("verdict", "FAIL")
            _log(f"[Orchestrator]   ↳ Functional verdict={verdict} failed_criteria={func_out.get('failed_criteria', [])}", verbose)

            if verdict != "PASS":
                last_error = "functional_eval_failed"
                evaluator_outputs["functional_evaluator"] = func_out

                _log("[Orchestrator] ❌ Functional evaluation failed. Archiving & retrying...", verbose)
                spec_blob = spec.model_dump() if hasattr(spec, "model_dump") else asdict(spec)  # type: ignore
                archive_failed_attempt(
                    failed_root=failed_root,
                    task_id=task_id,
                    attempt=attempt,
                    ws_root=ws_root,
                    package_name=expected_pkg,
                    spec_blob=spec_blob,
                    coder_feedback=feedback,
                    build_log_text=build_log_text,
                    evaluator_outputs=evaluator_outputs,
                    runtime_session_dir=last_session_dir,
                )

                # feedback to coder
                feedback = (
                    "FUNCTIONAL EVALUATION FAILED.\n"
                    f"ROOT CAUSE:\n{func_out.get('root_cause','')}\n\n"
                    "ACTIONABLE FIXES:\n" + "\n".join(f"- {x}" for x in func_out.get("actionable_fixes", []))
                )
                _log("[Orchestrator] Retrying...\n", verbose)
                continue

            # Success
            success = True
            final_verdict = "PASS"
            last_error = ""
            _log("[Orchestrator] ✅ Task SUCCESS (build + runtime + functional PASS).", verbose)
            break

        build_report["items"].append(
            {
                "task_id": task_id,
                "package_name": expected_pkg,
                "success": success,
                "attempts_used": attempts_used,
                "last_error": last_error,
                "workspace_path": str(ws_root),
                "build_log_path": str(ws_root / "build_logs" / "build.txt"),
            }
        )
        functional_report["items"].append(
            {"task_id": task_id, "package_name": expected_pkg, "verdict": final_verdict, "attempts_used": attempts_used, "last_error": last_error}
        )

    # write reports
    reports_dir = project_root / "reports"
    write_json(reports_dir / "build_report.json", build_report)
    write_json(reports_dir / "functional_report.json", functional_report)

    return build_report, functional_report

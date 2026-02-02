
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
    s = (stem or "").strip().lower()
    s = re.sub(r"[^a-z0-9_]+", "_", s)
    s = re.sub(r"_+", "_", s).strip("_")
    if not s:
        s = "generated_pkg"
    if s[0].isdigit():
        s = f"pkg_{s}"
    return s

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
    d = _attempt_dir(failed_root, task_id, attempt, package_name)

    # Move package out of workspace (requirement)
    pkg_src = ws_root / "src" / package_name
    pkg_dst = d / "package"
    if pkg_dst.exists():
        shutil.rmtree(pkg_dst)
    if pkg_src.exists():
        shutil.move(str(pkg_src), str(pkg_dst))

    write_json(d / "spec.json", spec_blob)
    write_json(d / "coder_prompt.json", {"system": CODER_SYSTEM_PROMPT, "feedback": coder_feedback})
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
) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    description_dir = project_root / "description"
    criteria_dir = project_root / "criteria"
    ws_root = project_root / "generated_ros2_packages"
    artifacts_root = project_root / "runtime_artifacts"
    failed_root = project_root / "failed_ros2_packages"
    build_script = project_root / "scripts" / "build_ros2_ws.sh"

    ws_root.mkdir(parents=True, exist_ok=True)
    artifacts_root.mkdir(parents=True, exist_ok=True)
    failed_root.mkdir(parents=True, exist_ok=True)
    (project_root / "reports").mkdir(parents=True, exist_ok=True)

    coder_chain = build_coder_chain(model_name="gpt-5.2")
    build_eval_chain = build_evaluator_chain(model_name="gpt-5.2")
    func_chain = build_functional_evaluator_chain(model_name="gpt-5.2")

    build_report: Dict[str, Any] = {"run_started_at": _utc_now(), "items": []}
    functional_report: Dict[str, Any] = {"run_started_at": _utc_now(), "items": []}

    txt_files = sorted(description_dir.glob("*.txt"))
    if not txt_files:
        raise RuntimeError(f"No description .txt files in {description_dir}")

    for txt in txt_files:
        task_id = txt.stem
        pkg_name = sanitize_pkg_name(task_id)
        desc_text = txt.read_text(encoding="utf-8", errors="replace").strip()

        criteria_path = criteria_dir / f"{task_id}.yaml"
        criteria = load_criteria(criteria_path, task_id=task_id)
        criteria_yaml_text = criteria_path.read_text(encoding="utf-8", errors="replace")

        feedback = ""
        success = False
        last_error = ""

        for attempt in range(1, max_attempts + 1):
            spec = generate_package_spec(coder_chain, desc_text, feedback=feedback)

            # enforce naming
            if spec.package_name.strip() != pkg_name:
                feedback = f"PACKAGE NAME MUST BE '{pkg_name}'. You returned '{spec.package_name}'. Fix."
                last_error = "package_name mismatch"
                continue

            pkg_root = ensure_pkg_created(ws_root=ws_root, package_name=pkg_name, ros_distro=ros_distro)
            write_files(pkg_root=pkg_root, files=[(f.path, f.content) for f in spec.files])

            rc, build_log_path = build_workspace(ws_root=ws_root, ros_distro=ros_distro, build_script=build_script)
            build_log_text = _read_text(build_log_path)

            if rc != 0:
                tail = build_log_text[-evaluator_max_chars:] if len(build_log_text) > evaluator_max_chars else build_log_text
                evaluator_output = {}
                try:
                    eval_res = evaluate_build_log(build_eval_chain, tail)
                    evaluator_output = eval_res.model_dump() if hasattr(eval_res, "model_dump") else asdict(eval_res)
                except Exception as e:
                    evaluator_output = {"error": f"build evaluator failed: {e}"}

                archive_failed_attempt(
                    failed_root=failed_root,
                    task_id=task_id,
                    attempt=attempt,
                    ws_root=ws_root,
                    package_name=pkg_name,
                    spec_blob=spec.model_dump(),
                    coder_feedback=feedback,
                    build_log_text=build_log_text,
                    evaluator_outputs={"build": evaluator_output},
                    runtime_session_dir=None,
                )

                feedback = (
                    "BUILD FAILED. Apply these fixes strictly.\n"
                    f"ROOT CAUSE:\n{evaluator_output.get('root_cause','')}\n\n"
                    f"ACTIONABLE FIXES:\n{evaluator_output.get('actionable_fixes','')}\n"
                )
                last_error = f"build failed rc={rc}"
                continue

            # runtime + scenarios (STOP_ON_FIRST_FAIL)
            scenario_failed = False
            failed_scenario_id: Optional[str] = None
            last_session_dir: Optional[Path] = None
            last_func_output: Optional[Dict[str, Any]] = None

            for scenario in criteria.scenarios:
                run_id = make_run_id()
                paths = session_paths(artifacts_root=artifacts_root, run_id=run_id, package_name=pkg_name, scenario_id=scenario["id"])
                # snapshots
                write_text(paths.session_dir / "description_snapshot.txt", desc_text)
                write_text(paths.session_dir / "criteria_snapshot.yaml", criteria_yaml_text)

                executor = RuntimeExecutor(
                    package_name=pkg_name,
                    scenario=scenario,
                    workspace_root=ws_root,
                    artifacts_root=artifacts_root,
                    run_id=run_id,
                    ros_distro=ros_distro,
                    startup_grace_sec=2,
                    teardown_grace_sec=3,
                )
                runtime_report, scenario_report = executor.run()
                last_session_dir = paths.session_dir

                scen_status = scenario_report.get("scenario_status", {}).get("status", "FAIL_STEP_ERROR")
                run_status = runtime_report.get("runtime_status", {}).get("status", "UNKNOWN_ERROR")

                if scen_status != "PASS" or run_status != "OK":
                    scenario_failed = True
                    failed_scenario_id = scenario["id"]
                    last_error = f"runtime/scenario failed: runtime={run_status}, scenario={scen_status}"
                    break

                # Functional evaluation per scenario (this is where criteria expectations matter)
                probes_text = ""
                for p in sorted((paths.session_dir / "probes").glob("*.txt")):
                    probes_text += f"## {p.name}\n{_read_text(p)}\n\n"

                node_logs_text = ""
                for p in sorted((paths.session_dir / "node_logs").glob("*.txt")):
                    node_logs_text += f"## {p.name}\n{_read_text(p)}\n\n"

                func_res = evaluate_functional(
                    func_chain,
                    criteria_yaml=criteria_yaml_text,
                    runtime_report_json=json.dumps(runtime_report, ensure_ascii=False, indent=2),
                    scenario_report_json=json.dumps(scenario_report, ensure_ascii=False, indent=2),
                    probes_text=probes_text,
                    node_logs_text=node_logs_text,
                )
                func_out = func_res.model_dump() if hasattr(func_res, "model_dump") else func_res.dict()
                last_func_output = func_out
                write_json(paths.session_dir / "functional_report.json", func_out)

                functional_report["items"].append({
                    "task_id": task_id,
                    "package_name": pkg_name,
                    "attempt": attempt,
                    "scenario_id": scenario["id"],
                    "verdict": func_out.get("verdict"),
                    "failed_criteria": func_out.get("failed_criteria", []),
                })

                if func_out.get("verdict") != "PASS":
                    scenario_failed = True
                    failed_scenario_id = scenario["id"]
                    last_error = f"functional failed: {failed_scenario_id}"
                    break

                if stop_on_first_fail and scenario_failed:
                    break

            if scenario_failed:
                archive_failed_attempt(
                    failed_root=failed_root,
                    task_id=task_id,
                    attempt=attempt,
                    ws_root=ws_root,
                    package_name=pkg_name,
                    spec_blob=spec.model_dump(),
                    coder_feedback=feedback,
                    build_log_text=build_log_text,
                    evaluator_outputs={"functional": last_func_output or {}, "error": last_error},
                    runtime_session_dir=last_session_dir,
                )
                feedback = (
                    "FUNCTIONAL/RUNTIME FAILED. Fix behavior and/or scenario satisfaction.\n"
                    f"Last error: {last_error}\n"
                    f"Last functional output: {json.dumps(last_func_output or {}, ensure_ascii=False, indent=2)}\n"
                )
                continue

            # success
            success = True
            last_error = ""
            break

        build_report["items"].append({
            "task_id": task_id,
            "package_name": pkg_name,
            "success": success,
            "last_error": last_error,
        })

    build_report["run_finished_at"] = _utc_now()
    functional_report["run_finished_at"] = _utc_now()
    return build_report, functional_report

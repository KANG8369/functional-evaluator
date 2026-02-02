from __future__ import annotations

import json
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional


class ScenarioRunner:
    """
    Executes exactly ONE scenario. Nodes must already be running.
    FAIL_FAST: the first failing step stops the scenario.
    """

    def __init__(
        self,
        *,
        run_id: str,
        package_name: str,
        scenario: Dict[str, Any],
        artifacts_dir: Path,
        ros_env_prefix: str,
        verbose: bool = True,
    ) -> None:
        self.run_id = run_id
        self.package_name = package_name
        self.scenario = scenario
        self.scenario_id = scenario["id"]
        self.artifacts_dir = artifacts_dir
        self.ros_env_prefix = ros_env_prefix
        self.verbose = verbose

        self.steps_report: List[Dict[str, Any]] = []
        # state that steps can share (e.g. last action goal handle)
        self.state: Dict[str, Any] = {}

    def _log(self, msg: str) -> None:
        if self.verbose:
            print(f"[ScenarioRunner] {msg}", flush=True)

    def _utc_now(self) -> str:
        return datetime.now(timezone.utc).isoformat(timespec="seconds")

    def _run_cmd(self, cmd: str, timeout_sec: int) -> Dict[str, Any]:
        """
        Run a ROS CLI command via bash -lc so it can use 'source ...; ros2 ...'.
        Returns dict with rc/stdout/stderr.
        """
        started = self._utc_now()
        proc = subprocess.run(
            ["bash", "-lc", f"{self.ros_env_prefix}; {cmd}"],
            text=True,
            capture_output=True,
            timeout=timeout_sec,
        )
        ended = self._utc_now()
        return {
            "cmd": cmd,
            "timeout_sec": timeout_sec,
            "rc": proc.returncode,
            "stdout": proc.stdout,
            "stderr": proc.stderr,
            "timing": {"started_at_utc": started, "ended_at_utc": ended},
        }

    def _write_json(self, name: str, data: Dict[str, Any]) -> None:
        p = self.artifacts_dir / name
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")

    # ---------- Step handlers ----------

    def _step_delay(self, body: Dict[str, Any]) -> Dict[str, Any]:
        dur = float(body.get("duration_sec", 1))
        self._log(f"delay {dur}s")
        time.sleep(max(0.0, dur))
        return {"result": "OK", "duration_sec": dur}

    def _step_passive_run(self, body: Dict[str, Any]) -> Dict[str, Any]:
        dur = float(body.get("duration_sec", 1))
        self._log(f"passive_run {dur}s")
        time.sleep(max(0.0, dur))
        return {"result": "OK", "duration_sec": dur}

    def _step_wait_for_service(self, body: Dict[str, Any]) -> Dict[str, Any]:
        srv = body["service"]
        timeout = int(body.get("timeout_sec", 5))
        # ros2 service wait exists on Humble; use it.
        rep = self._run_cmd(f"ros2 service wait {srv} -t {timeout}", timeout_sec=timeout + 2)
        ok = rep["rc"] == 0
        return {"result": "OK" if ok else "FAIL", **rep}

    def _step_service_call(self, body: Dict[str, Any]) -> Dict[str, Any]:
        srv = body["service"]
        srv_type = body["type"]
        req = body.get("request", "{}")
        timeout = int(body.get("timeout_sec", 5))
        # use -t for timeout if supported; also guard overall timeout
        rep = self._run_cmd(f"ros2 service call {srv} {srv_type} '{req}'", timeout_sec=timeout + 5)
        ok = rep["rc"] == 0
        return {"result": "OK" if ok else "FAIL", **rep}

    def _step_topic_echo_wait(self, body: Dict[str, Any]) -> Dict[str, Any]:
        topic = body["topic"]
        n = int(body.get("min_messages", 1))
        timeout = int(body.get("timeout_sec", 5))
        # ros2 topic echo -n N ... will exit after N messages
        rep = self._run_cmd(f"ros2 topic echo -n {n} {topic}", timeout_sec=timeout)
        ok = rep["rc"] == 0 and (rep.get("stdout") or "").strip() != ""
        return {"result": "OK" if ok else "FAIL", **rep}

    def _step_action_send_goal(self, body: Dict[str, Any]) -> Dict[str, Any]:
        action = body["action"]
        action_type = body["type"]
        goal = body.get("goal", {})
        timeout = int(body.get("timeout_sec", 10))
        feedback = bool(body.get("feedback", False))

        # Note: ros2 action send_goal prints output; with --feedback it stays running.
        # We do NOT want it to run forever here; we use a timeout and capture.
        goal_str = json.dumps(goal)
        cmd = f"ros2 action send_goal {action} {action_type} '{goal_str}'"
        if feedback:
            cmd += " --feedback"

        rep = self._run_cmd(cmd, timeout_sec=timeout)
        ok = rep["rc"] == 0

        # Save last action for subsequent cancel/result waits
        self.state["last_action"] = action
        self.state["last_action_type"] = action_type
        self.state["last_goal_sent"] = goal

        return {"result": "OK" if ok else "FAIL", **rep}

    def _step_wait_for_feedback(self, body: Dict[str, Any]) -> Dict[str, Any]:
        """
        Minimal heuristic: verify we can see at least one feedback line in node logs
        is hard from here. Instead we check 'ros2 action info' and allow it to pass
        if action exists; actual feedback evidence should be in node logs and judged by functional evaluator.
        """
        timeout = int(body.get("timeout_sec", 5))
        min_messages = int(body.get("min_messages", 1))

        action = self.state.get("last_action")
        if not action:
            return {"result": "FAIL", "error": "No prior action_send_goal state (last_action missing)."}

        rep = self._run_cmd(f"ros2 action info {action}", timeout_sec=timeout)
        ok = rep["rc"] == 0
        rep["min_messages"] = min_messages
        rep["note"] = "Feedback presence should be validated from node logs by Functional Evaluator."
        return {"result": "OK" if ok else "FAIL", **rep}

    def _step_action_cancel_goal(self, body: Dict[str, Any]) -> Dict[str, Any]:
        action = body.get("action") or self.state.get("last_action")
        timeout = int(body.get("timeout_sec", 5))
        if not action:
            return {"result": "FAIL", "error": "No action name available for cancel."}

        # Some distros accept 'ros2 action cancel <action_name>'
        rep = self._run_cmd(f"ros2 action cancel {action}", timeout_sec=timeout)
        ok = rep["rc"] == 0
        return {"result": "OK" if ok else "FAIL", **rep}

    def _step_wait_for_result(self, body: Dict[str, Any]) -> Dict[str, Any]:
        """
        Heuristic: check action is still introspectable. The definitive result should be in node logs.
        """
        timeout = int(body.get("timeout_sec", 10))
        action = self.state.get("last_action")
        if not action:
            return {"result": "FAIL", "error": "No prior action_send_goal state (last_action missing)."}

        rep = self._run_cmd(f"ros2 action info {action}", timeout_sec=timeout)
        ok = rep["rc"] == 0
        rep["note"] = "Result codes/succeeded/canceled should be validated from node logs by Functional Evaluator."
        return {"result": "OK" if ok else "FAIL", **rep}

    # ---------- Dispatch ----------

    def _execute_step(self, idx: int, step: Dict[str, Any]) -> Dict[str, Any]:
        if not isinstance(step, dict) or len(step) != 1:
            return {
                "result": "FAIL",
                "error": "Invalid step format. Each step must be a dict with exactly one key.",
                "step": step,
            }

        step_type = next(iter(step.keys()))
        body = step[step_type] or {}
        if not isinstance(body, dict):
            body = {}

        try:
            if step_type == "delay":
                out = self._step_delay(body)
            elif step_type == "passive_run":
                out = self._step_passive_run(body)
            elif step_type == "wait_for_service":
                out = self._step_wait_for_service(body)
            elif step_type == "service_call":
                out = self._step_service_call(body)
            elif step_type == "topic_echo_wait":
                out = self._step_topic_echo_wait(body)
            elif step_type == "action_send_goal":
                out = self._step_action_send_goal(body)
            elif step_type == "wait_for_feedback":
                out = self._step_wait_for_feedback(body)
            elif step_type == "action_cancel_goal":
                out = self._step_action_cancel_goal(body)
            elif step_type == "wait_for_result":
                out = self._step_wait_for_result(body)
            else:
                out = {"result": "FAIL", "error": f"Unknown step type: {step_type}", "body": body}
        except subprocess.TimeoutExpired as e:
            out = {"result": "FAIL", "error": "TIMEOUT", "cmd": getattr(e, "cmd", ""), "timeout_sec": getattr(e, "timeout", None)}
        except Exception as e:
            out = {"result": "FAIL", "error": type(e).__name__, "message": str(e)}

        out["step_index"] = idx
        out["step_type"] = step_type
        return out

    def run(self) -> Dict[str, Any]:
        started_at = self._utc_now()
        status = "PASS"
        failed_step_index: Optional[int] = None
        error_type: Optional[str] = None
        message: str = ""

        steps = self.scenario.get("steps", [])
        if not isinstance(steps, list):
            steps = []

        self._log(f"Start scenario_id={self.scenario_id} steps={len(steps)}")

        for idx, step in enumerate(steps):
            self._log(f"Step {idx}: {list(step.keys())[0] if isinstance(step, dict) and step else step}")
            step_report = self._execute_step(idx, step)
            self.steps_report.append(step_report)

            if step_report.get("result") != "OK":
                status = "FAIL_STEP"
                failed_step_index = idx
                error_type = step_report.get("error") or "FAIL"
                message = step_report.get("message") or step_report.get("stderr") or step_report.get("stdout") or ""
                self._log(f"FAIL_FAST at step {idx}: {error_type}")
                break

        ended_at = self._utc_now()
        scenario_report = {
            "schema_version": "scenario_report.v1",
            "run_id": self.run_id,
            "package_name": self.package_name,
            "scenario_id": self.scenario_id,
            "steps": self.steps_report,
            "scenario_status": {
                "status": "PASS" if status == "PASS" else status,
                "failed_step_index": failed_step_index,
                "error_type": error_type,
                "message": message[:2000],
            },
            "timing": {"started_at_utc": started_at, "ended_at_utc": ended_at},
        }

        self._write_json("scenario_report.json", scenario_report)
        self._log(f"Scenario status: {scenario_report['scenario_status']['status']}")
        return scenario_report

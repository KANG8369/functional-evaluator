
from __future__ import annotations

import json
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Tuple, Optional

class ScenarioStepError(RuntimeError):
    pass

class ScenarioRunner:
    """Executes exactly ONE scenario. Nodes must already be running. FAIL_FAST."""

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

    def _log(self, msg: str) -> None:
        if self.verbose:
            print(f\"[ScenarioRunner] {msg}\", flush=True)
        # state between steps (e.g., goal_id)
        self.state: Dict[str, Any] = {}

    def run(self) -> Dict[str, Any]:
        started_at = self._utc_now()
        status = "PASS"
        failed_step_index = None
        error_type = None
        message = ""

        self._log(f"Start scenario_id={self.scenario_id}, steps={len(self.scenario['steps'])}")

        for idx, step in enumerate(self.scenario["steps"]):
            step_type, body = self._parse_step(step)
            self._log(f"Step {idx}: {step_type}")

            report = self._execute_step(idx, step_type, body)
            self.steps_report.append(report)

            if report["result"]["status"] != "OK":
                status = "FAIL_STEP_TIMEOUT" if report["result"]["status"] == "TIMEOUT" else "FAIL_STEP_ERROR"
                failed_step_index = idx
                error_type = report["result"]["status"]
                message = report["result"].get("error", "")
                self._log(f"FAIL_FAST at step {idx}: {report['result']}")
                break

        ended_at = self._utc_now()
        scenario_report = {
            "schema_version": "scenario_report.v1",
            "run_id": self.run_id,
            "package_name": self.package_name,
            "scenario_id": self.scenario_id,
            "steps": self.steps_report,
            "scenario_status": {
                "status": status,
                "failed_step_index": failed_step_index,
                "error_type": error_type,
                "message": message,
            },
            "timing": {"started_at_utc": started_at, "ended_at_utc": ended_at},
        }
        self._log(f"Scenario status: {scenario_report['scenario_status']['status']}")
        self._write_json("scenario_report.json", scenario_report)
        return scenario_report

    def _execute_step(self, index: int, step_type: str, body: Dict[str, Any]) -> Dict[str, Any]:
        started = self._utc_now()
        timeout = body.get("timeout_sec")

        try:
            handler = getattr(self, f"_handle_{step_type}")
        except AttributeError:
            raise ScenarioStepError(f"Unsupported step type: {step_type}")

        try:
            observations = handler(body)
            result = {"status": "OK", "error": None}
        except subprocess.TimeoutExpired:
            observations = {}
            result = {"status": "TIMEOUT", "error": "step timeout"}
        except Exception as e:
            observations = {}
            result = {"status": "ERROR", "error": str(e)}

        ended = self._utc_now()
        return {
            "index": index,
            "type": step_type,
            "params": body,
            "timing": {"started_at_utc": started, "ended_at_utc": ended, "timeout_sec": timeout},
            "result": result,
            "observations": observations,
        }

    # ---------------- Step handlers ----------------

    def _handle_delay(self, body: Dict[str, Any]) -> Dict[str, Any]:
        seconds = float(body["seconds"])
        time.sleep(seconds)
        return {"slept_sec": seconds, "error": None}

    def _handle_passive_run(self, body: Dict[str, Any]) -> Dict[str, Any]:
        duration = float(body["duration_sec"])
        time.sleep(duration)
        return {"duration_sec": duration, "error": None}

    def _handle_wait_for_service(self, body: Dict[str, Any]) -> Dict[str, Any]:
        service = body["service"]
        timeout = int(body.get("timeout_sec", 5))
        stdout, rc = self._run_cmd(f"ros2 service wait {service}", timeout)
        return {"service_found": rc == 0, "wait_time_sec": timeout, "error": None if rc == 0 else stdout.strip()}

    def _handle_service_call(self, body: Dict[str, Any]) -> Dict[str, Any]:
        service = body["service"]
        request = body["request"]
        srv_type = body.get("type", "example_interfaces/srv/AddTwoInts")
        timeout = int(body.get("timeout_sec", 5))
        cmd = f"ros2 service call {service} {srv_type} \"{request}\""
        stdout, rc = self._run_cmd(cmd, timeout)
        return {
            "request_sent": True,
            "response_received": rc == 0,
            "raw_output": stdout,
            "error": None if rc == 0 else f"return_code={rc}",
        }

    def _handle_topic_echo_wait(self, body: Dict[str, Any]) -> Dict[str, Any]:
        topic = body["topic"]
        min_messages = int(body.get("min_messages", 1))
        timeout = int(body.get("timeout_sec", 5))
        cmd = f"ros2 topic echo -n {min_messages} {topic}"
        stdout, rc = self._run_cmd(cmd, timeout)
        lines = [ln for ln in stdout.splitlines() if ln.strip()]
        return {
            "messages_observed": len(lines),
            "sample_messages": lines[:10],
            "error": None if rc == 0 and len(lines) > 0 else stdout.strip(),
        }

    # -------- Action steps (for fibonacci) --------
    # NOTE: ROS 2 CLI action UX varies slightly across distros/tools.
    # We implement robust best-effort capture and record raw outputs.

    def _handle_action_send_goal(self, body: Dict[str, Any]) -> Dict[str, Any]:
        action = body["action"]  # e.g. /fibonacci
        action_type = body.get("type", "example_interfaces/action/Fibonacci")
        goal = body.get("goal", {})
        timeout = int(body.get("timeout_sec", 10))
        feedback = bool(body.get("feedback", True))

        # ros2 action send_goal <action_name> <action_type> "<goal_yaml>" --feedback
        goal_yaml = json.dumps(goal)
        fb_flag = "--feedback" if feedback else ""
        cmd = f"ros2 action send_goal {action} {action_type} \"{goal_yaml}\" {fb_flag}"
        stdout, rc = self._run_cmd(cmd, timeout)

        # Try to extract goal UUID if present
        goal_id = None
        for line in stdout.splitlines():
            if "Goal ID" in line or "Goal id" in line or "goal_id" in line:
                goal_id = line.strip()
                break

        self.state["last_action_name"] = action
        self.state["last_action_type"] = action_type
        self.state["last_goal_id_raw"] = goal_id
        self.state["last_send_goal_output"] = stdout

        accepted = ("accepted" in stdout.lower()) or (rc == 0)
        rejected = ("rejected" in stdout.lower())

        return {
            "goal_sent": True,
            "goal_accepted": bool(accepted and not rejected),
            "goal_rejected": bool(rejected),
            "goal_id": goal_id,
            "server_available": rc == 0,
            "raw_output": stdout,
            "error": None if rc == 0 else f"return_code={rc}",
        }

    def _handle_action_cancel_goal(self, body: Dict[str, Any]) -> Dict[str, Any]:
        action = body.get("action") or self.state.get("last_action_name")
        timeout = int(body.get("timeout_sec", 5))
        if not action:
            raise ScenarioStepError("No action name available for cancel")

        # Attempt cancel-all on that action name (most robust across CLI versions)
        cmd = f"ros2 action cancel {action}"
        stdout, rc = self._run_cmd(cmd, timeout)

        accepted = ("accepted" in stdout.lower()) or ("canceling" in stdout.lower()) or (rc == 0)
        return {
            "cancel_requested": True,
            "cancel_accepted": bool(accepted),
            "raw_output": stdout,
            "error": None if rc == 0 else f"return_code={rc}",
        }

    def _handle_wait_for_feedback(self, body: Dict[str, Any]) -> Dict[str, Any]:
        # For CLI-based flows, feedback is usually printed by send_goal --feedback.
        # Here we validate feedback presence in last send_goal output if available.
        min_messages = int(body.get("min_messages", 1))
        timeout = int(body.get("timeout_sec", 5))
        time.sleep(min(0.5, timeout))

        out = str(self.state.get("last_send_goal_output", ""))
        # Heuristic: fibonacci feedback commonly contains "feedback"
        count = sum(1 for ln in out.splitlines() if "feedback" in ln.lower())
        return {
            "feedback_messages_observed": count,
            "min_required": min_messages,
            "error": None if count >= min_messages else "no feedback observed in send_goal output; consider node logs",
        }

    def _handle_wait_for_result(self, body: Dict[str, Any]) -> Dict[str, Any]:
        # CLI send_goal prints result at end. We'll parse last_send_goal_output.
        timeout = int(body.get("timeout_sec", 5))
        time.sleep(min(0.5, timeout))
        out = str(self.state.get("last_send_goal_output", ""))

        result_received = ("result" in out.lower()) or ("succeeded" in out.lower()) or ("canceled" in out.lower())
        code = None
        for key in ["SUCCEEDED", "ABORTED", "CANCELED", "succeeded", "aborted", "canceled"]:
            if key.lower() in out.lower():
                code = key.upper()
                break

        return {
            "result_received": bool(result_received),
            "result_code": code,
            "raw_output": out,
            "error": None if result_received else "no result observed in send_goal output; consider node logs",
        }

    # ---------------- Utilities ----------------

    def _run_cmd(self, cmd: str, timeout: int) -> Tuple[str, int]:
        full_cmd = f"{self.ros_env_prefix} {cmd}"
        proc = subprocess.run(
            ["bash", "-lc", full_cmd],
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        return (proc.stdout or "") + (proc.stderr or ""), proc.returncode

    def _parse_step(self, step: Dict[str, Any]) -> Tuple[str, Dict[str, Any]]:
        if len(step) != 1:
            raise ScenarioStepError(f"Invalid step format: {step}")
        step_type = next(iter(step))
        return step_type, step[step_type]

    def _write_json(self, name: str, data: Dict[str, Any]) -> None:
        path = self.artifacts_dir / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")

    @staticmethod
    def _utc_now() -> str:
        return datetime.now(timezone.utc).isoformat(timespec="seconds")

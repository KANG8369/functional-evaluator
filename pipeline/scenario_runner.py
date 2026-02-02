import json
import subprocess
import time
from pathlib import Path

class ScenarioRunner:
    """
    Executes scenario steps defined in criteria YAML while nodes are running.
    Produces scenario_report.json with per-step observations.

    Supported step types (keys):
      - passive_run: {duration_sec}
      - delay: {duration_sec}
      - topic_echo_wait: {topic, min_messages, timeout_sec}
      - wait_for_service: {service, timeout_sec}
      - service_call: {service_type, service, args, timeout_sec}
      - action_send_goal: {action_type, action, goal, timeout_sec}
      - action_cancel_goal: {action, timeout_sec}
    """

    def __init__(self, scenario, scenario_id, session_dir, ros_env_prefix_cmd, verbose=False, fail_fast=True):
        self.scenario = scenario
        self.scenario_id = scenario_id
        self.session_dir = Path(session_dir)
        self.verbose = verbose
        self.fail_fast = fail_fast
        self.ros_env_prefix_cmd = ros_env_prefix_cmd  # callable(cmd_str)->list

        self.steps_report = []

    def _log(self, msg):
        if self.verbose:
            print("[ScenarioRunner] " + msg, flush=True)

    def _run_cmd(self, cmd_str, timeout=None):
        cmd = self.ros_env_prefix_cmd(cmd_str)
        p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=timeout)
        return p.returncode, p.stdout, p.stderr

    def _step_passive(self, body):
        dur = float(body.get("duration_sec", 1))
        self._log("passive_run {}s".format(dur))
        time.sleep(dur)
        return {"status": "OK", "duration_sec": dur}

    def _step_delay(self, body):
        return self._step_passive(body)

    def _step_topic_echo_wait(self, body):
        topic = body.get("topic", "")
        min_messages = int(body.get("min_messages", 1))
        timeout_sec = int(body.get("timeout_sec", 5))
        if not topic:
            return {"status": "ERROR", "error": "missing topic"}
        # Use ros2 topic echo with -n
        cmd = "ros2 topic echo -n {} {} --once".format(min_messages, topic)
        self._log("topic_echo_wait {} (n={}, timeout={}s)".format(topic, min_messages, timeout_sec))
        try:
            rc, out, err = self._run_cmd(cmd, timeout=timeout_sec)
        except subprocess.TimeoutExpired:
            return {"status": "TIMEOUT", "topic": topic, "timeout_sec": timeout_sec}
        if rc != 0:
            return {"status": "ERROR", "topic": topic, "rc": rc, "stderr": err[-2000:]}
        return {"status": "OK", "topic": topic, "stdout_tail": out[-2000:]}

    def _step_wait_for_service(self, body):
        srv = body.get("service", "")
        timeout_sec = int(body.get("timeout_sec", 5))
        if not srv:
            return {"status": "ERROR", "error": "missing service"}
        cmd = "ros2 service list"
        self._log("wait_for_service {} (timeout={}s)".format(srv, timeout_sec))
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            rc, out, err = self._run_cmd(cmd, timeout=timeout_sec)
            if rc == 0 and srv in out.splitlines():
                return {"status": "OK", "service": srv}
            time.sleep(0.2)
        return {"status": "TIMEOUT", "service": srv, "timeout_sec": timeout_sec}

    def _step_service_call(self, body):
        srv_type = body.get("service_type", "")
        srv = body.get("service", "")
        args = body.get("args", "")
        timeout_sec = int(body.get("timeout_sec", 5))
        if not (srv_type and srv):
            return {"status": "ERROR", "error": "missing service_type or service"}
        cmd = "ros2 service call {} {} {}".format(srv, srv_type, args)
        self._log("service_call {}".format(srv))
        try:
            rc, out, err = self._run_cmd(cmd, timeout=timeout_sec)
        except subprocess.TimeoutExpired:
            return {"status": "TIMEOUT", "service": srv, "timeout_sec": timeout_sec}
        if rc != 0:
            return {"status": "ERROR", "service": srv, "rc": rc, "stderr": err[-2000:], "stdout_tail": out[-2000:]}
        return {"status": "OK", "service": srv, "stdout_tail": out[-2000:]}

    def _step_action_send_goal(self, body):
        act_type = body.get("action_type", "")
        act = body.get("action", "")
        goal = body.get("goal", "")
        timeout_sec = int(body.get("timeout_sec", 8))
        if not (act_type and act and goal):
            return {"status": "ERROR", "error": "missing action_type/action/goal"}
        cmd = "ros2 action send_goal {} {} '{}'".format(act, act_type, goal)
        self._log("action_send_goal {}".format(act))
        try:
            rc, out, err = self._run_cmd(cmd, timeout=timeout_sec)
        except subprocess.TimeoutExpired:
            return {"status": "TIMEOUT", "action": act, "timeout_sec": timeout_sec}
        if rc != 0:
            return {"status": "ERROR", "action": act, "rc": rc, "stderr": err[-2000:], "stdout_tail": out[-2000:]}
        return {"status": "OK", "action": act, "stdout_tail": out[-2000:]}

    def _step_action_cancel_goal(self, body):
        act = body.get("action", "")
        timeout_sec = int(body.get("timeout_sec", 5))
        if not act:
            return {"status": "ERROR", "error": "missing action"}
        cmd = "ros2 action cancel {} --all".format(act)
        self._log("action_cancel_goal {}".format(act))
        try:
            rc, out, err = self._run_cmd(cmd, timeout=timeout_sec)
        except subprocess.TimeoutExpired:
            return {"status": "TIMEOUT", "action": act, "timeout_sec": timeout_sec}
        if rc != 0:
            return {"status": "ERROR", "action": act, "rc": rc, "stderr": err[-2000:], "stdout_tail": out[-2000:]}
        return {"status": "OK", "action": act, "stdout_tail": out[-2000:]}

    def run(self):
        steps = self.scenario.get("steps", [])
        self._log("Start scenario_id={} steps={}".format(self.scenario_id, len(steps)))
        scenario_status = {"status": "PASS"}

        for idx, step in enumerate(steps):
            if not isinstance(step, dict) or len(step) != 1:
                rep = {"step_index": idx, "status": "ERROR", "error": "invalid step format", "raw": step}
                self.steps_report.append(rep)
                scenario_status = {"status": "FAIL_STEP_ERROR", "failed_step": idx}
                if self.fail_fast:
                    break
                continue

            step_type = list(step.keys())[0]
            body = step[step_type] or {}
            self._log("Step {}: {}".format(idx, step_type))

            try:
                if step_type == "passive_run":
                    obs = self._step_passive(body)
                elif step_type == "delay":
                    obs = self._step_delay(body)
                elif step_type == "topic_echo_wait":
                    obs = self._step_topic_echo_wait(body)
                elif step_type == "wait_for_service":
                    obs = self._step_wait_for_service(body)
                elif step_type == "service_call":
                    obs = self._step_service_call(body)
                elif step_type == "action_send_goal":
                    obs = self._step_action_send_goal(body)
                elif step_type == "action_cancel_goal":
                    obs = self._step_action_cancel_goal(body)
                else:
                    obs = {"status": "ERROR", "error": "unknown step_type", "step_type": step_type}
            except Exception as e:
                obs = {"status": "ERROR", "error": str(e), "step_type": step_type}

            rep = {"step_index": idx, "step_type": step_type, "observation": obs}
            self.steps_report.append(rep)

            if obs.get("status") != "OK":
                scenario_status = {"status": "FAIL_STEP_ERROR", "failed_step": idx, "message": obs}
                if self.fail_fast:
                    self._log("FAIL_FAST at step {}: {}".format(idx, obs.get("status")))
                    break

        report = {
            "scenario_id": self.scenario_id,
            "scenario_status": scenario_status,
            "steps": self.steps_report,
        }
        (self.session_dir / "scenario_report.json").write_text(json.dumps(report, indent=2))
        self._log("Scenario status: {}".format(scenario_status.get("status")))
        return report

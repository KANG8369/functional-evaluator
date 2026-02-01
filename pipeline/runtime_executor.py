
from __future__ import annotations

import json
import os
import signal
import subprocess
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from pipeline.scenario_runner import ScenarioRunner
from pipeline.artifacts import write_json, write_meta_json

class RuntimeExecutor:
    """Launches all nodes in a package concurrently, runs ONE scenario session, captures logs and probes."""

    def __init__(
        self,
        *,
        package_name: str,
        scenario: Dict[str, Any],
        workspace_root: Path,
        artifacts_root: Path,
        run_id: str,
        ros_distro: str = "humble",
        startup_grace_sec: int = 2,
        teardown_grace_sec: int = 3,
    ) -> None:
        self.package_name = package_name
        self.scenario = scenario
        self.scenario_id = scenario["id"]
        self.workspace_root = workspace_root
        self.artifacts_root = artifacts_root
        self.run_id = run_id
        self.ros_distro = ros_distro
        self.startup_grace_sec = startup_grace_sec
        self.teardown_grace_sec = teardown_grace_sec

        self.ros_domain_id = self._alloc_ros_domain_id()

        self.session_dir = self.artifacts_root / self.run_id / self.package_name / self.scenario_id
        self.probes_dir = self.session_dir / "probes"
        self.node_logs_dir = self.session_dir / "node_logs"

        self.processes: List[subprocess.Popen] = []
        self.executables: List[Dict[str, Any]] = []

    def run(self) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        started_at = self._utc_now()
        self.probes_dir.mkdir(parents=True, exist_ok=True)
        self.node_logs_dir.mkdir(parents=True, exist_ok=True)

        write_meta_json(session_dir=self.session_dir, ros_distro=self.ros_distro, ros_domain_id=self.ros_domain_id, ws_root=self.workspace_root)

        runtime_status = "OK"
        runtime_error = ""

        try:
            self._launch_nodes()
            time.sleep(self.startup_grace_sec)

            runner = ScenarioRunner(
                run_id=self.run_id,
                package_name=self.package_name,
                scenario=self.scenario,
                artifacts_dir=self.session_dir,
                ros_env_prefix=self._ros_env_prefix(),
            )
            scenario_report = runner.run()

            self._collect_probes()

            # if any process died early, mark crash
            if any(p.poll() is not None for p in self.processes):
                runtime_status = "CRASH_DETECTED"

        except Exception as e:
            runtime_status = "UNKNOWN_ERROR"
            runtime_error = str(e)
            scenario_report = {
                "schema_version": "scenario_report.v1",
                "run_id": self.run_id,
                "package_name": self.package_name,
                "scenario_id": self.scenario_id,
                "steps": [],
                "scenario_status": {
                    "status": "FAIL_STEP_ERROR",
                    "failed_step_index": None,
                    "error_type": type(e).__name__,
                    "message": runtime_error,
                },
                "timing": {"started_at_utc": started_at, "ended_at_utc": self._utc_now()},
            }
            write_json(self.session_dir / "scenario_report.json", scenario_report)

        finally:
            self._teardown()
            self._fill_exit_info()

        ended_at = self._utc_now()

        runtime_report = {
            "schema_version": "runtime_report.v1",
            "run_id": self.run_id,
            "package_name": self.package_name,
            "scenario_id": self.scenario_id,
            "timing": {
                "started_at_utc": started_at,
                "ended_at_utc": ended_at,
                "startup_grace_sec": self.startup_grace_sec,
                "teardown_grace_sec": self.teardown_grace_sec,
            },
            "executables": self.executables,
            "process_control": {
                "launcher": "script",
                "shell": "bash -lc",
                "sourced": [
                    f"/opt/ros/{self.ros_distro}/setup.bash",
                    str(self.workspace_root / "install" / "setup.bash"),
                ],
                "stop_method": "sigint_then_sigkill",
            },
            "runtime_status": {
                "status": runtime_status,
                "crash_detected": runtime_status != "OK",
                "notes": runtime_error,
            },
            "environment": {"ros_distro": self.ros_distro, "ros_domain_id": self.ros_domain_id},
        }

        write_json(self.session_dir / "runtime_report.json", runtime_report)
        return runtime_report, scenario_report

    # ---------------- internals ----------------

    def _launch_nodes(self) -> None:
        execs = self._discover_executables()
        if not execs:
            raise RuntimeError(f"No executables discovered for package: {self.package_name}")

        for exe in execs:
            log_path = self.node_logs_dir / f"{exe}.txt"
            cmd = (
                f"{self._ros_env_prefix()} "
                f"script -q -c \"ros2 run {self.package_name} {exe}\" "
                f"{log_path}"
            )
            proc = subprocess.Popen(
                ["bash", "-lc", cmd],
                preexec_fn=os.setsid,
                env=self._runtime_env(),
            )
            self.processes.append(proc)
            self.executables.append({
                "name": exe,
                "pid": proc.pid,
                "log_path": str(log_path),
                "start_ok": True,
                "exit": {"exited": False, "exit_code": None, "signal": None, "ended_early": False},
            })

    def _teardown(self) -> None:
        # SIGINT
        for proc in self.processes:
            if proc.poll() is None:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)

        time.sleep(self.teardown_grace_sec)

        # SIGKILL
        for proc in self.processes:
            if proc.poll() is None:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)

    def _fill_exit_info(self) -> None:
        for exe, proc in zip(self.executables, self.processes):
            rc = proc.poll()
            exe["exit"] = {
                "exited": rc is not None,
                "exit_code": rc,
                "signal": None,
                "ended_early": False,
            }

    def _collect_probes(self) -> None:
        probes = {
            "ros_node_list.txt": "ros2 node list",
            "ros_topic_list.txt": "ros2 topic list",
            "ros_service_list.txt": "ros2 service list",
            "ros_action_list.txt": "ros2 action list",
        }
        for fname, cmd in probes.items():
            out, _ = self._run_cmd(cmd, timeout=5)
            (self.probes_dir / fname).write_text(out, encoding="utf-8")

    def _discover_executables(self) -> List[str]:
        out, rc = self._run_cmd(f"ros2 pkg executables {self.package_name}", timeout=5)
        if rc != 0:
            return []
        return [line.split()[-1] for line in out.splitlines() if line.strip()]

    def _run_cmd(self, cmd: str, timeout: int) -> Tuple[str, int]:
        full_cmd = f"{self._ros_env_prefix()} {cmd}"
        proc = subprocess.run(
            ["bash", "-lc", full_cmd],
            capture_output=True,
            text=True,
            timeout=timeout,
            env=self._runtime_env(),
        )
        return (proc.stdout or "") + (proc.stderr or ""), proc.returncode

    def _runtime_env(self) -> Dict[str, str]:
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = str(self.ros_domain_id)
        return env

    def _ros_env_prefix(self) -> str:
        return (
            f"source /opt/ros/{self.ros_distro}/setup.bash && "
            f"source {self.workspace_root}/install/setup.bash &&"
        )

    @staticmethod
    def _alloc_ros_domain_id() -> int:
        return int(uuid.uuid4().int % 200) + 1

    @staticmethod
    def _utc_now() -> str:
        return datetime.now(timezone.utc).isoformat(timespec="seconds")

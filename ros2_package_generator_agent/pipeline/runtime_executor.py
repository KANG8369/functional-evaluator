from __future__ import annotations

import json
import os
import signal
import subprocess
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Tuple

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
        verbose: bool = True,
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
        self.verbose = verbose

        self.ros_domain_id = self._alloc_ros_domain_id()

        self.session_dir = self.artifacts_root / self.run_id / self.package_name / self.scenario_id
        self.probes_dir = self.session_dir / "probes"
        self.node_logs_dir = self.session_dir / "node_logs"

        self.processes: List[subprocess.Popen] = []
        self.executables: List[Dict[str, Any]] = []

    def _log(self, msg: str) -> None:
        if self.verbose:
            print(f"[RuntimeExecutor] {msg}", flush=True)

    def _utc_now(self) -> str:
        return datetime.now(timezone.utc).isoformat(timespec="seconds")

    def _alloc_ros_domain_id(self) -> int:
        # randomized to reduce collisions if multiple runs
        return int(uuid.uuid4().int % 101) + 10

    def _ros_env_prefix(self) -> str:
        ros_setup = f"/opt/ros/{self.ros_distro}/setup.bash"
        ws_setup = str(self.workspace_root / "install" / "setup.bash")
        # Do not use 'set -u' here to avoid AMENT_TRACE_SETUP_FILES unbound issues.
        return f"set -e; source {ros_setup}; source {ws_setup}; export ROS_DOMAIN_ID={self.ros_domain_id}"

    def _discover_executables(self) -> List[Tuple[str, str]]:
        # returns list of (pkg, exe)
        cmd = f"{self._ros_env_prefix()}; ros2 pkg executables {self.package_name}"
        proc = subprocess.run(["bash", "-lc", cmd], text=True, capture_output=True)
        if proc.returncode != 0:
            raise RuntimeError(f"ros2 pkg executables failed: {proc.stderr.strip()}")
        pairs: List[Tuple[str, str]] = []
        for line in (proc.stdout or "").splitlines():
            parts = line.strip().split()
            if len(parts) >= 2:
                pairs.append((parts[0], parts[1]))
        return pairs

    def _launch_nodes(self) -> None:
        pairs = self._discover_executables()
        if not pairs:
            raise RuntimeError(f"No executables found for package '{self.package_name}'.")
        for pkg, exe in pairs:
            log_path = self.node_logs_dir / f"{exe}.txt"
            # Use script to capture terminal-like output
            cmd = (
                f"{self._ros_env_prefix()}; "
                f"script -q -c \"ros2 run {pkg} {exe}\" \"{log_path.as_posix()}\""
            )
            p = subprocess.Popen(["bash", "-lc", cmd], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.processes.append(p)
            self.executables.append({
                "package": pkg,
                "executable": exe,
                "cmd": f"ros2 run {pkg} {exe}",
                "log_file": str(log_path),
                "pid": p.pid,
            })

    def _collect_probes(self) -> None:
        self.probes_dir.mkdir(parents=True, exist_ok=True)

        def run_probe(name: str, cmd: str) -> None:
            full = f"{self._ros_env_prefix()}; {cmd}"
            proc = subprocess.run(["bash", "-lc", full], text=True, capture_output=True)
            out = (proc.stdout or "") + ("\n" + proc.stderr if proc.stderr else "")
            (self.probes_dir / name).write_text(out, encoding="utf-8")

        run_probe("node_list.txt", "ros2 node list")
        run_probe("topic_list.txt", "ros2 topic list")
        run_probe("service_list.txt", "ros2 service list")
        run_probe("action_list.txt", "ros2 action list")

    def _teardown(self) -> None:
        # SIGINT then SIGKILL
        for p in self.processes:
            if p.poll() is None:
                try:
                    os.kill(p.pid, signal.SIGINT)
                except Exception:
                    pass
        t_end = time.time() + float(self.teardown_grace_sec)
        while time.time() < t_end:
            if all(p.poll() is not None for p in self.processes):
                return
            time.sleep(0.1)
        for p in self.processes:
            if p.poll() is None:
                try:
                    os.kill(p.pid, signal.SIGKILL)
                except Exception:
                    pass

    def _fill_exit_info(self) -> None:
        for exe in self.executables:
            pid = exe.get("pid")
            proc = next((p for p in self.processes if p.pid == pid), None)
            if proc is None:
                continue
            exe["returncode"] = proc.poll()

    def run(self) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        started_at = self._utc_now()
        self._log(f"Run start run_id={self.run_id} pkg={self.package_name} scenario={self.scenario_id} ROS_DOMAIN_ID={self.ros_domain_id}")

        self.probes_dir.mkdir(parents=True, exist_ok=True)
        self.node_logs_dir.mkdir(parents=True, exist_ok=True)
        write_meta_json(session_dir=self.session_dir, ros_distro=self.ros_distro, ros_domain_id=self.ros_domain_id, ws_root=self.workspace_root)

        runtime_status = "OK"
        runtime_error = ""

        try:
            self._log("Launching nodes (concurrent)...")
            self._launch_nodes()
            self._log(f"Startup grace: {self.startup_grace_sec}s")
            time.sleep(self.startup_grace_sec)

            self._log("Executing scenario steps...")
            runner = ScenarioRunner(
                run_id=self.run_id,
                package_name=self.package_name,
                scenario=self.scenario,
                artifacts_dir=self.session_dir,
                ros_env_prefix=self._ros_env_prefix(),
                verbose=self.verbose,
            )
            scenario_report = runner.run()

            self._log("Collecting ROS graph probes...")
            self._collect_probes()

            # crash detection
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
            self._log("Tearing down (SIGINT -> SIGKILL)...")
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

import json
import signal
import subprocess
import time
import uuid
from pathlib import Path

from pipeline.scenario_runner import ScenarioRunner


class RuntimeExecutor:
    def __init__(
        self,
        package_name,
        scenario,
        workspace_root,
        artifacts_root,
        ros_distro,
        startup_grace_sec=2,
        runtime_budget_sec=10,
        teardown_grace_sec=3,
        verbose=False,
    ):
        self.package_name = package_name
        self.scenario = scenario
        self.workspace_root = Path(workspace_root)
        self.artifacts_root = Path(artifacts_root)
        self.ros_distro = ros_distro
        self.startup_grace_sec = int(startup_grace_sec)
        self.runtime_budget_sec = int(runtime_budget_sec)
        self.teardown_grace_sec = int(teardown_grace_sec)
        self.verbose = verbose

        self.run_id = time.strftime("%Y%m%dT%H%M%SZ") + "_" + uuid.uuid4().hex[:6]
        self.scenario_id = scenario.get("id", "unknown_scenario")
        self.ros_domain_id = int(uuid.uuid4().int % 200) + 1

        self.session_dir = self.artifacts_root / self.run_id / self.package_name / self.scenario_id
        self.node_logs_dir = self.session_dir / "node_logs"
        self.probes_dir = self.session_dir / "probes"
        self.processes = []

    def _log(self, msg):
        if self.verbose:
            print("[RuntimeExecutor] " + msg, flush=True)

    def _ros_cmd(self, cmd_str):
        ros_setup = "/opt/ros/{}/setup.bash".format(self.ros_distro)
        ws_setup = (self.workspace_root / "install" / "setup.bash").as_posix()
        full = (
            "source {ros} && source {ws} && export ROS_DOMAIN_ID={dom} && {cmd}"
        ).format(ros=ros_setup, ws=ws_setup, dom=self.ros_domain_id, cmd=cmd_str)
        return ["bash", "-lc", full]

    def _discover_executables(self):
        p = subprocess.run(self._ros_cmd("ros2 pkg executables {}".format(self.package_name)),
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if p.returncode != 0:
            raise RuntimeError("ros2 pkg executables failed: " + p.stderr)
        exes = []
        for line in p.stdout.splitlines():
            parts = line.strip().split()
            if len(parts) == 2:
                exes.append(parts[1])
        return exes, p.stdout + "\n" + p.stderr

    def _launch_nodes(self, executables):
        self.node_logs_dir.mkdir(parents=True, exist_ok=True)
        for exe in executables:
            log_path = self.node_logs_dir / "{}.txt".format(exe)
            cmd = self._ros_cmd('script -q -c "ros2 run {} {}" "{}"'.format(self.package_name, exe, log_path))
            proc = subprocess.Popen(cmd)
            self.processes.append(proc)

    def _collect_probes(self):
        self.probes_dir.mkdir(parents=True, exist_ok=True)
        probes = {
            "nodes": "ros2 node list",
            "topics": "ros2 topic list",
            "services": "ros2 service list",
            "actions": "ros2 action list",
        }
        for name, cmd in probes.items():
            p = subprocess.run(self._ros_cmd(cmd), stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            (self.probes_dir / "{}.txt".format(name)).write_text(p.stdout + "\n" + p.stderr)

    def _teardown(self):
        self._log("Tearing down (SIGINT -> SIGKILL)...")
        for p in self.processes:
            try:
                p.send_signal(signal.SIGINT)
            except Exception:
                pass
        time.sleep(self.teardown_grace_sec)
        for p in self.processes:
            if p.poll() is None:
                try:
                    p.kill()
                except Exception:
                    pass

    def run(self):
        self._log("Run start run_id={} pkg={} scenario={} ROS_DOMAIN_ID={}".format(
            self.run_id, self.package_name, self.scenario_id, self.ros_domain_id
        ))
        self.session_dir.mkdir(parents=True, exist_ok=True)

        runtime_status = {"status": "OK"}
        exec_list = []
        exec_raw = ""
        scenario_report = None

        try:
            exec_list, exec_raw = self._discover_executables()
            if not exec_list:
                raise RuntimeError("No executables found for package")
            self._log("Launching nodes (concurrent)...")
            self._launch_nodes(exec_list)

            time.sleep(self.startup_grace_sec)

            # probes before scenario
            self._collect_probes()

            # run scenario steps (FAIL_FAST)
            runner = ScenarioRunner(
                scenario=self.scenario,
                scenario_id=self.scenario_id,
                session_dir=self.session_dir,
                ros_env_prefix_cmd=self._ros_cmd,
                verbose=self.verbose,
                fail_fast=True,
            )
            scenario_report = runner.run()

            # allow nodes to keep running a bit (optional budget)
            time.sleep(max(0, self.runtime_budget_sec - self.startup_grace_sec))

        except Exception as e:
            runtime_status = {"status": "UNKNOWN_ERROR", "error": str(e)}

        finally:
            self._teardown()

        runtime_report = {
            "run_id": self.run_id,
            "package": self.package_name,
            "scenario_id": self.scenario_id,
            "ros_domain_id": self.ros_domain_id,
            "executables": exec_list,
            "executables_raw": exec_raw[-4000:],
            "runtime_status": runtime_status,
        }
        (self.session_dir / "runtime_report.json").write_text(json.dumps(runtime_report, indent=2))

        if scenario_report is None:
            scenario_report = {
                "scenario_id": self.scenario_id,
                "scenario_status": {"status": "FAIL_STEP_ERROR", "message": "scenario did not run"},
                "steps": [],
            }
            (self.session_dir / "scenario_report.json").write_text(json.dumps(scenario_report, indent=2))

        return runtime_report, scenario_report

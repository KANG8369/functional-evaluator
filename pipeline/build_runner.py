
import subprocess
from pathlib import Path

def build_workspace(ws_root, ros_distro):
    log = Path(ws_root) / "build_logs" / "build.txt"
    log.parent.mkdir(parents=True, exist_ok=True)
    proc = subprocess.run(["bash", "scripts/build_ros2_ws.sh", str(ws_root), ros_distro])
    return proc.returncode, log

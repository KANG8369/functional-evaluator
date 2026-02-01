
from __future__ import annotations

import subprocess
from pathlib import Path
from typing import Tuple

def build_workspace(*, ws_root: Path, ros_distro: str, build_script: Path) -> Tuple[int, Path]:
    if not build_script.exists():
        raise FileNotFoundError(f"Build script not found: {build_script}")
    log_path = ws_root / "build_logs" / "build.txt"
    proc = subprocess.run(["bash", str(build_script), str(ws_root), ros_distro], text=True)
    return proc.returncode, log_path


from __future__ import annotations

import subprocess
from pathlib import Path
from typing import Iterable, Tuple

def ensure_pkg_created(*, ws_root: Path, package_name: str, ros_distro: str) -> Path:
    src_root = ws_root / "src"
    src_root.mkdir(parents=True, exist_ok=True)

    pkg_root = src_root / package_name
    if pkg_root.exists():
        return pkg_root

    cmd = (
        "set -eo pipefail; "
        f"source /opt/ros/{ros_distro}/setup.bash; "
        f"ros2 pkg create {package_name} "
        "--build-type ament_cmake "
        "--dependencies rclcpp "
        f"--destination-directory '{src_root.as_posix()}'"
    )
    subprocess.run(["bash", "-lc", cmd], check=True)
    return pkg_root

def write_files(*, pkg_root: Path, files: Iterable[Tuple[str, str]]) -> None:
    for rel, content in files:
        p = pkg_root / Path(rel)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(content, encoding="utf-8")

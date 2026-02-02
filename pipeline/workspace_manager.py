import subprocess
from pathlib import Path

def ensure_pkg_created(ws_root=None, package_name=None, ros_distro="humble", pkg_name=None, **kwargs):
    """
    Ensure <ws_root>/src/<package_name> exists by calling ros2 pkg create.
    Accepts both package_name and pkg_name for compatibility.
    """
    if ws_root is None:
        raise TypeError("ws_root is required")
    name = package_name if package_name is not None else pkg_name
    if not name:
        raise TypeError("package_name (or pkg_name) is required")
    ws_root = Path(ws_root)
    src_root = ws_root / "src"
    src_root.mkdir(parents=True, exist_ok=True)

    pkg_root = src_root / name
    if pkg_root.exists():
        return pkg_root

    cmd = (
        "set -eo pipefail; "
        "source /opt/ros/{}/setup.bash; "
        "ros2 pkg create {} --build-type ament_cmake --dependencies rclcpp "
        "--destination-directory '{}'".format(ros_distro, name, src_root.as_posix())
    )
    subprocess.run(["bash", "-lc", cmd], check=True)
    return pkg_root


def write_files(pkg_root=None, files=None, **kwargs):
    """
    Write generated files under pkg_root. `files` can be:
      - list of objects with .path and .content
      - list of dicts with keys path/content
      - list of (path, content) tuples
    Paths are relative to the package root.
    """
    if pkg_root is None:
        raise TypeError("pkg_root is required")
    if files is None:
        raise TypeError("files is required")

    pkg_root = Path(pkg_root)
    for f in files:
        if isinstance(f, tuple) and len(f) == 2:
            rel, content = f
        elif isinstance(f, dict):
            rel, content = f.get("path"), f.get("content")
        else:
            rel = getattr(f, "path", None)
            content = getattr(f, "content", None)
        if rel is None or content is None:
            raise ValueError("Invalid file entry: {}".format(f))
        p = pkg_root / Path(rel)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(content, encoding="utf-8")

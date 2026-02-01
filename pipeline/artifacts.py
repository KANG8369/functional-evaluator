
from __future__ import annotations

import json
import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict

@dataclass(frozen=True)
class SessionPaths:
    run_id: str
    session_dir: Path
    probes_dir: Path
    node_logs_dir: Path

def make_run_id() -> str:
    ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return f"{ts}_{uuid.uuid4().hex[:6]}"

def session_paths(*, artifacts_root: Path, run_id: str, package_name: str, scenario_id: str) -> SessionPaths:
    session_dir = artifacts_root / run_id / package_name / scenario_id
    return SessionPaths(
        run_id=run_id,
        session_dir=session_dir,
        probes_dir=session_dir / "probes",
        node_logs_dir=session_dir / "node_logs",
    )

def write_json(path: Path, data: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")

def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")

def write_meta_json(*, session_dir: Path, ros_distro: str, ros_domain_id: int, ws_root: Path) -> None:
    meta = {
        "schema_version": "evidence.v1",
        "run_id": session_dir.parents[2].name,
        "created_at_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "ros": {"distro": ros_distro, "ros_domain_id": ros_domain_id},
        "workspace": {
            "ws_root": str(ws_root),
            "install_setup": str(ws_root / "install" / "setup.bash"),
        },
    }
    write_json(session_dir / "meta.json", meta)

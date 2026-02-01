
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

from pipeline.orchestrator import run_pipeline

def main() -> None:
    parser = argparse.ArgumentParser(description="ROS2 package generator pipeline")
    parser.add_argument("--max-attempts", type=int, default=4)
    parser.add_argument("--evaluator-max-chars", type=int, default=20000)
    parser.add_argument("--stop-on-first-fail", action="store_true", default=True)
    args = parser.parse_args()

    if "OPENAI_API_KEY" not in os.environ:
        raise RuntimeError("Set OPENAI_API_KEY env variable.")

    project_root = Path(__file__).parent
    ros_distro = os.environ.get("ROS_DISTRO", "humble")

    build_report, functional_report = run_pipeline(
        project_root=project_root,
        ros_distro=ros_distro,
        max_attempts=args.max_attempts,
        evaluator_max_chars=args.evaluator_max_chars,
        stop_on_first_fail=True,
    )

    reports_dir = project_root / "reports"
    reports_dir.mkdir(parents=True, exist_ok=True)
    (reports_dir / "build_report.json").write_text(json.dumps(build_report, ensure_ascii=False, indent=2), encoding="utf-8")
    (reports_dir / "functional_report.json").write_text(json.dumps(functional_report, ensure_ascii=False, indent=2), encoding="utf-8")

    print("Build report:", reports_dir / "build_report.json")
    print("Functional report:", reports_dir / "functional_report.json")

if __name__ == "__main__":
    main()


import argparse
from pathlib import Path
from pipeline.orchestrator import run_pipeline

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--max-attempts", type=int, default=4)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    project_root = Path(__file__).parent
    build_report, functional_report = run_pipeline(
        project_root=project_root,
        max_attempts=args.max_attempts,
        verbose=args.verbose,
    )

    print("Build report:", project_root / "reports" / "build_report.json")
    print("Functional report:", project_root / "reports" / "functional_report.json")

if __name__ == "__main__":
    main()

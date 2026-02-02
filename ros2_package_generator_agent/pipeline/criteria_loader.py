
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml

@dataclass(frozen=True)
class Criteria:
    task_id: str
    raw: Dict[str, Any]
    scenarios: List[Dict[str, Any]]

def load_criteria(criteria_path: Path, task_id: str) -> Criteria:
    if not criteria_path.exists():
        raise FileNotFoundError(f"Criteria YAML not found: {criteria_path}")

    raw = yaml.safe_load(criteria_path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError(f"Criteria YAML must be a dict: {criteria_path}")

    scenarios = raw.get("scenarios")
    if not isinstance(scenarios, list) or not scenarios:
        raise ValueError(f"'scenarios' must be a non-empty list in {criteria_path}")

    for s in scenarios:
        if not isinstance(s, dict):
            raise ValueError(f"Scenario must be a dict in {criteria_path}: {s}")
        if not s.get("id") or not isinstance(s["id"], str):
            raise ValueError(f"Scenario missing string 'id' in {criteria_path}: {s}")
        if "steps" not in s or not isinstance(s["steps"], list) or not s["steps"]:
            raise ValueError(f"Scenario '{s.get('id')}' missing non-empty 'steps' list in {criteria_path}")

        # each step should be single-key dict: {step_type: {...}}
        for step in s["steps"]:
            if not isinstance(step, dict) or len(step) != 1:
                raise ValueError(f"Invalid step format in scenario '{s['id']}': {step}")

    return Criteria(task_id=task_id, raw=raw, scenarios=scenarios)

from __future__ import annotations
from pathlib import Path
import json
from typing import Tuple

BUDGET_FILE = "_frame_budget.json"

def _load_budget(root: Path) -> dict:
    f = root / "frames_sample" / BUDGET_FILE
    if f.exists():
        return json.loads(f.read_text(encoding="utf-8"))
    return {"fail_left": 10, "succ_left": 10}

def _save_budget(root: Path, budget: dict) -> None:
    d = root / "frames_sample"
    d.mkdir(parents=True, exist_ok=True)
    (d / BUDGET_FILE).write_text(json.dumps(budget, indent=2), encoding="utf-8")

def should_save_frames(run_root: Path, outcome: str) -> Tuple[bool, Path]:
    """
    outcome ∈ {"success", "collision_human", "other"}
    Returns (save?, frames_root_dir).
    """
    budget = _load_budget(run_root)
    if outcome == "success":
        if budget.get("succ_left", 0) > 0:
            budget["succ_left"] -= 1
            _save_budget(run_root, budget)
            return True, run_root / "frames_sample"
        return False, run_root / "frames_sample"
    # treat all non-success as a "failure" for sampling purposes
    if budget.get("fail_left", 0) > 0:
        budget["fail_left"] -= 1
        _save_budget(run_root, budget)
        return True, run_root / "frames_sample"
    return False, run_root / "frames_sample"

def set_initial_budget(run_root: Path, k_fail: int = 10, k_succ: int = 10) -> None:
    _save_budget(run_root, {"fail_left": int(k_fail), "succ_left": int(k_succ)})

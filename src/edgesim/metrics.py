from __future__ import annotations
from pathlib import Path
from typing import Dict, Any, List
import csv
import json

def _read_index(per_run_dir: Path) -> List[Dict[str, Any]]:
	index = per_run_dir / "index.csv"
	if not index.exists():
		return []
	out: List[Dict[str, Any]] = []
	with open(index, "r", newline="", encoding="utf-8") as f:
		for i, row in enumerate(csv.reader(f)):
			if i == 0:  # header
				continue
			run_id, success, time_s, steps = row
			out.append({"run_id": run_id, "success": (success.lower() == "true"), "time_s": float(time_s), "steps": int(steps)})
	return out

def aggregate(per_run_dir: Path) -> Dict[str, Any]:
	rows = _read_index(per_run_dir)
	if not rows:
		return {"runs": 0, "successes": 0, "failures": 0, "avg_time": 0.0, "avg_steps": 0}
	runs = len(rows)
	successes = sum(1 for r in rows if r["success"])
	failures = runs - successes
	avg_time = sum(r["time_s"] for r in rows) / runs
	avg_steps = int(round(sum(r["steps"] for r in rows) / runs))
	return {"runs": runs, "successes": successes, "failures": failures, "avg_time": avg_time, "avg_steps": avg_steps}

def write_summary(run_dir: Path, summary: Dict[str, Any]) -> None:
	(run_dir / "summary.json").write_text(json.dumps(summary, indent=2))

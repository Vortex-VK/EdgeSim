from __future__ import annotations
from typing import Dict, Any, List
from pathlib import Path
import csv
import json
import time
import numpy as np

from .io_utils import ensure_dir, write_json
from .sim_one import run_one

def _per_run_dir(root: Path, k: int) -> Path:
	rd = root / "per_run" / f"run_{k:04d}"
	ensure_dir(rd)
	return rd

def _index_row(k: int, res: Dict[str, Any]) -> List[str]:
	return [f"{k:04d}", str(bool(res.get("success", False))), f"{float(res.get('time', 0.0)):.3f}", str(int(res.get("steps", 0)))]

def run_batch(prompt: str, scn: Dict[str, Any], seeds: List[int], run_dir: Path, profile: str = "minimal") -> Dict[str, Any]:
	"""
	Executes seeded rollouts:
	- Writes per-run folders under run_dir/per_run/run_XXXX/
	- Each run folder contains run_one.csv (from sim_one)
	- Also writes per_run/index.csv (run_id, success, time, steps)
	Returns a lightweight batch summary.
	"""
	t0 = time.time()
	index_rows: List[List[str]] = [["run_id", "success", "time_s", "steps"]]

	successes = 0
	times: List[float] = []
	steps: List[int] = []

	for k, seed in enumerate(seeds):
		prd = _per_run_dir(run_dir, k)
		# Attach seed and profile to scenario copy
		scn_local = dict(scn)
		scn_local["seed"] = int(seed)
		scn_local["profile"] = str(profile)

		res = run_one(prompt, scn_local, prd, dt=float(scn_local.get("runtime", {}).get("dt", 0.05)), realtime=False, gui=False)
		index_rows.append(_index_row(k, res))

		if res.get("success", False):
			successes += 1
		times.append(float(res.get("time", 0.0)))
		steps.append(int(res.get("steps", 0)))

	# Write per_run/index.csv
	per_run = run_dir / "per_run"
	ensure_dir(per_run)
	with open(per_run / "index.csv", "w", newline="", encoding="utf-8") as f:
		w = csv.writer(f)
		w.writerows(index_rows)

	t1 = time.time()
	summary = {
		"runs": len(seeds),
		"successes": successes,
		"failures": len(seeds) - successes,
		"avg_time": (sum(times) / len(times)) if times else 0.0,
		"avg_steps": int(round(sum(steps) / len(steps))) if steps else 0,
		"profile": profile,
		"wallclock_s": round(t1 - t0, 3),
	}
	write_json(run_dir / "summary.json", summary)
	return summary

from __future__ import annotations
from typing import Dict, Any, List, Optional
from pathlib import Path
import csv
import json
import time

from .sampling import set_initial_budget
from .io_utils import ensure_dir, write_json
from .sim_one import run_one

def _per_run_dir(root: Path, k: int) -> Path:
	rd = root / "per_run" / f"run_{k:04d}"
	ensure_dir(rd)
	return rd

def _index_row(k: int, res: Dict[str, Any]) -> List[str]:
	return [f"{k:04d}", str(bool(res.get("success", False))), f"{float(res.get('time', 0.0)):.3f}", str(int(res.get("steps", 0)))]

def _eta_exceeds(elapsed_s: float, done: int, total: int, budget_min: Optional[float]) -> bool:
	if not budget_min or budget_min <= 0 or done <= 0:
		return False
	runs_per_sec = done / max(1e-6, elapsed_s)
	est_total_s = total / max(1e-6, runs_per_sec)
	return est_total_s > budget_min * 60.0

def run_batch(
	prompt: str,
	scn: Dict[str, Any],
	seeds: List[int],
	run_dir: Path,
	profile: str = "minimal",
	time_budget_min: Optional[float] = None,
	auto_degrade: bool = False,
	check_interval: int = 10,
) -> Dict[str, Any]:
	"""
	Executes seeded rollouts:
	- Writes per-run folders under run_dir/per_run/run_XXXX/
	- Each run folder contains run_one.csv (from sim_one)
	- Also writes per_run/index.csv (run_id, success, time, steps)
	- Adds optional time budget guardrail with early stop ("auto-degrade").
	Returns a lightweight batch summary and writes perf.json.
	"""
	t0 = time.perf_counter()
	index_rows: List[List[str]] = [["run_id", "success", "time_s", "steps"]]

	successes = 0
	times: List[float] = []
	steps: List[int] = []

	completed = 0
	total = len(seeds)
	auto_degraded = False
	set_initial_budget(run_dir, k_fail=10, k_succ=10)

	for k, seed in enumerate(seeds):
		prd = _per_run_dir(run_dir, k)
		# Attach seed and profile to scenario copy
		scn_local = dict(scn)
		scn_local["seed"] = int(seed)
		scn_local["profile"] = str(profile)

		res = run_one(
			prompt,
			scn_local,
			prd,
			dt=float(scn_local.get("runtime", {}).get("dt", 0.05)),
			realtime=False,
			gui=False
		)
		index_rows.append(_index_row(k, res))

		if res.get("success", False):
			successes += 1
		times.append(float(res.get("time", 0.0)))
		steps.append(int(res.get("steps", 0)))
		completed += 1

		# Periodically check whether finishing all runs will exceed the budget
		if check_interval and completed % check_interval == 0:
			elapsed = time.perf_counter() - t0
			if _eta_exceeds(elapsed, completed, total, time_budget_min):
				if auto_degrade:
					auto_degraded = True
					break
				# else continue to finish all runs

	# Write per_run/index.csv
	per_run = run_dir / "per_run"
	ensure_dir(per_run)
	with open(per_run / "index.csv", "w", newline="", encoding="utf-8") as f:
		w = csv.writer(f)
		w.writerows(index_rows)

	elapsed = time.perf_counter() - t0
	sims_per_min = (completed / max(1e-6, elapsed)) * 60.0

	# Summary reflects *completed* runs (important if degraded)
	summary = {
		"runs": completed,
		"requested": total,
		"successes": successes,
		"failures": completed - successes,
		"avg_time": (sum(times) / len(times)) if times else 0.0,
		"avg_steps": int(round(sum(steps) / len(steps))) if steps else 0,
		"profile": profile,
		"wallclock_s": round(elapsed, 3),
		"auto_degraded": bool(auto_degraded),
	}
	write_json(run_dir / "summary.json", summary)

	# perf.json for the report / debugging
	perf = {
		"completed": completed,
		"requested": total,
		"elapsed_sec": round(elapsed, 3),
		"sims_per_min": round(sims_per_min, 2),
		"time_budget_min": time_budget_min,
		"auto_degraded": bool(auto_degraded),
		"profile": profile,
	}
	write_json(run_dir / "perf.json", perf)

	return summary

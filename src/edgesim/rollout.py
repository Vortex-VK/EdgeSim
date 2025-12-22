from __future__ import annotations
from typing import Dict, Any, List, Optional, Tuple
from pathlib import Path
import csv
import json
import os
import sys
import time
import copy
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

def _extract_duration_s(scn: Dict[str, Any], default: float = 90.0) -> float:
	try:
		return float(scn.get("runtime", {}).get("duration_s", default))
	except Exception:
		return default

def _extract_lidar_hz(scn: Dict[str, Any], default: float = 10.0) -> float:
	try:
		return float((scn.get("sensors", {}) or {}).get("lidar", {}).get("hz", default))
	except Exception:
		return default

def _apply_policy(scn: Dict[str, Any], duration_cap_s: Optional[float], lidar_hz_cap: Optional[float]) -> Dict[str, Any]:
	"""Return a scenario copy with policy constraints applied (non-destructive)."""
	s = copy.deepcopy(scn)
	if duration_cap_s is not None:
		s.setdefault("runtime", {})
		# keep the smaller of current and cap
		cur = _extract_duration_s(s)
		s["runtime"]["duration_s"] = min(cur, float(duration_cap_s))
	if lidar_hz_cap is not None:
		s.setdefault("sensors", {})
		s["sensors"].setdefault("lidar", {})
		cur = _extract_lidar_hz(s)
		s["sensors"]["lidar"]["hz"] = min(cur, float(lidar_hz_cap))
	return s

def _estimate_cost_per_sim_min(wallclock_s: float, sim_minutes_total: float) -> Optional[float]:
	"""
	Very simple estimator: use CPU cost per hour from env EDGESIM_CPU_COST_PER_HR (default 0.0).
	cost_per_sim_min = (wallclock_hours * cpu_rate) / sim_minutes_total
	"""
	try:
		rate = float(os.getenv("EDGESIM_CPU_COST_PER_HR", "0.0"))
	except Exception:
		rate = 0.0
	if sim_minutes_total <= 0:
		return None
	wallclock_hours = max(0.0, wallclock_s) / 3600.0
	return (wallclock_hours * rate) / sim_minutes_total

def _format_eta(eta_s: Optional[float]) -> str:
	try:
		if eta_s is None:
			return "--:--"
		eta = float(eta_s)
	except Exception:
		return "--:--"
	if eta < 0:
		return "--:--"
	total_s = int(round(eta))
	hours, rem = divmod(total_s, 3600)
	mins, secs = divmod(rem, 60)
	if hours > 0:
		return f"{hours}h{mins:02d}m"
	return f"{mins:02d}:{secs:02d}"

def _progress_line(completed: int, total: int, elapsed_s: float, width: int = 28) -> str:
	if total <= 0:
		pct = 0.0
	else:
		pct = max(0.0, min(1.0, completed / float(total)))
	filled = int(width * pct)
	bar = "#" * filled + "-" * (width - filled)
	if completed > 0:
		runs_per_min = (completed / max(1e-6, elapsed_s)) * 60.0
		remaining = max(0, total - completed)
		runs_per_sec = completed / max(1e-6, elapsed_s)
		eta_s = remaining / max(1e-6, runs_per_sec)
	else:
		runs_per_min = 0.0
		eta_s = None
	return f"[{bar}] {completed}/{total} | {runs_per_min:.1f} runs/min | ETA {_format_eta(eta_s)}"

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
	- Time budget guardrail with auto-degrade policy:
	    * When projected ETA would exceed budget, shorten remaining run duration.
	    * If still over budget, reduce LiDAR Hz (e.g., 10 -> 5).
	Returns a lightweight batch summary and writes perf.json / summary.json with policy + telemetry.
	"""
	t0 = time.perf_counter()
	index_rows: List[List[str]] = [["run_id", "success", "time_s", "steps"]]

	successes = 0
	times: List[float] = []
	steps: List[int] = []
	sim_seconds_accum = 0.0

	completed = 0
	total = len(seeds)
	# Track original knobs so we can compute caps proportionally
	orig_duration_s = _extract_duration_s(scn, default=90.0)
	orig_lidar_hz = _extract_lidar_hz(scn, default=10.0)

	# Policy state (None means no cap applied)
	policy_applied = {
		"activated": False,
		"reason": None,  # "time_budget"
		"duration_cap_s": None,  # float
		"lidar_hz_cap": None,    # float
	}
	use_progress = sys.stdout.isatty()

	for k, seed in enumerate(seeds):
		prd = _per_run_dir(run_dir, k)

		# Decide if we need to tighten policy before launching this run
		if auto_degrade and time_budget_min and time_budget_min > 0 and completed > 0:
			elapsed = time.perf_counter() - t0
			if _eta_exceeds(elapsed, completed, total, time_budget_min):
				# Compute remaining budget and tighten per-run caps
				budget_remaining_s = time_budget_min * 60.0 - elapsed
				remaining = max(1, total - completed)

				# Target average wall time per remaining run to stay within budget
				target_wall_per_run_s = max(5.0, budget_remaining_s / remaining)

				# Heuristic mapping wall time -> sim duration cap:
				# Assume wall_time ~= 0.6 * duration_s (empirical), clamp sensibly.
				est_ratio = 0.6
				duration_cap = max(10.0, min(orig_duration_s, target_wall_per_run_s / est_ratio))

				# First level: shorten duration
				lidar_cap = None
				# If duration already very short but still projected to exceed, drop LiDAR rate by half (min 3 Hz)
				if duration_cap <= 20.0:
					lidar_cap = max(3.0, min(orig_lidar_hz, round(orig_lidar_hz / 2.0)))

				policy_applied.update({
					"activated": True,
					"reason": "time_budget",
					"duration_cap_s": float(duration_cap),
					"lidar_hz_cap": float(lidar_cap) if lidar_cap is not None else None,
				})

		# Attach seed and profile to scenario copy
		scn_local = dict(scn)
		scn_local["seed"] = int(seed)
		scn_local["profile"] = str(profile)

		# Apply policy (if any)
		if policy_applied["activated"]:
			scn_local = _apply_policy(
				scn_local,
				duration_cap_s=policy_applied["duration_cap_s"],
				lidar_hz_cap=policy_applied["lidar_hz_cap"],
			)

		# Run one rollout
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
		runtime_s = float(res.get("time", 0.0))
		times.append(runtime_s)
		steps.append(int(res.get("steps", 0)))
		sim_seconds_accum += runtime_s
		completed += 1

		elapsed = time.perf_counter() - t0
		line = _progress_line(completed, total, elapsed)
		if use_progress:
			print(f"\r{line}", end="", flush=True)
		else:
			print(line)

		# Periodically check whether finishing all runs will exceed the budget.
		# Note: since we *degrade* dynamically above, we no longer break early here unless
		# the user asked for auto_degrade=False (in which case we ignore the budget entirely).
		if check_interval and completed % check_interval == 0 and auto_degrade:
			# Re-evaluate and possibly tighten policy sooner for the next runs
			# (actual tightening is handled at the top of the loop)
			pass

	# Write per_run/index.csv
	per_run = run_dir / "per_run"
	ensure_dir(per_run)
	with open(per_run / "index.csv", "w", newline="", encoding="utf-8") as f:
		w = csv.writer(f)
		w.writerows(index_rows)

	if use_progress:
		print()

	elapsed = time.perf_counter() - t0
	sims_per_min = (completed / max(1e-6, elapsed)) * 60.0
	sim_minutes_total = sim_seconds_accum / 60.0
	est_cost_per_sim_min = _estimate_cost_per_sim_min(elapsed, sim_minutes_total)

	# Summary reflects *completed* runs
	summary = {
		"runs": completed,
		"requested": total,
		"successes": successes,
		"failures": completed - successes,
		"avg_time": (sum(times) / len(times)) if times else 0.0,
		"avg_steps": int(round(sum(steps) / len(steps))) if steps else 0,
		"profile": profile,
		"wallclock_s": round(elapsed, 3),
		"sim_minutes_total": round(sim_minutes_total, 3),
		"est_cost_per_sim_min": (round(est_cost_per_sim_min, 5) if isinstance(est_cost_per_sim_min, float) else None),
		"auto_degraded": bool(policy_applied["activated"]),
		"auto_degrade_policy": policy_applied if policy_applied["activated"] else None,
	}
	write_json(run_dir / "summary.json", summary)

	# perf.json for deeper inspection
	perf = {
		"completed": completed,
		"requested": total,
		"elapsed_sec": round(elapsed, 3),
		"sims_per_min": round(sims_per_min, 2),
		"time_budget_min": time_budget_min,
		"profile": profile,
		"sim_minutes_total": round(sim_minutes_total, 3),
		"est_cost_per_sim_min": (round(est_cost_per_sim_min, 5) if isinstance(est_cost_per_sim_min, float) else None),
		"auto_degraded": bool(policy_applied["activated"]),
		"auto_degrade_policy": policy_applied if policy_applied["activated"] else None,
		"original_knobs": {
			"duration_s": orig_duration_s,
			"lidar_hz": orig_lidar_hz,
		},
	}
	write_json(run_dir / "perf.json", perf)

	return summary

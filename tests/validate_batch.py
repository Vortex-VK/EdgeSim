#!/usr/bin/env python3
from __future__ import annotations
import sys, csv, json, hashlib, os, math
from pathlib import Path
from typing import List, Dict, Tuple

EXPECTED_HEADER = [
	"t","x","y","yaw","v_cmd","w_cmd",
	"min_clearance_lidar","min_clearance_geom",
	"event","event_detail","in_wet","human_phase","near_stop","hard_brake",
	"near_miss","occluded_hazard","interaction","sensor_faults",
	"min_ttc","odom_v","odom_w","imu_ax","imu_ay","imu_wz"
]

def die(msg: str, code: int = 2):
	print(f"[FAIL] {msg}")
	sys.exit(code)

def warn(msg: str):
	print(f"[WARN] {msg}")

def ok(msg: str):
	print(f"[OK] {msg}")

def has_files(run_dir: Path) -> None:
	required = ["scenario.yaml", "seeds.json", "summary.json", "report.md", "per_run"]
	for name in required:
		if not (run_dir / name).exists():
			die(f"Missing required artifact: {name}")
	ok("All required top-level artifacts present")

def read_csv_rows(path: Path) -> Tuple[List[str], List[List[str]]]:
	with path.open(newline="", encoding="utf-8") as f:
		r = csv.reader(f)
		try:
			header = next(r)
		except StopIteration:
			return [], []
		rows = [row for row in r]
	return header, rows

def float_or_nan(s: str) -> float:
	try:
		return float(s)
	except Exception:
		return float("nan")

def check_csv_invariants(csv_path: Path) -> Dict[str, int | float | bool]:
	hdr, rows = read_csv_rows(csv_path)
	if hdr != EXPECTED_HEADER:
		die(f"{csv_path}: Unexpected header.\nExpected: {EXPECTED_HEADER}\nGot:      {hdr}")

	if not rows:
		die(f"{csv_path}: No rows found")

	# Column indices
	col = {name:i for i, name in enumerate(hdr)}
	last_t = -1.0
	last_min_geom = float("inf")
	seen_success = False
	seen_collision_human = False
	seen_slip = 0
	seen_fallen = False
	near_stop_count = 0

	for r_i, r in enumerate(rows):
		# monotone time
		t = float_or_nan(r[col["t"]]); 
		if math.isnan(t): die(f"{csv_path}: row {r_i} has non-float t")
		if t < last_t - 1e-9:
			die(f"{csv_path}: time not non-decreasing at row {r_i} ({t} < {last_t})")
		last_t = t

		# geom clearance monotone non-increasing
		mc_geom = float_or_nan(r[col["min_clearance_geom"]])
		if math.isnan(mc_geom): die(f"{csv_path}: row {r_i} has non-float min_clearance_geom")
		if mc_geom > last_min_geom + 1e-6:
			die(f"{csv_path}: min_clearance_geom increased at row {r_i} ({mc_geom} > {last_min_geom})")
		last_min_geom = min(last_min_geom, mc_geom)

		# human phase semantics:
		phase = r[col["human_phase"]]
		if phase not in ("none","running","fallen"):
			die(f"{csv_path}: row {r_i} has invalid human_phase '{phase}'")

		# Once fallen is observed, it must remain fallen for the rest of the run
		if seen_fallen and phase != "fallen":
			die(f"{csv_path}: human_phase left 'fallen' at row {r_i} (got '{phase}')")
		if phase == "fallen":
			seen_fallen = True

		# events semantics
		event = r[col["event"]]
		if event == "mission_success":
			seen_success = True
			# success should be terminal (no rows after with events)
			if r_i != len(rows) - 1:
				die(f"{csv_path}: 'mission_success' not terminal (rows continue after success)")
		elif event == "collision_human":
			seen_collision_human = True
			if phase not in ("running","fallen"):
				die(f"{csv_path}: collision_human without human present (phase={phase})")
		elif event == "floor_slip":
			seen_slip += 1

		# near_stop accounting
		try:
			near_stop_val = int(r[col["near_stop"]])
			if near_stop_val not in (0,1): 
				die(f"{csv_path}: near_stop not 0/1 at row {r_i}")
			near_stop_count += 1 if near_stop_val == 1 else 0
		except Exception:
			die(f"{csv_path}: near_stop not int at row {r_i}")

	# only one slip per run (by design)
	if seen_slip > 1:
		die(f"{csv_path}: more than one 'slip' event ({seen_slip})")

	return {
		"success": 1 if seen_success else 0,
		"collision_human": 1 if seen_collision_human else 0,
		"near_stop_rows": near_stop_count,
	}

def digest_dir(per_run_dir: Path) -> str:
	h = hashlib.sha256()
	for p in sorted(per_run_dir.rglob("*.csv")):
		h.update(p.read_bytes())
	return h.hexdigest()

def load_summary(path: Path) -> Dict:
	try:
		return json.loads(path.read_text(encoding="utf-8"))
	except Exception:
		die(f"Failed to load {path}")

def validate_batch(run_dir: Path) -> Dict[str, int]:
	has_files(run_dir)
	per_run = run_dir / "per_run"
	if not per_run.exists():
		die("per_run directory missing")

	csvs = sorted(per_run.glob("run_*/run_one.csv"))
	if not csvs:
		die("No run_* CSVs found under per_run")

	count_success = 0
	count_coll_h = 0
	count_runs = 0
	with_near_stops = 0

	for c in csvs:
		stats = check_csv_invariants(c)
		count_success += int(stats["success"])
		count_coll_h += int(stats["collision_human"])
		with_near_stops += 1 if int(stats["near_stop_rows"]) > 0 else 0
		count_runs += 1

	ok(f"Checked {count_runs} runs for schema + invariants")

	# Aggregate plausibility (loose, but catches “all-success” bugs)
	if count_success == 0 or count_coll_h == 0:
		warn(f"Aggregate outcomes are one-sided (success={count_success}, collision_human={count_coll_h}).")
	else:
		ok(f"Aggregate outcomes show mix (success={count_success}, collision_human={count_coll_h})")

	# Acceptance: failure rate >= 10% for stress prompt
	fail_rate = (count_coll_h / max(1, count_runs))
	if fail_rate < 0.10:
		warn(f"Failure rate {fail_rate:.2%} < 10% target for stress scenario")
	else:
		ok(f"Failure rate {fail_rate:.2%} meets ≥10% target (stress scenario)")

	if with_near_stops == 0:
		warn("No near_stop rows observed—proximity brake may not be triggering")
	else:
		ok(f"near_stop observed in {with_near_stops}/{count_runs} runs")

	return {
		"runs": count_runs,
		"success": count_success,
		"collision_human": count_coll_h,
	}

def compare_batches(dir1: Path, dir2: Path) -> None:
	d1 = digest_dir(dir1 / "per_run")
	d2 = digest_dir(dir2 / "per_run")
	if d1 == d2:
		ok("Determinism: per_run CSV digests MATCH (identical)")
	else:
		warn("Determinism: per_run CSV digests DIFFER (seeds or nondeterminism differ)")
	s1 = load_summary(dir1 / "summary.json")
	s2 = load_summary(dir2 / "summary.json")
	if s1.get("counts") == s2.get("counts") or s1.get("outcomes_pct") == s2.get("outcomes_pct"):
		ok("Coarse summaries match")
	else:
		warn("Coarse summaries differ (expected if seeds changed)")

def main():
	if len(sys.argv) < 2:
		print("Usage: python tests/validate_batch.py runs/<batch_dir> [runs/<second_batch_dir_for_compare>]")
		sys.exit(1)
	run_dir = Path(sys.argv[1]).resolve()
	if not run_dir.exists():
		die(f"Batch dir not found: {run_dir}")
	stats = validate_batch(run_dir)

	if len(sys.argv) >= 3:
		run_dir2 = Path(sys.argv[2]).resolve()
		if not run_dir2.exists():
			die(f"Second batch dir not found: {run_dir2}")
		compare_batches(run_dir, run_dir2)

	ok("Validation finished")

if __name__ == "__main__":
	main()

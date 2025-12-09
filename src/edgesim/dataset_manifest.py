from __future__ import annotations
from pathlib import Path
from typing import Dict, Any, List, Optional, Tuple
import csv, json, math

import yaml


def _safe_float(val: Any, default: float = 0.0) -> float:
	try:
		f = float(val)
		if math.isfinite(f):
			return f
	except Exception:
		pass
	return default


def _load_scenario(batch_dir: Path, scenario: Optional[Dict[str, Any]]) -> Dict[str, Any]:
	if scenario:
		return dict(scenario)
	scenario_path = batch_dir / "scenario.yaml"
	if scenario_path.exists():
		try:
			return yaml.safe_load(scenario_path.read_text(encoding="utf-8")) or {}
		except Exception:
			return {}
	return {}


def _iter_runs(batch_dir: Path) -> List[Tuple[str, Path]]:
	per_run = batch_dir / "per_run"
	if per_run.exists():
		out: List[Tuple[str, Path]] = []
		for sub in sorted(per_run.glob("run_*")):
			csv_path = sub / "run_one.csv"
			if csv_path.exists():
				out.append((sub.name, csv_path))
		return out
	# fallback: single-run layout (csv in root)
	root_csv = batch_dir / "run_one.csv"
	return [("run_0000", root_csv)] if root_csv.exists() else []


def _parse_run(csv_path: Path) -> Dict[str, Any]:
	last_t = 0.0
	dist = 0.0
	prev_xy: Optional[Tuple[float, float]] = None
	min_ttc = float("inf")
	events = {"collision_human": 0, "collision_static": 0, "slip": 0, "near_miss": 0, "occluded_hazard": 0}
	with csv_path.open("r", newline="", encoding="utf-8") as f:
		reader = csv.DictReader(f)
		for row in reader:
			t = _safe_float(row.get("t"))
			x = _safe_float(row.get("x"))
			y = _safe_float(row.get("y"))
			if prev_xy is not None:
				dist += math.hypot(x - prev_xy[0], y - prev_xy[1])
			prev_xy = (x, y)
			last_t = max(last_t, t)
			mt = row.get("min_ttc")
			if mt not in (None, ""):
				val = _safe_float(mt, default=float("inf"))
				if val == val:  # not NaN
					min_ttc = min(min_ttc, val)
			ev = (row.get("event") or "").strip()
			if ev == "collision_human":
				events["collision_human"] += 1
			elif ev == "collision_static":
				events["collision_static"] += 1
			elif ev == "floor_slip":
				events["slip"] += 1
			try:
				if int(row.get("near_miss", 0)) > 0:
					events["near_miss"] += 1
			except Exception:
				pass
			try:
				if int(row.get("occluded_hazard", 0)) > 0:
					events["occluded_hazard"] += 1
			except Exception:
				pass
	avg_speed = dist / last_t if last_t > 1e-6 else 0.0
	return {
		"duration_s": last_t,
		"avg_speed_mps": avg_speed,
		"min_ttc": None if (not math.isfinite(min_ttc)) else min_ttc,
		"events": events,
		"flags": {
			"near_miss": events.get("near_miss", 0) > 0,
			"occluded_hazard": events.get("occluded_hazard", 0) > 0,
		},
	}


def _randomization_from_world(world_path: Path) -> Dict[str, Any]:
	try:
		world = json.loads(world_path.read_text(encoding="utf-8"))
	except Exception:
		return {}
	mu_vals: List[float] = []
	for z in world.get("floor_zones", []):
		try:
			if z.get("mu") is not None:
				mu_vals.append(float(z["mu"]))
		except Exception:
			continue
	return {"traction_mu": sorted(set(round(v, 3) for v in mu_vals))}


def _randomization_from_scn(scn: Dict[str, Any]) -> Dict[str, Any]:
	sensors = (scn.get("sensors") or {})
	lidar_cfg = (sensors.get("lidar") or {})
	layout = scn.get("layout", {}) or {}
	randomization = {
		"lidar_noise_sigma": lidar_cfg.get("noise_sigma"),
		"lidar_dropout_pct": lidar_cfg.get("dropout_pct"),
		"clutter_level": layout.get("clutter_level"),
		"behavior_presets": [],
	}
	behaviors = set()
	for h in scn.get("hazards", {}).get("human", []) or []:
		try:
			if h.get("role"):
				behaviors.add(str(h["role"]))
			elif h.get("type"):
				behaviors.add(str(h["type"]))
		except Exception:
			continue
	randomization["behavior_presets"] = sorted(behaviors)
	return randomization


def build_dataset_manifest(batch_dir: Path, scenario: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
	batch_dir = Path(batch_dir)
	runs = _iter_runs(batch_dir)
	if not runs:
		return {}
	scn = _load_scenario(batch_dir, scenario)
	run_summaries: List[Dict[str, Any]] = []
	event_totals = {"collision_human": 0, "collision_static": 0, "slip": 0, "near_miss": 0, "occluded_hazard": 0}
	duration_list: List[float] = []
	avg_speed_list: List[float] = []
	min_ttc_list: List[float] = []
	for run_id, csv_path in runs:
		stats = _parse_run(csv_path)
		stats["run_id"] = run_id
		run_summaries.append(stats)
		duration_list.append(stats["duration_s"])
		avg_speed_list.append(stats["avg_speed_mps"])
		if isinstance(stats.get("min_ttc"), (int, float)):
			min_ttc_list.append(float(stats["min_ttc"]))
		for k, v in stats.get("events", {}).items():
			if k in event_totals:
				event_totals[k] += int(v)
	# randomization knobs
	world_path = batch_dir / "world.json"
	if not world_path.exists():
		for _, csv_path in runs:
			cand = csv_path.parent / "world.json"
			if cand.exists():
				world_path = cand
				break
	randomization = _randomization_from_world(world_path)
	randomization.update(_randomization_from_scn(scn))

	try:
		manifest_meta = json.loads((batch_dir / "manifest.json").read_text(encoding="utf-8"))
	except Exception:
		manifest_meta = {}

	event_totals["collisions"] = event_totals.get("collision_human", 0) + event_totals.get("collision_static", 0)

	result: Dict[str, Any] = {
		"prompt": manifest_meta.get("prompt"),
		"runs": run_summaries,
		"event_counts": event_totals,
		"stats": {
			"duration_avg_s": sum(duration_list) / len(duration_list) if duration_list else 0.0,
			"avg_speed_mean_mps": sum(avg_speed_list) / len(avg_speed_list) if avg_speed_list else 0.0,
			"min_ttc_min_s": min(min_ttc_list) if min_ttc_list else None,
		},
		"tags": scn.get("taxonomy", {}),
		"randomization": randomization,
	}
	out_path = batch_dir / "dataset_manifest.json"
	try:
		out_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
	except Exception:
		pass
	return result

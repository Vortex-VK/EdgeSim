from __future__ import annotations
from typing import Any, Dict, List


# Minimal Scenario Graph (V0) — plain dicts to keep it simple


def default_layout() -> Dict[str, Any]:
	return {
		"map_size_m": [20.0, 20.0],
		"aisles": [
			{"name": "A1", "rect": [2.0, 1.0, 16.0, 3.0]},
			{"name": "A2", "rect": [2.0, 8.0, 16.0, 10.0]},
		],
		"walls": [
			{"poly": [[0, 0], [20, 0], [20, 20], [0, 20]]}
		],
		"start": [1.0, 1.0],
		"goal": [19.0, 19.0],
	}


def default_agent() -> Dict[str, Any]:
	return {
		"id": "amr_0",
		"type": "AMR",
		"radius_m": 0.4,
		"max_speed_mps": 1.2,
		"controller": {
			"planner": "astar_grid",
			"follower": "pid_waypoint",
		},
	}


def default_sensors() -> Dict[str, Any]:
	"""
	Default sensor configuration.

	We keep it minimal but add a 'lidar' sub-dict so the parser/scenario
	can tweak the lidar update rate if needed.
	"""
	return {
		"camera_noise": 0.0,   # flag/range for later
		"detection_dropout": 0.0,
		"latency_ms": 0,
		"lighting_dim": False,
		"lidar": {
			"hz": 10.0,
		},
	}


def empty_hazards() -> Dict[str, Any]:
	return {
		"traction": [],   # list of {zone: [x0,y0,x1,y1], mu}
		"human": [],      # list of {path: "line"|"waypoints", rate_per_min: float, speed_mps: [min,max]}
		"clutter": {
			"aisle_density": 0.0,
			"overhang_prob": 0.0,
		},
	}


def default_runtime(n_runs: int) -> Dict[str, Any]:
	return {
		"duration_s": 120,
		"dt": 0.1,
		"N_runs": n_runs,
	}


def new_scenario(n_runs: int = 100) -> Dict[str, Any]:
	"""
	Create a fresh scenario dict.

	We keep taxonomy open-ended (you can add more flags later), and we
	add 'site_overrides' so the natural-language parser can override
	site_profiles/<SITE>.json in a per-scenario way.
	"""
	return {
		"layout": default_layout(),
		"agents": [default_agent()],
		"hazards": empty_hazards(),
		"sensors": default_sensors(),
		"runtime": default_runtime(n_runs),
		"taxonomy": {  # for coverage counting later
			"visibility": False,
			"traction": False,
			"localization": False,
			"human_behavior": False,
			# extended flags (safe to ignore elsewhere if unused)
			"occlusion": False,
			"sensor_fault": False,
			"multi_actor": False,
		},
		# Per-scenario overrides that get merged on top of site_profiles.
		# See run_one() in sim_one.py for how they are applied.
		"site_overrides": {},
	}

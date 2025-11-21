from __future__ import annotations
from typing import Any, Dict, List


# Minimal Scenario Graph (V0) — plain dicts to keep it simple


def default_geometry() -> Dict[str, Any]:
	return {
		"aisles": [],
		"junctions": [],
		"blind_corners": [],
		"racking": [],
		"open_storage": [],
		"endcaps": [],
	}


def default_floor_surfaces() -> List[Dict[str, Any]]:
	return [
		{"id": "base_floor", "type": "dry", "zone": [0.0, 0.0, 20.0, 20.0], "mu": 0.85, "brake_scale": 1.0, "slip_boost": 0.0, "imu_vibe_g": 0.2},
	]


def default_transition_zones() -> List[Dict[str, Any]]:
	return []


def default_static_obstacles() -> List[Dict[str, Any]]:
	return []


def default_environment() -> Dict[str, Any]:
	"""Environment-wide knobs (kept minimal for the core sim)."""
	return {}


def default_vehicle_roster() -> List[Dict[str, Any]]:
	return [
		{"id": "Forklift_01", "type": "forklift", "path": [[4.0, 4.0], [4.0, 16.0]], "speed_mps": 1.2, "fork_height_m": 0.2, "reflective": True},
		{"id": "Forklift_02", "type": "forklift", "path": [[15.5, 4.0], [15.5, 16.0]], "speed_mps": 1.0, "fork_height_m": 1.2, "carrying_pallet": True},
		{"id": "PalletJack_01", "type": "pallet_jack", "path": [[8.5, 2.0], [12.0, 2.0]], "speed_mps": 0.9},
		{"id": "Tugger_Train", "type": "tugger_train", "path": [[10.0, 6.0], [10.0, 14.0]], "speed_mps": 1.4, "length_m": 2.5},
	]


def default_layout() -> Dict[str, Any]:
	return {
		"map_size_m": [20.0, 20.0],
		# Start empty so a blank prompt yields an open floor; aisles get added only when requested.
		"aisles": [],
		"walls": [
			{"poly": [[0, 0], [20, 0], [20, 20], [0, 20]]}
		],
		"start": [2.0, 10.0],
		"goal": [18.0, 10.0],
		"geometry": default_geometry(),
		"floor_surfaces": default_floor_surfaces(),
		"transition_zones": default_transition_zones(),
		"static_obstacles": default_static_obstacles(),
	}


def default_agent() -> Dict[str, Any]:
	return {
		"id": "amr_0",
		"type": "AMR",
		"radius_m": 0.4,
		"max_speed_mps": 1.2,
	}


def default_sensors() -> Dict[str, Any]:
	return {
		"lidar": {
			"hz": 10.0,
			"max_range_m": 8.0,
			"noise_sigma": 0.02,
			"dropout_pct": 0.01,
		},
		"imu": {"noise_std": 0.01},
		"odom": {"bias_v": 0.02, "bias_w": 0.01},
	}


def empty_hazards() -> Dict[str, Any]:
	return {
		"traction": [],   # list of {zone: [x0,y0,x1,y1], mu}
		"human": [],      # list of {path: "line"|"waypoints", rate_per_min: float, speed_mps: [min,max]}
		"vehicles": [],
		"floor_events": [],    # spills forming in runtime
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
		"environment": default_environment(),
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

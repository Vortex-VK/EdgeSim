from __future__ import annotations
import copy
import re
from typing import Any, Dict, List

from .schema import new_scenario, default_vehicle_roster

# --- Keyword map for simple rule-based parsing (V0+) ---
KEYWORDS = {
	"traction":   ["wet", "slippery", "spill", "spillage", "oil"],
	"visibility": ["night", "dim", "dark", "low light", "reflective"],
	"human":      ["human", "picker", "worker", "pedestrian", "crossing"],
	"traffic":    ["high pallet traffic", "rush", "busy", "heavy traffic"],
	"overhang":   ["overhang", "irregular load"],

	# Existing special intents
	"two_crosswalks": ["two crosswalks", "double crosswalk", "staggered human crossings", "two crossers"],
	"narrow":     ["narrow aisle", "narrow corridor", "tight aisle"],
	"falling":    ["falling object", "pallet drops", "pallet falling", "drop near", "shatter"],

	# New human-behavioral flavors
	"distracted": ["on phone", "phone in hand", "head-down", "head down",
	               "not paying attention", "distracted worker"],
	"busy_crossing": ["group of workers", "platoon", "crowd", "busy crossing",
	                  "many people crossing", "multiple workers"],

	# Occlusion / blind corners
	"blind_corner": ["blind corner", "around the corner", "hidden crosswalk",
	                 "endcap", "blocked view", "occluded crosswalk"],

	# Industrial vehicles / carts
	"forklift": ["forklift", "hi-lo", "lift truck", "fork truck"],
	"pallet_jack": ["pallet jack", "hand truck", "hand-cart", "hand cart"],

	# Density / congestion / cart stacking
	"busy_aisle": ["busy aisle", "busy aisles", "busy aisleway", "congested aisle"],
	"congestion": ["congestion", "gridlock", "backed up", "logjam"],
	"cart_block": ["cart blocking", "cart stuck", "cart across", "cart left in aisle", "cart blocking aisle"],

	# Floor & transition events
	"cleaning": ["cleaning liquid", "cleaning spill", "foam", "detergent", "freshly mopped", "mopping"],
	"transition": ["doorway", "fire door", "sliding door", "threshold bump", "loading dock", "dock plate"],

	# Geometry / occlusion
	"high_shelf": ["high shelf", "high-bay", "high bay", "high rack", "high-shelf occlusion"],

	# Human posture/types
	"worker_carry": ["carrying box", "carrying tote", "carrying bin", "worker carrying"],
	"child_actor": ["child-height", "child height", "kid", "youth", "child"],
	"fast_walker": ["fast walker", "fast walking", "hurried", "running picker"],

	# Forklift reversing / alarms
	"forklift_reverse": ["forklift reversing", "forklift backing", "forklift beeping", "forklift alarm"],

	# Clothing / reflectivity
	"reflective": ["reflective vest", "hi-vis", "hi vis", "high visibility", "high-visibility"],
	"dark_clothing": ["dark clothing", "dark hoodie", "black jacket", "no vest"],

	# Sensor / lidar issues
	"sensor_fault": ["lidar fault", "sensor glitch", "ghost obstacle",
	                 "false obstacle", "false positive", "sensor issue"],
	"degraded_mode": ["degraded mode", "reduced lidar", "low sensor", "cpu overload",
	                  "failsafe mode", "degraded sensing"],
}

_GROUP_CROSS_TERMS = [
	"group of workers",
	"group of people",
	"group crossing",
	"workers crossing",
	"group of staff",
]

_MAIN_AISLE_TERMS = [
	"main aisle",
	"main-aisle",
	"central aisle",
]

_T_INTERSECTION_TERMS = [
	"t intersection",
	"t-intersection",
	"t junction",
	"t-junction",
]

def _is_main_aisle_lr_prompt(text: str) -> bool:
	text_l = text.lower()
	has_main = any(term in text_l for term in _MAIN_AISLE_TERMS)
	has_lr = ("left to right" in text_l) or ("left-right" in text_l) or ("left→right" in text_l)
	return has_main and has_lr

def _is_t_intersection_prompt(text: str) -> bool:
	text_l = text.lower()
	return any(term in text_l for term in _T_INTERSECTION_TERMS)

def _apply_main_aisle_lr_layout(scn: Dict[str, Any]) -> None:
	layout = scn.setdefault("layout", {})
	layout["map_size_m"] = [20.0, 20.0]
	layout["start"] = [2.0, 9.0]
	layout["goal"] = [18.0, 9.0]
	geom = layout.setdefault("geometry", {})
	geom.clear()
	geom["main_aisle_lane_y"] = 9.0
	geom["main_aisle_lane_x"] = 10.0
	geom["cross_access"] = True
	# Split horizontal racks to leave a doorway near the crossing corridor
	geom["racking"] = [
		{"id": "Rack_lower_W", "aabb": [1.0, 6.0, 9.0, 7.0], "height_m": 3.0},
		{"id": "Rack_lower_E", "aabb": [11.0, 6.0, 19.0, 7.0], "height_m": 3.0},
		{"id": "Rack_upper_W", "aabb": [1.0, 11.0, 9.0, 12.0], "height_m": 3.0},
		{"id": "Rack_upper_E", "aabb": [11.0, 11.0, 19.0, 12.0], "height_m": 3.0},
	]
	layout["aisles"] = [
		{"id": "A_main", "name": "main_aisle", "rect": [1.0, 7.0, 19.0, 11.0], "type": "straight", "pad": [0.0, 0.2, 0.0, 0.2], "racking": False},
		{"id": "A_cross", "name": "cross_access", "rect": [9.0, 7.0, 11.0, 11.0], "type": "cross", "pad": [0.2, 0.0, 0.2, 0.0], "racking": False},
	]
	layout["floor_surfaces"] = [{
		"id": "base_floor",
		"type": "dry",
		"zone": [0.0, 0.0, 20.0, 20.0],
		"mu": 0.85,
		"brake_scale": 1.0,
		"slip_boost": 0.0,
		"imu_vibe_g": 0.2,
	}]
	layout["transition_zones"] = []
	layout["static_obstacles"] = []
	layout["lock_aisles"] = True
	layout["auto_aisles_from_paths"] = False
	layout["paths_define_aisles"] = False
	layout["main_lr_case"] = True
	hazards = scn.setdefault("hazards", {})
	hazards["vehicles"] = []
	hazards["human"] = [{
		"path": "waypoints",
		"waypoints": [[10.0, 6.5], [10.0, 13.5]],
		"rate_per_min": 2.0,
		"group_size": 4,
		"start_delay_s": 8.0,
		"speed_mps": [0.8, 1.4],
		"motion_patterns": ["linear", "start_stop", "close_pass"],
		"interaction_modes": ["mutual_yield", "human_yield", "robot_yield"],
	}]
	hazards["traction"] = hazards.get("traction", [])
	hazards["floor_events"] = hazards.get("floor_events", [])
	scn["taxonomy"]["multi_actor"] = True
	scn["taxonomy"]["human_behavior"] = True
	# baseline aisle clutter for realism
	layout["static_obstacles"] = [
		{"type": "standing_pallet", "aabb": [5.5, 7.4, 6.6, 8.3], "height": 1.0},
		{"type": "standing_pallet", "aabb": [13.6, 9.6, 14.6, 10.6], "height": 1.0},
	]

def _apply_t_intersection_layout(scn: Dict[str, Any]) -> None:
	layout = scn.setdefault("layout", {})
	layout["map_size_m"] = [20.0, 20.0]
	layout["start"] = [2.0, 9.0]
	layout["goal"] = [18.0, 9.0]
	geom = layout.setdefault("geometry", {})
	geom.clear()
	geom["t_intersection_case"] = True
	geom["main_aisle_lane_y"] = 9.0
	geom["branch_lane_x"] = 10.0
	geom["racking"] = [
		{"id": "Rack_lower_W", "aabb": [1.0, 6.0, 9.1, 7.0], "height_m": 3.0},
		{"id": "Rack_lower_E", "aabb": [10.9, 6.0, 19.0, 7.0], "height_m": 3.0},
		{"id": "Rack_upper_W", "aabb": [1.0, 11.0, 9.1, 12.0], "height_m": 3.0},
		{"id": "Rack_upper_E", "aabb": [10.9, 11.0, 19.0, 12.0], "height_m": 3.0},
		{"id": "Rack_branch_side_left_low", "aabb": [7.6, 3.5, 8.8, 7.0], "height_m": 3.0},
		{"id": "Rack_branch_side_left_high", "aabb": [7.6, 11.0, 8.8, 16.5], "height_m": 3.0},
		{"id": "Rack_branch_side_right", "aabb": [11.2, 3.5, 12.4, 16.5], "height_m": 3.0},
	]
	layout["aisles"] = [
		{"id": "A_main", "name": "main_aisle", "rect": [1.0, 7.0, 19.0, 11.0], "type": "straight", "pad": [0.0, 0.2, 0.0, 0.2], "racking": False},
		{"id": "A_branch_vertical", "name": "branch_vertical", "rect": [9.1, 3.5, 10.9, 16.5], "type": "straight", "pad": [0.0, 0.2, 0.0, 0.0], "racking": False, "appearance": "vertical_branch"},
	]
	layout["junctions"] = [
		{"id": "J_center", "rect": [9.1, 7.0, 10.9, 11.0], "type": "t", "pad": [0.2, 0.0, 0.2, 0.0]}
	]
	layout["transition_zones"] = []
	layout["static_obstacles"] = []
	layout["lock_aisles"] = True
	layout["auto_aisles_from_paths"] = False
	layout["paths_define_aisles"] = False
	layout["t_case"] = True
	hazards = scn.setdefault("hazards", {})
	hazards["vehicles"] = [{
		"id": "ForkliftTBranch",
		"type": "forklift",
		"path": [[10.0, 3.6], [10.0, 16.2]],
		"speed_mps": 1.0,
		"warning_lights": True,
		"reversing_bias": False,
		"body_height_m": 0.9,
		"base_height_m": 0.45,
		"base_z": 0.9,
	}]
	hazards["human"] = [
		{
			"path": "waypoints",
			"waypoints": [[10.0, 15.5], [10.0, 8.0]],
			"group_size": 1,
			"speed_mps": [0.7, 1.0],
			"motion_patterns": ["hesitation", "emerge_from_occlusion", "start_stop"],
			"interaction_modes": ["mutual_yield", "robot_yield"],
			"start_delay_s_min": 0.05,
			"start_delay_s_max": 0.2,
			"start_delay_s": 0.0,
		},
		{
			"path": "waypoints",
			"waypoints": [[10.0, 4.0], [10.0, 10.5]],
			"group_size": 1,
			"speed_mps": [0.7, 1.2],
			"motion_patterns": ["hesitation", "emerge_from_occlusion", "start_stop"],
			"interaction_modes": ["mutual_yield", "robot_yield"],
			"start_delay_s_min": 0.05,
			"start_delay_s_max": 0.2,
			"start_delay_s": 0.0,
		}
	]
	hazards["traction"] = hazards.get("traction", [])
	hazards["floor_events"] = hazards.get("floor_events", [])
	scn["taxonomy"]["multi_actor"] = True
	scn["taxonomy"]["human_behavior"] = True
	scn["taxonomy"]["occlusion"] = True

def _is_simple_forklift_aisle_prompt(text: str) -> bool:
	text_l = text.lower()
	has_forklift = _has_any(text_l, KEYWORDS["forklift"])
	has_aisle = "aisle" in text_l or "aisles" in text_l
	has_group = any(term in text_l for term in _GROUP_CROSS_TERMS) or _has_any(text_l, KEYWORDS["busy_crossing"])
	return has_forklift and has_aisle and has_group

def _apply_simple_forklift_aisle_layout(scn: Dict[str, Any]) -> None:
	layout = scn.setdefault("layout", {})
	layout["map_size_m"] = [20.0, 20.0]
	layout["start"] = [2.0, 9.0]
	layout["goal"] = [18.0, 9.0]
	geom = layout.setdefault("geometry", {})
	geom.clear()
	main_lane_y = 9.0
	forklift_lane_y = 12.0
	geom["main_aisle_lane_y"] = main_lane_y
	geom["forklift_aisle_lane_y"] = forklift_lane_y
	geom["main_cross_x"] = 10.0
	geom["racking"] = [
		{"id": "Rack_lower_W", "aabb": [1.0, 6.5, 8.5, 7.5], "height_m": 3.0},
		{"id": "Rack_lower_E", "aabb": [11, 6.5, 19.0, 7.5], "height_m": 3.0},
		{"id": "Rack_upper_W", "aabb": [1.0, 10.5, 8.5, 11.5], "height_m": 3.0},
		{"id": "Rack_upper_E", "aabb": [11, 10.5, 19.0, 11.5], "height_m": 3.0},
		{"id": "Rack_forklift_W", "aabb": [1.0, 13.5, 8.5, 14.5], "height_m": 3.0},
		{"id": "Rack_forklift_E", "aabb": [11, 13.5, 19.0, 14.5], "height_m": 3.0},
	]
	geom["open_storage"] = []
	geom["endcaps"] = []
	geom["blind_corners"] = []
	layout["aisles"] = [
		{"id": "A_main", "name": "main_aisle", "rect": [1.0, 7.5, 19.0, 10.5], "type": "straight", "pad": [0.0, 0.2, 0.0, 0.2], "racking": False},
		{"id": "A_forklift", "name": "forklift_aisle", "rect": [1.0, 11.8, 19.0, 13.6], "type": "straight", "pad": [0.0, 0.2, 0.0, 0.2], "racking": False},
		{"id": "A_dock", "name": "dock_staging", "rect": [1.0, 4.2, 19.0, 6.0], "type": "straight", "pad": [0.0, 0.2, 0.0, 0.2], "racking": False},
	]
	layout["floor_surfaces"] = [{
		"id": "base_floor",
		"type": "dry",
		"zone": [0.0, 0.0, 20.0, 20.0],
		"mu": 0.85,
		"brake_scale": 1.0,
		"slip_boost": 0.0,
		"imu_vibe_g": 0.2,
	}]
	layout["transition_zones"] = []
	layout["static_obstacles"] = []
	layout["lock_aisles"] = True
	layout["auto_aisles_from_paths"] = False
	layout["paths_define_aisles"] = False
	layout["simple_forklift_aisle"] = True
	# Pallet jack left in dock area
	layout["static_obstacles"].append({
		"id": "PalletJack_Dock",
		"type": "pallet_jack",
		"shape": "box",
		"aabb": [5.0, 4.5, 6.4, 5.1],
		"height": 1.0,
		"occlusion": True,
	})
	# Staging clutter
	# keep dock clutter lightweight to avoid out-of-aisle floating blocks
	layout["static_obstacles"].append({"id": "PalletStack_Main", "type": "standing_pallet", "aabb": [14.0, 7.6, 15.0, 8.6], "height": 1.2})

	hazards = scn.setdefault("hazards", {})
	hazards["vehicles"] = [
		{
			"id": "Forklift_Upper_01",
			"type": "forklift",
			"path": [[3.0, forklift_lane_y], [17.0, forklift_lane_y]],
			"speed_mps": 0.9,
			"warning_lights": True,
			"reflective": True,
			"ping_pong": True,
			"carrying_pallet": True,
		},
	]
	# Crossing workers and queued humans behind cart
	hazards["human"] = [
		{
			"path": "waypoints",
			"waypoints": [[geom["main_cross_x"], 7.0], [geom["main_cross_x"], 12.0]],
			"group_size": 5,
			"group_size_range": [3, 7],
			"rate_per_min": 2.0,
			"speed_mps": [0.8, 1.2],
			"motion_patterns": ["linear", "start_stop", "close_pass", "emerge_from_occlusion"],
			"interaction_modes": ["mutual_yield", "human_yield", "robot_yield"],
			"start_delay_s": 6.0,
			"group_spacing_m": 0.55,
		},
		{
			"path": "waypoints",
			"waypoints": [[4.0, main_lane_y], [9.0, main_lane_y]],
			"group_size": 5,
			"group_size_range": [3, 7],
			"rate_per_min": 3.0,
			"speed_mps": [0.5, 0.9],
			"motion_patterns": ["start_stop", "hesitation"],
			"interaction_modes": ["close_pass", "mutual_yield"],
			"start_delay_s": 8.0,
			"group_spacing_m": 0.5,
		},
	]

def _layout_list(scn: Dict[str, Any], key: str) -> List[Dict[str, Any]]:
	layout = scn.setdefault("layout", {})
	val = layout.get(key)
	if not isinstance(val, list):
		val = [] if val is None else list(val if isinstance(val, list) else [])
		layout[key] = val
	return val


def _geom_list(scn: Dict[str, Any], key: str) -> List[Dict[str, Any]]:
	layout = scn.setdefault("layout", {})
	geom = layout.setdefault("geometry", {})
	val = geom.get(key)
	if not isinstance(val, list):
		val = [] if val is None else list(val if isinstance(val, list) else [])
		geom[key] = val
	return val


def _merge_vehicle_roster(scn: Dict[str, Any], allow_types: set[str] | None = None) -> None:
	vehicles = scn.setdefault("hazards", {}).setdefault("vehicles", [])
	for tpl in default_vehicle_roster():
		if allow_types and tpl.get("type") not in allow_types:
			continue
		if any(v.get("id") == tpl.get("id") for v in vehicles):
			continue
		vehicles.append(copy.deepcopy(tpl))

def _ensure_vehicle(scn: Dict[str, Any], cfg: Dict[str, Any]) -> None:
	vehicles = scn.setdefault("hazards", {}).setdefault("vehicles", [])
	vid = cfg.get("id")
	if vid and any(v.get("id") == vid for v in vehicles):
		return
	vehicles.append(copy.deepcopy(cfg))


def _ensure_vehicle(scn: Dict[str, Any], cfg: Dict[str, Any]) -> None:
	vehicles = scn.setdefault("hazards", {}).setdefault("vehicles", [])
	vid = cfg.get("id")
	if vid and any(v.get("id") == vid for v in vehicles):
		return
	vehicles.append(copy.deepcopy(cfg))

def _ensure_vehicle(scn: Dict[str, Any], cfg: Dict[str, Any]) -> None:
	vehicles = scn.setdefault("hazards", {}).setdefault("vehicles", [])
	vid = cfg.get("id")
	if vid and any(v.get("id") == vid for v in vehicles):
		return
	vehicles.append(copy.deepcopy(cfg))


def _ensure_human_entry(scn: Dict[str, Any], idx: int = 0) -> Dict[str, Any]:
	humans = scn.setdefault("hazards", {}).setdefault("human", [])
	while len(humans) <= idx:
		humans.append({
			"path": "line",
			"rate_per_min": 2.0,
			"speed_mps": [0.8, 1.4],
			"motion_patterns": ["linear", "start_stop", "hesitation"],
		})
	cfg = humans[idx]
	cfg.setdefault("motion_patterns", ["linear"])
	cfg.setdefault("interaction_modes", ["mutual_yield", "human_yield", "robot_yield"])
	cfg.setdefault("visibility_awareness", "normal")
	return cfg


def _add_behavior_tag(cfg: Dict[str, Any], tag: str) -> None:
	tags = cfg.setdefault("behavior_tags", [])
	if tag not in tags:
		tags.append(tag)


def _add_floor_event(scn: Dict[str, Any], evt: Dict[str, Any]) -> None:
	scn.setdefault("hazards", {}).setdefault("floor_events", []).append(evt)


def _add_static_obstacle(scn: Dict[str, Any], obs: Dict[str, Any]) -> None:
	obs_list = _layout_list(scn, "static_obstacles")
	if "id" not in obs:
		obs["id"] = f"CustomObs_{len(obs_list)+1:02d}"
	obs_list.append(obs)


def _add_transition_zone(scn: Dict[str, Any], zone: Dict[str, Any]) -> None:
	tz = _layout_list(scn, "transition_zones")
	if "id" not in zone:
		zone["id"] = f"DoorExtra_{len(tz)+1:02d}"
	tz.append(zone)


def _has_any(text: str, words: list[str]) -> bool:
	text_l = text.lower()
	return any(w in text_l for w in words)


def parse_numbers(text: str) -> Dict[str, float]:
	"""
	Tiny extractor for a few knobs:
	- friction like "mu 0.35" or "friction 0.35"
	- duration like "time limit 200" / "200s"
	- human period like "every 30s" -> rate_per_min = 2.0
	- alt rate like "1.5/min" or "rate 1.5 per min"
	"""
	text_l = text.lower()
	out: Dict[str, float] = {}

	# friction
	m = re.search(r"(mu|friction)\s*([:=]?\s*)?(?P<val>0\.[0-9]+)", text_l)
	if m:
		out["mu"] = float(m.group("val"))

	# duration seconds
	m = re.search(r"(duration|time limit|limit)\s*([:=]?\s*)?(?P<sec>\d{2,4})\s*s?", text_l)
	if m:
		out["duration_s"] = float(m.group("sec"))

	# human crossing period like "every 30s"
	m = re.search(r"every\s*(?P<p>\d{1,4})\s*s", text_l)
	if m:
		period_s = float(m.group("p"))
		if period_s > 0:
			out["human_rate_per_min"] = 60.0 / period_s

	# explicit rate like "1.5/min" or "rate 1.5 per min"
	m = re.search(r"(rate|crossings?)\s*([:=]?\s*)?(?P<rpm>\d+(\.\d+)?)\s*/?\s*min", text_l)
	if m:
		out["human_rate_per_min"] = float(m.group("rpm"))

	return out


def prompt_to_scenario(prompt: str, n_runs: int = 100) -> Dict[str, Any]:
	scn = new_scenario(n_runs)
	text = prompt.lower()
	nums = parse_numbers(prompt)
	t_case = _is_t_intersection_prompt(text)
	main_lr_case = _is_main_aisle_lr_prompt(text) and not t_case
	simple_aisle_case = _is_simple_forklift_aisle_prompt(text) and not main_lr_case and not t_case
	if t_case:
		_apply_t_intersection_layout(scn)
	elif main_lr_case:
		_apply_main_aisle_lr_layout(scn)
	elif simple_aisle_case:
		_apply_simple_forklift_aisle_layout(scn)
	if ("aisle" in text and "no aisle" not in text and "without aisle" not in text and
	    not simple_aisle_case and not main_lr_case and not t_case):
		scn["layout"]["auto_aisles_from_paths"] = True

	# convenience handle for overrides
	def _so_root() -> Dict[str, Any]:
		return scn.setdefault("site_overrides", {})

	# --- Visibility ---
	if _has_any(text, KEYWORDS["visibility"]):
		scn["taxonomy"]["visibility"] = True
		so_root = _so_root()
		lid = so_root.setdefault("lidar", {})
		lid.setdefault("noise_sigma", 0.03)
		lid.setdefault("dropout", 0.05)
		lid.setdefault("latency_ms_max", 70)

	# --- Traction (wet patch) ---
	if _has_any(text, KEYWORDS["traction"]):
		mu = nums.get("mu", 0.45)
		# simple 2x3m patch near "dock" (heuristic)
		patch = {"zone": [12.0, 2.0, 14.0, 5.0], "mu": float(mu)}
		scn["hazards"]["traction"].append(patch)
		scn["taxonomy"]["traction"] = True

	if _has_any(text, KEYWORDS["cleaning"]):
		clean_zone = [5.0, 1.0, 9.0, 2.4]
		floor_list = _layout_list(scn, "floor_surfaces")
		floor_list.append({
			"id": f"cleaning_{len(floor_list)+1:02d}",
			"type": "cleaning_liquid",
			"zone": clean_zone,
			"mu": 0.33,
			"brake_scale": 1.7,
			"slip_boost": 0.6,
			"imu_vibe_g": 0.4,
		})
		_add_floor_event(scn, {
			"type": "cleaning_liquid",
			"zone": clean_zone,
			"spawn_time_s": 25.0,
			"spread_rate_mps": 0.1,
			"mu": 0.33,
		})
		scn["taxonomy"]["traction"] = True

	# --- Human crossing (generic, single stream) ---
	if _has_any(text, KEYWORDS["human"]) and not scn["layout"].get("main_lr_case") and not scn["layout"].get("t_case"):
		rate = nums.get("human_rate_per_min", 2.0)  # every 30s default
		cfg = _ensure_human_entry(scn, len(scn["hazards"]["human"]))
		cfg.update({
			"path": "line",
			"rate_per_min": float(rate),
			"speed_mps": [0.8, 1.4],
		})
		cfg.setdefault("motion_patterns", ["linear", "start_stop", "hesitation", "emerge_from_occlusion"])
		cfg.setdefault("interaction_modes", ["mutual_yield", "human_yield", "robot_yield", "close_pass"])
		cfg.setdefault("visibility_awareness", "normal")
		scn["taxonomy"]["human_behavior"] = True

	# --- Traffic / congestion keywords imply multi-actor environments ---
	if (_has_any(text, KEYWORDS["traffic"]) or
	    _has_any(text, KEYWORDS["busy_aisle"]) or
	    _has_any(text, KEYWORDS["congestion"])):
		scn["taxonomy"]["multi_actor"] = True
		if scn.get("layout", {}).get("main_lr_case"):
			vehicles = scn.setdefault("hazards", {}).setdefault("vehicles", [])
			# Two forklifts with pallets, opposing directions along the main aisle
			if not any(v.get("id") == "Forklift_Main_East" for v in vehicles):
				vehicles.append({
					"id": "Forklift_Main_East",
					"type": "forklift",
					"path": [[4.0, 9.1], [16.0, 9.1]],
					"speed_mps": 0.85,
					"warning_lights": True,
					"reversing_bias": False,
					"reflective": True,
					"ping_pong": True,
					"carrying_pallet": True,
				})
			if not any(v.get("id") == "Forklift_Main_West" for v in vehicles):
				vehicles.append({
					"id": "Forklift_Main_West",
					"type": "forklift",
					"path": [[16.5, 8.9], [3.5, 8.9]],
					"speed_mps": 0.85,
					"warning_lights": True,
					"reversing_bias": False,
					"reflective": True,
					"ping_pong": True,
					"carrying_pallet": True,
				})
			static_obs = _layout_list(scn, "static_obstacles")
			if not any(o.get("id") == "PalletStack_Congest_01" for o in static_obs):
				static_obs.append({"id": "PalletStack_Congest_01", "type": "standing_pallet", "aabb": [8.0, 5.2, 9.0, 6.2], "height": 1.2})
			if not any(o.get("id") == "PalletStack_Congest_02" for o in static_obs):
				static_obs.append({"id": "PalletStack_Congest_02", "type": "standing_pallet", "aabb": [12.0, 11.0, 13.0, 12.0], "height": 1.2})
		elif scn.get("layout", {}).get("simple_forklift_aisle"):
			# Dedicated forklift aisle already has a forklift; avoid duplicating vehicles from congestion keywords.
			pass

	# --- Overhangs / irregular loads increase occlusion pressure ---
	if _has_any(text, KEYWORDS["overhang"]):
		scn["taxonomy"]["occlusion"] = True

	# --- Narrow aisle hint (keep lightweight; world builder may interpret later) ---
	if _has_any(text, KEYWORDS["narrow"]):
		scn["layout"]["narrow_aisle_hint"] = True  # advisory flag for world builder
		# note: taxonomy doesn't have "narrow" in schema, but dict is open
		scn["taxonomy"]["narrow"] = True  # type: ignore[index]

	# --- Falling object emphasis (nudges injector probabilities via site_overrides) ---
	if _has_any(text, KEYWORDS["falling"]):
		so_root = _so_root()
		so_inj = so_root.setdefault("injectors", {})
		f = so_inj.setdefault("falling_object", {})
		# Bump falling object probability; leave others unchanged unless scenario says otherwise
		f.setdefault("p", 0.80)

	# --- Two-crosswalks special case (edge-focused, self-contained) ---
	if _has_any(text, KEYWORDS["two_crosswalks"]):
		# Ensure we have exactly two crosser configs (world currently uses first,
		# but world_digest / taxonomy still see both)
		scn.setdefault("hazards", {})["human"] = []
		for _ in range(2):
			cfg = _ensure_human_entry(scn, len(scn["hazards"]["human"]))
			cfg.update({
				"path": "line",
				"rate_per_min": nums.get("human_rate_per_min", 2.0),
				"speed_mps": [0.8, 1.4],
			})
			cfg.setdefault("motion_patterns", ["linear", "start_stop", "mutual_yield"])
		scn["taxonomy"]["human_behavior"] = True
		scn["taxonomy"]["multi_actor"] = True

		# Force overlap and faster crossings; add slip dynamics
		scn["hazards"]["human"][0].update({
			"cross_x": 10.0,
			"duration_s_running": 2.2,
			"p_slip": 0.60,
			"trigger_mu": 0.9, "trigger_sigma": 0.2,
		})
		scn["hazards"]["human"][1].update({
			"cross_x": 10.8,
			"duration_s_running": 2.2,
			"p_slip": 0.60,
			"trigger_mu": 0.9, "trigger_sigma": 0.2,
			"start_delay_s": 0.8,
		})

		# Scenario-local overrides that keep it “edge” even if the text didn’t say “wet”
		so_root = _so_root()
		so_root.setdefault("traction", {}).update({
			"mu_wet_min": 0.35,
			"mu_wet_max": 0.55,
		})
		so_root.update({
			"ensure_wet_corridor_pct": 0.85,
			"ensure_wet_corridor_width_m": 1.4,
		})

		# Keep fallen humans persistent and slippery
		so_root.setdefault("human", {}).update({
			"fall_duration_s": 10.0,    # linger longer
			"slip_min_exposure_s": 0.25 # brief but non-zero exposure
		})
		so_inj = so_root.setdefault("injectors", {})
		so_inj.setdefault("lidar_blackout", {}).update({"p": 0.70, "duration_s": 6.0, "sector_deg": 90.0})
		so_inj.setdefault("falling_object", {}).update({"p": 0.85})

		scn["taxonomy"]["occlusion"] = True
		scn["taxonomy"]["sensor_fault"] = True

	# --- Distracted human (phone, late reaction) ---
	if _has_any(text, KEYWORDS["distracted"]):
		scn["taxonomy"]["human_behavior"] = True
		so_root = _so_root()
		h = so_root.setdefault("human", {})
		# more likely to slip when distracted
		h.setdefault("slip_prob", 0.95)

	if _has_any(text, KEYWORDS["worker_carry"]):
		cfg = _ensure_human_entry(scn, 0)
		_add_behavior_tag(cfg, "carry_payload")
		cfg["carried_mass_kg"] = 12.0
		cfg["posture"] = "leaning"
		cfg.setdefault("motion_patterns", []).append("reduced_visibility")
		scn["taxonomy"]["human_behavior"] = True

	if _has_any(text, KEYWORDS["fast_walker"]):
		cfg = _ensure_human_entry(scn, 0)
		cfg["speed_mps"] = [1.4, 2.2]
		_add_behavior_tag(cfg, "fast_walk")
		cfg.setdefault("motion_patterns", []).append("zig_zag")
		scn["taxonomy"]["human_behavior"] = True

	if _has_any(text, KEYWORDS["child_actor"]):
		cfg = _ensure_human_entry(scn, len(scn["hazards"]["human"]))
		cfg.update({
			"path": "line",
			"rate_per_min": 1.0,
			"speed_mps": [0.6, 1.0],
			"height_scale": 0.6,
			"visibility_awareness": "reduced",
		})
		_add_behavior_tag(cfg, "child_height")
		scn["taxonomy"]["human_behavior"] = True

	# --- Busy crossing / groups / platoons ---
	if _has_any(text, KEYWORDS["busy_crossing"]):
		scn["taxonomy"]["human_behavior"] = True
		scn["taxonomy"]["multi_actor"] = True
		_ensure_human_entry(scn, 0)
		for idx in range(len(scn["hazards"]["human"])):
			cfg = _ensure_human_entry(scn, idx)
			cfg["group_size"] = max(2, int(cfg.get("group_size", 3)))
			cfg.setdefault("motion_patterns", []).append("close_pass")
			cfg.setdefault("interaction_modes", []).append("mutual_yield")

	# --- Blind corner / occluded crosswalks ---
	if _has_any(text, KEYWORDS["blind_corner"]) and not scn["layout"].get("t_case"):
		scn["taxonomy"]["occlusion"] = True
		so_root = _so_root()
		so_inj = so_root.setdefault("injectors", {})
		blk = so_inj.setdefault("lidar_blackout", {})
		blk.setdefault("p", 0.90)
		blk.setdefault("cx", 10.0)
		blk.setdefault("cy", 10.0)
		blk.setdefault("trigger_radius", 2.5)
		blk.setdefault("duration_s", 3.5)
		blk.setdefault("sector_deg", 80.0)
		blk.setdefault("sector_offset_deg", 0.0)
		layout = scn["layout"]
		layout["start"] = [2.0, 9.0]
		layout["goal"] = [18.0, 9.0]
		geom = layout.setdefault("geometry", {})
		endcaps = geom.setdefault("endcaps", [])
		geom["blind_corner_lane_x"] = 12.5
		if not any(g.get("id") == "PromptBlindCorner_main" for g in endcaps):
			endcaps.append({"id": "PromptBlindCorner_main", "aabb": [10.6, 8.5, 11.4, 16.0]})
			endcaps.append({"id": "PromptBlindCorner_side", "aabb": [6.2, 9.3, 10.6, 10.5]})
		scn["agents"][0]["max_speed_mps"] = max(float(scn["agents"][0].get("max_speed_mps", 1.2)), 1.5)
	if _has_any(text, KEYWORDS["high_shelf"]):
		racks = _geom_list(scn, "racking")
		racks.append({
			"id": f"HighShelf_{len(racks)+1:02d}",
			"aabb": [13.0, 4.0, 15.0, 12.5],
			"levels": 4,
			"height_m": 6.0,
			"type": "high_bay",
			"occlusion": {"reflectivity": 0.8, "lidar_shadow_deg": 90},
		})
		scn["taxonomy"]["occlusion"] = True

	# --- Forklift-related chaos (load overhangs, drops) ---
	if (_has_any(text, KEYWORDS["forklift"]) and
	    not scn["layout"].get("main_lr_case") and
	    not scn["layout"].get("t_case") and
	    not scn["layout"].get("simple_forklift_aisle")):
		scn["taxonomy"]["multi_actor"] = True
		geom = scn["layout"].setdefault("geometry", {})
		lane_x = geom.get("forklift_aisle_lane_x")
		lane_y = geom.get("forklift_aisle_lane_y")
		if lane_x is None:
			lane_x = geom.get("blind_corner_lane_x", 9.0)
		veh_list = scn.setdefault('hazards', {}).setdefault('vehicles', [])
		veh_id = f"Forklift_{len(veh_list)+1:02d}"
		if lane_y is not None or geom.get("forklift_aisle_lane_x") is not None:
			veh_id = f"ForkliftAisle_{len(veh_list)+1:02d}"
		f_default = {
			"id": veh_id,
			"type": "forklift",
			"path": [[2.0, lane_y if lane_y is not None else 2.0], [18.0, lane_y if lane_y is not None else 18.0]] if lane_y is not None else [[lane_x, 2.0], [lane_x, 18.0]],
			"speed_mps": 1.2,
			"warning_lights": True,
			"reversing_bias": True,
		}
		if lane_y is not None or geom.get("forklift_aisle_lane_x") is not None:
			f_default["reversing_bias"] = False
		_ensure_vehicle(scn, f_default)

	if _has_any(text, KEYWORDS["forklift_reverse"]):
		_ensure_vehicle(scn, {
			"id": f"ForkliftRev_{len(scn.setdefault('hazards', {}).setdefault('vehicles', []))+1:02d}",
			"type": "forklift",
			"path": [[6.0, 4.0], [14.0, 4.0]],
			"speed_mps": 1.0,
		})
		for veh in scn["hazards"].setdefault("vehicles", []):
			if veh.get("type") != "forklift":
				continue
			veh["reversing_bias"] = True
			veh["warning_lights"] = True
			veh["alarm"] = "backup_beeper"
			veh["rear_occlusion_deg"] = 70.0
		scn["taxonomy"]["multi_actor"] = True

	# --- Pallet jack / hand-cart => denser clutter & more nonhuman contacts ---
	if _has_any(text, KEYWORDS["pallet_jack"]):
		scn["taxonomy"]["multi_actor"] = True
		_ensure_vehicle(scn, {
			"id": f"PalletJack_{len(scn.setdefault('hazards', {}).setdefault('vehicles', []))+1:02d}",
			"type": "pallet_jack",
			"path": [[8.5, 2.0], [12.0, 2.0]],
			"speed_mps": 0.9,
		})

	if _has_any(text, KEYWORDS["cart_block"]):
		aabb = [9.2, 7.6, 10.8, 8.8]
		# In the parallel/forklift-aisle layout, move the cart off the crosswalk to the west side
		if scn.get("layout", {}).get("simple_forklift_aisle"):
			aabb = [12.8, 8.5, 13.8, 9.3]  # right-side, off the crosswalk
		_add_static_obstacle(scn, {
			"type": "cart_block",
			"shape": "box",
			"aabb": aabb,
			"height": 1.2,
			"occlusion": True,
		})
		scn["taxonomy"]["multi_actor"] = True

	if _has_any(text, KEYWORDS["transition"]):
		_add_transition_zone(scn, {
			"type": "fire_door",
			"zone": [4.0, 0.0, 5.0, 1.4],
			"threshold_cm": 2.0,
			"slope_deg": 4.0,
			"sensor_shadow": True,
		})
		scn["taxonomy"]["occlusion"] = True

	# --- Clothing / reflectivity — affect lidar noise/dropout ---
	if _has_any(text, KEYWORDS["reflective"]):
		scn["taxonomy"]["visibility"] = True
		so_root = _so_root()
		lid = so_root.setdefault("lidar", {})
		lid.setdefault("noise_sigma", 0.01)
		lid.setdefault("dropout", 0.005)

	if _has_any(text, KEYWORDS["dark_clothing"]):
		scn["taxonomy"]["visibility"] = True
		so_root = _so_root()
		lid = so_root.setdefault("lidar", {})
		lid.setdefault("noise_sigma", 0.03)
		lid.setdefault("dropout", 0.08)

	# --- Sensor faults / ghost obstacles / glitchy lidar ---
	if _has_any(text, KEYWORDS["sensor_fault"]):
		scn["taxonomy"]["sensor_fault"] = True
		so_root = _so_root()
		lid = so_root.setdefault("lidar", {})
		lid.setdefault("noise_sigma", 0.05)
		lid.setdefault("dropout", 0.15)
		lid.setdefault("latency_ms_max", 120)
		so_inj = so_root.setdefault("injectors", {})
		blk = so_inj.setdefault("lidar_blackout", {})
		blk.setdefault("p", 0.60)
		blk.setdefault("duration_s", 5.0)
		blk.setdefault("sector_deg", 120.0)

	# --- Degraded mode: slower lidar, more jitter ---
	if _has_any(text, KEYWORDS["degraded_mode"]):
		scn["taxonomy"]["sensor_fault"] = True
		# slower lidar update directly via sensors
		scn["sensors"].setdefault("lidar", {})["hz"] = 5.0
		so_root = _so_root()
		lid = so_root.setdefault("lidar", {})
		lid.setdefault("latency_ms_max", 150)

	# --- Duration override (if present in text) ---
	if "duration_s" in nums:
		scn["runtime"]["duration_s"] = int(nums["duration_s"])  # type: ignore[arg-type]

	if simple_aisle_case:
		scn["hazards"]["human"] = []
		cfg = _ensure_human_entry(scn, 0)
		cfg.update({
			"path": "waypoints",
			"rate_per_min": 2.0,
			"speed_mps": [0.8, 1.4],
			"group_size": 3,
			"waypoints": [[10.0, 6.5], [10.0, 12.5]],
		})
		cfg["motion_patterns"] = ["linear", "start_stop", "close_pass"]
		cfg["interaction_modes"] = ["mutual_yield", "human_yield", "robot_yield"]
		scn["taxonomy"]["human_behavior"] = True

	_align_vehicle_paths(scn)

	layout = scn.get("layout", {})
	geom = layout.get("geometry", {}) or {}

	# Align human crossings with blind-corner geometry so they don't spawn inside walls
	if scn["taxonomy"].get("occlusion"):
		lane_x = geom.get("blind_corner_lane_x")
		if lane_x is None:
			for endcap in geom.get("endcaps", []):
				aabb = endcap.get("aabb")
				if not aabb:
					continue
				if "PromptBlindCorner" in (endcap.get("id") or ""):
					lane_x = aabb[2] + 1.0
					break
		if lane_x is not None:
			for cfg in scn["hazards"].get("human", []):
				cfg["cross_x"] = lane_x

	if simple_aisle_case:
		main_cross = geom.get("main_cross_x", 10.0)
		for cfg in scn["hazards"].get("human", []):
			cfg["cross_x"] = main_cross
	if main_lr_case:
		cross = geom.get("main_aisle_lane_x", 10.0)
		for cfg in scn["hazards"].get("human", []):
			cfg["cross_x"] = cross

	if layout.get("t_case"):
		branch_x = geom.get("branch_lane_x", 10.0)
		for cfg in scn["hazards"].get("human", []):
			cfg["cross_x"] = branch_x

	return scn

def _align_vehicle_paths(scn: Dict[str, Any]) -> None:
	layout = scn.get("layout", {})
	if layout.get("main_lr_case") or layout.get("t_case"):
		return
	Lx = float(layout.get("map_size_m", [20.0, 20.0])[0])
	geom = layout.get("geometry", {})
	endcaps = geom.get("endcaps", [])
	lane_hint = geom.get("blind_corner_lane_x")
	forklift_lane_x = geom.get("forklift_aisle_lane_x")
	forklift_lane_y = geom.get("forklift_aisle_lane_y")
	vehicles = scn.setdefault("hazards", {}).setdefault("vehicles", [])
	for veh in vehicles:
		if veh.get("type") != "forklift":
			continue
		path = veh.get("path") or []
		if forklift_lane_y is not None and len(path) >= 2:
			if all(abs(float(pt[1]) - float(forklift_lane_y)) < 0.3 for pt in path[:2]):
				continue
		if forklift_lane_x is not None and len(path) >= 2 and forklift_lane_y is None:
			if all(abs(float(pt[0]) - float(forklift_lane_x)) < 0.3 for pt in path[:2]):
				continue
		if forklift_lane_y is not None:
			if len(path) >= 2:
				veh["path"] = [[path[0][0], forklift_lane_y], [path[1][0], forklift_lane_y]]
				continue
		if lane_hint is not None:
			if len(path) >= 2:
				veh["path"] = [[lane_hint, path[0][1]], [lane_hint, path[1][1]]]
				continue
		if not path:
			continue
		xs = [pt[0] for pt in path]
		if (max(xs) - min(xs)) > 0.5:
			continue
		lane_x = xs[0]
		new_x = lane_x
		for endcap in endcaps:
			aabb = endcap.get("aabb")
			if not aabb:
				continue
			if aabb[0] <= lane_x <= aabb[2]:
				margin = 0.9
				east = aabb[2] + margin
				west = aabb[0] - margin
				if west > 0.5:
					new_x = west
				elif east < Lx - 0.5:
					new_x = east
				break
		if new_x != lane_x:
			veh["path"] = [[new_x, pt[1]] for pt in path]

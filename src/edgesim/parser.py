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

_PARALLEL_AISLE_TERMS = [
	"parallel aisle",
	"parallel aisles",
	"two aisles",
	"double aisles",
	"twin aisles",
	"two parallel aisles",
]

_NUM_RE = r"-?\d+(?:\.\d+)?"
_SEG_RE = rf"(?P<x0>{_NUM_RE})\s*,\s*(?P<y0>{_NUM_RE})\s*(?:to|->|-)\s*(?P<x1>{_NUM_RE})\s*,\s*(?P<y1>{_NUM_RE})"
_PT_RE = rf"(?P<x>{_NUM_RE})\s*,\s*(?P<y>{_NUM_RE})"

_AISLE_SEG_RE = re.compile(rf"(?P<label>main\s+aisle|aisle)[^\n]*?from\s+{_SEG_RE}", re.IGNORECASE)
_HUMAN_SEG_RE = re.compile(
	rf"(?:human(?:\s+crossing)?|pedestrian)[^\n]*?(?:from|starting at|start at)\s+"
	rf"(?P<x0>{_NUM_RE})\s*,\s*(?P<y0>{_NUM_RE})"
	rf"(?:[^\n]*?(?:to|->|-)\s*(?P<x1>{_NUM_RE})\s*,\s*(?P<y1>{_NUM_RE}))?",
	re.IGNORECASE,
)
_VEHICLE_SEG_RE = re.compile(rf"(forklift|vehicle|cart|tugger)[^\n]*?from\s+{_SEG_RE}", re.IGNORECASE)
_PATCH_SEG_RE = re.compile(rf"(wet\s+patch|spill|oil|traction)[^\n]*?(?:from|covering|zone)\s+{_SEG_RE}", re.IGNORECASE)
_STATIC_PT_RE = re.compile(rf"(pallet|obstacle|cart block|column)[^\n]*?(?:at|center(?:ed)? at)\s+{_PT_RE}", re.IGNORECASE)

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

def _wants_parallel_aisles(text: str) -> bool:
	text_l = text.lower()
	return any(term in text_l for term in _PARALLEL_AISLE_TERMS)

def _is_no_human(text: str) -> bool:
	text_l = text.lower()
	return ("no human" in text_l) or ("no workers" in text_l) or ("no pedestrians" in text_l)

def _is_no_vehicle(text: str) -> bool:
	text_l = text.lower()
	return ("no vehicle" in text_l) or ("no vehicles" in text_l) or ("no forklift" in text_l) or ("no forklifts" in text_l)

def _is_single_straight_prompt(text: str) -> bool:
	text_l = text.lower()
	return ("single straight aisle" in text_l) or ("one aisle" in text_l) or ("robot going left to right" in text_l)

def _is_parallel_pass_prompt(text: str) -> bool:
	text_l = text.lower()
	return ("parallel warehouse aisles" in text_l) or ("parallel aisles" in text_l and "worker" in text_l)

def _is_narrow_cross_prompt(text: str) -> bool:
	text_l = text.lower()
	return ("narrow aisle" in text_l and "crossing" in text_l)

def _is_wide_main_prompt(text: str) -> bool:
	text_l = text.lower()
	return (("wide aisle" in text_l or "main corridor" in text_l or "wide main aisle" in text_l) and "no human" in text_l and "no vehicle" in text_l)

def _axis_aligned_rect_from_centerline(start: tuple[float, float], end: tuple[float, float], width: float) -> List[float]:
	x0, y0 = start
	x1, y1 = end
	half_w = max(0.05, float(width) * 0.5)
	if abs(y1 - y0) < 1e-6:
		x_lo, x_hi = sorted([x0, x1])
		return [x_lo, y0 - half_w, x_hi, y0 + half_w]
	if abs(x1 - x0) < 1e-6:
		y_lo, y_hi = sorted([y0, y1])
		return [x0 - half_w, y_lo, x0 + half_w, y_hi]
	x_lo, x_hi = sorted([x0, x1])
	y_lo, y_hi = sorted([y0, y1])
	return [x_lo - half_w, y_lo - half_w, x_hi + half_w, y_hi + half_w]

def _add_columns(static_obs: List[Dict[str, Any]], centers: List[tuple[float, float]],
                 radius: float = 0.2, height: float = 2.4, prefix: str = "Column") -> None:
	for idx, (cx, cy) in enumerate(centers):
		static_obs.append({
			"id": f"{prefix}_{idx+1:02d}",
			"type": "column",
			"shape": "cylinder",
			"pos": [float(cx), float(cy)],
			"radius": float(radius),
			"height": float(height),
		})

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
	hazards["vehicles"] = [
		{
			"id": "Forklift_MainParallel",
			"type": "forklift",
			"path": [[2.0, 9.0], [18.0, 9.0]],
			"speed_mps": 0.95,
			"ping_pong": True,
			"warning_lights": True,
			"carrying_pallet": True,
		},
	]
	hazards["human"] = [{
		"path": "waypoints",
		"waypoints": [[10.0, 7.2], [10.0, 10.8]],
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
		{"type": "standing_pallet", "aabb": [5.5, 4.5, 6.6, 5.4], "height": 1.0},
		{"type": "standing_pallet", "aabb": [13.6, 13.0, 14.6, 13.9], "height": 1.0},
		{"type": "standing_pallet", "aabb": [4.0, 2.5, 5.0, 3.5], "height": 1.0},
		# keep cart clutter off the main aisle corridor to avoid blocking the robot lane
		{"type": "cart_block", "aabb": [2.0, 1.5, 3.2, 2.5], "height": 1.2},
	]
	_ensure_map_bounds(layout, hazards)
	_ensure_start_goal_open(layout, tuple(layout["map_size_m"]))

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
	_ensure_map_bounds(layout, hazards)
	_ensure_start_goal_open(layout, tuple(layout["map_size_m"]))

def _is_simple_forklift_aisle_prompt(text: str) -> bool:
	text_l = text.lower()
	has_forklift = _has_any(text_l, KEYWORDS["forklift"])
	has_aisle = "aisle" in text_l or "aisles" in text_l
	has_group = any(term in text_l for term in _GROUP_CROSS_TERMS) or _has_any(text_l, KEYWORDS["busy_crossing"])
	return has_forklift and has_aisle and has_group

def _apply_simple_forklift_aisle_layout(scn: Dict[str, Any]) -> None:
	layout = scn.setdefault("layout", {})
	layout["map_size_m"] = [24.0, 22.0]
	layout["start"] = [3.0, 12.0]
	layout["goal"] = [21.0, 12.0]
	geom = layout.setdefault("geometry", {})
	geom.clear()
	# Geometry plan (top-down, meters):
	# x: 0→24, y: 0→22
	# - Main aisle along y=12 (1.4 m), with racks on both sides
	# - Forklift aisle along y=15 (1.5 m, high-bay racks)
	# - Dock/staging band y≈2–9 with a 2.2 m dock lane at y=6.5
	# - North–south spine at x=12 tying dock ↔ main ↔ forklift, padded at junctions
	# - Endcaps and columns create blind corners/occlusion without blocking aisles
	main_y = 12.0
	forklift_y = 15.0
	dock_y = 6.5
	spine_x = 12.0
	x_min, x_max = 2.0, 22.0
	main_rect = _axis_aligned_rect_from_centerline((x_min, main_y), (x_max, main_y), 1.4)
	forklift_rect = _axis_aligned_rect_from_centerline((x_min, forklift_y), (x_max, forklift_y), 1.5)
	dock_rect = _axis_aligned_rect_from_centerline((x_min, dock_y), (x_max, dock_y), 2.2)
	spine_rect = _axis_aligned_rect_from_centerline((spine_x, 4.0), (spine_x, 18.0), 1.3)

	geom["main_aisle_lane_y"] = main_y
	geom["forklift_aisle_lane_y"] = forklift_y
	geom["main_cross_x"] = spine_x
	geom["dock_lane_y"] = dock_y
	geom["racking"] = []
	geom["open_storage"] = []
	geom["endcaps"] = []
	geom["blind_corners"] = []

	layout["aisles"] = [
		{"id": "A_main", "name": "main_aisle", "rect": main_rect, "type": "straight", "pad": [0.05, 0.15, 0.05, 0.15], "racking": {"gap_m": 0.2, "depth_m": 0.95, "height_m": 3.6, "type": "rack"}},
		{"id": "A_forklift", "name": "forklift_aisle", "rect": forklift_rect, "type": "straight", "pad": [0.05, 0.15, 0.05, 0.15], "racking": {"gap_m": 0.2, "depth_m": 0.95, "height_m": 4.2, "type": "high_bay"}, "high_bay": True},
		{"id": "A_dock", "name": "dock_lane", "rect": dock_rect, "type": "straight", "pad": [0.05, 0.05, 0.05, 0.05], "racking": False, "mu": 0.78},
		{"id": "A_spine", "name": "spine_connector", "rect": spine_rect, "type": "straight", "pad": [0.05, 0.2, 0.05, 0.2], "racking": False},
	]
	layout["junctions"] = [
		{"id": "J_main_spine", "rect": [spine_x - 0.9, main_y - 0.9, spine_x + 0.9, main_y + 0.9], "type": "cross", "pad": [0.15, 0.15, 0.15, 0.15]},
		{"id": "J_forklift_spine", "rect": [spine_x - 0.9, forklift_y - 0.9, spine_x + 0.9, forklift_y + 0.9], "type": "cross", "pad": [0.2, 0.2, 0.2, 0.2]},
		{"id": "J_dock_spine", "rect": [spine_x - 1.0, dock_y - 1.1, spine_x + 1.0, dock_y + 1.1], "type": "cross", "pad": [0.2, 0.1, 0.2, 0.1]},
	]
	layout["floor_surfaces"] = [
		{"id": "base_floor", "type": "dry", "zone": [0.0, 0.0, 24.0, 22.0], "mu": 0.85, "brake_scale": 1.0, "slip_boost": 0.0, "imu_vibe_g": 0.2},
		{"id": "dock_slab", "type": "wet", "zone": [1.0, 2.0, 23.0, 9.0], "mu": 0.55, "brake_scale": 1.25, "slip_boost": 0.35, "imu_vibe_g": 0.25},
		{"id": "spine_grit", "type": "rough", "zone": [10.6, 8.2, 13.4, 18.5], "mu": 0.75, "brake_scale": 1.1, "slip_boost": 0.0, "imu_vibe_g": 0.3},
	]
	layout["transition_zones"] = []
	layout["static_obstacles"] = []

	geom["open_storage"].append({"id": "DockStorage_W", "aabb": [3.0, 2.4, 7.0, 4.4], "height_m": 1.4, "type": "open_storage"})
	geom["open_storage"].append({"id": "DockStorage_E", "aabb": [17.0, 2.6, 21.0, 4.6], "height_m": 1.4, "type": "open_storage"})
	geom["endcaps"].extend([
		{"id": "Endcap_Main_NE", "aabb": [12.65, main_y + 0.5, 13.45, main_y + 1.35]},
		{"id": "Endcap_Main_SW", "aabb": [10.55, main_y - 1.35, 11.35, main_y - 0.5]},
		{"id": "Endcap_Dock_SE", "aabb": [12.6, dock_y + 0.55, 13.4, dock_y + 1.15]},
		{"id": "Endcap_Dock_NW", "aabb": [10.6, dock_y - 1.15, 11.4, dock_y - 0.55]},
	])

	static_obs = [
		{"id": "PalletJack_Dock", "type": "pallet_jack", "shape": "box", "aabb": [4.6, 4.6, 6.0, 5.3], "height": 1.0, "occlusion": True},
		{"id": "PalletStack_Staging", "type": "standing_pallet", "aabb": [7.2, 5.4, 8.2, 6.4], "height": 1.2},
		{"id": "PalletStack_Spine", "type": "standing_pallet", "aabb": [13.3, 10.9, 14.1, 11.7], "height": 1.0},
		{"id": "Cart_Block_Main", "type": "cart_block", "shape": "box", "aabb": [9.4, 11.4, 10.4, 12.2], "height": 1.25, "occlusion": True},
	]
	_add_columns(static_obs, [
		(4.5, 10.6), (8.5, 13.6), (14.5, 10.6),
		(4.5, 13.4), (8.5, 10.4), (14.5, 13.4),
	], radius=0.18, height=2.4, prefix="ColumnGrid")
	layout["static_obstacles"] = static_obs

	hazards = scn.setdefault("hazards", {})
	hazards["vehicles"] = [
		{
			"id": "Forklift_Upper_01",
			"type": "forklift",
			"path": [[x_min + 1.0, forklift_y], [x_max - 1.0, forklift_y]],
			"speed_mps": 0.9,
			"warning_lights": True,
			"reflective": True,
			"ping_pong": True,
			"carrying_pallet": True,
		},
	]
	# Crossing workers and queued humans behind cart, aligned to aisle geometry
	hazards["human"] = [
		{
			"path": "waypoints",
			"waypoints": [[spine_x, main_y - 0.7], [spine_x, main_y + 0.8]],
			"group_size": 3,
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
			"waypoints": [[4.5, main_y], [9.5, main_y]],
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
	layout["lock_aisles"] = True
	layout["auto_aisles_from_paths"] = False
	layout["paths_define_aisles"] = False
	layout["simple_forklift_aisle"] = True
	scn["taxonomy"]["multi_actor"] = True
	scn["taxonomy"]["human_behavior"] = True
	scn["taxonomy"]["occlusion"] = True
	_ensure_map_bounds(layout, hazards)
	_ensure_start_goal_open(layout, tuple(layout["map_size_m"]))

def _apply_parallel_aisles_layout(scn: Dict[str, Any]) -> None:
	layout = scn.setdefault("layout", {})
	if layout.get("aisles"):
		return
	map_size = layout.get("map_size_m") or [20.0, 20.0]
	try:
		Lx, Ly = float(map_size[0]), float(map_size[1])
	except Exception:
		Lx, Ly = 20.0, 20.0
	x0 = max(0.5, 0.1 * Lx)
	x1 = max(x0 + 2.0, Lx - 0.5)
	width = max(1.8, 0.08 * Ly)
	gap = max(3.0, 0.25 * Ly)
	first_yc = max(width, 0.18 * Ly)
	second_yc = min(Ly - width, first_yc + gap)

	def _rect(yc: float) -> List[float]:
		hw = width * 0.5
		return [x0, max(0.4, yc - hw), x1, min(Ly - 0.4, yc + hw)]

	aisles = layout.setdefault("aisles", [])
	aisle_1 = _rect(first_yc)
	aisle_2 = _rect(second_yc)
	rack_cfg = {"gap_m": 0.15, "depth_m": 0.95, "height_m": 3.4, "type": "rack"}
	aisles.extend([
		{"id": "A_par_01", "name": "aisle_1", "rect": aisle_1, "type": "straight", "racking": rack_cfg},
		{"id": "A_par_02", "name": "aisle_2", "rect": aisle_2, "type": "straight", "racking": rack_cfg},
	])

	layout["lock_aisles"] = True
	layout["auto_aisles_from_paths"] = False
	layout["paths_define_aisles"] = False

	default_start = [2.0, 10.0]
	default_goal = [18.0, 10.0]
	# Keep caller-provided start/goal; only realign the defaults so the robot spawns on a lane.
	if layout.get("start") == default_start and layout.get("goal") == default_goal:
		mid_y = 0.5 * (aisle_2[1] + aisle_2[3])
		layout["start"] = [x0 + 0.6, mid_y]
		layout["goal"] = [x1 - 0.6, mid_y]
	hazards = scn.setdefault("hazards", {})
	_ensure_map_bounds(layout, hazards)
	_ensure_start_goal_open(layout, tuple(layout["map_size_m"]))

def _clamp_xy(pt: tuple[float, float], bounds: tuple[float, float]) -> List[float]:
	x, y = pt
	Lx, Ly = bounds
	return [
		max(0.0, min(Lx, float(x))),
		max(0.0, min(Ly, float(y))),
	]

def _clamp_rect(rect: List[float], bounds: tuple[float, float]) -> List[float]:
	x0, y0, x1, y1 = rect
	Lx, Ly = bounds
	x0, x1 = sorted([float(x0), float(x1)])
	y0, y1 = sorted([float(y0), float(y1)])
	return [
		max(0.0, min(Lx, x0)),
		max(0.0, min(Ly, y0)),
		max(0.0, min(Lx, x1)),
		max(0.0, min(Ly, y1)),
	]

def _rect_contains_pt(rect: List[float] | tuple[float, float, float, float], pt: tuple[float, float]) -> bool:
	x0, y0, x1, y1 = rect
	px, py = pt
	return (x0 <= px <= x1) and (y0 <= py <= y1)

def _project_point_to_rects(pt: tuple[float, float], rects: List[List[float] | tuple[float, float, float, float]]) -> List[float]:
	if not rects:
		return [float(pt[0]), float(pt[1])]
	px, py = pt
	best = None
	best_d2 = None
	for rect in rects:
		x0, y0, x1, y1 = rect
		nx = min(max(px, x0), x1)
		ny = min(max(py, y0), y1)
		dx = px - nx
		dy = py - ny
		d2 = dx * dx + dy * dy
		if best_d2 is None or d2 < best_d2:
			best_d2 = d2
			best = (nx, ny)
	return [float(best[0]), float(best[1])] if best is not None else [float(px), float(py)]

def _iter_aabb_slots(layout: Dict[str, Any], hazards: Dict[str, Any]) -> List[tuple[str, Dict[str, Any], str]]:
	slots: List[tuple[str, Dict[str, Any], str]] = []
	for aisle in layout.get("aisles", []) or []:
		slots.append(("aisle", aisle, "rect"))
	for junction in layout.get("junctions", []) or []:
		if junction.get("rect"):
			slots.append(("junction", junction, "rect"))
	for obstacle in layout.get("static_obstacles", []) or []:
		if obstacle.get("aabb"):
			slots.append(("static", obstacle, "aabb"))
	geom = layout.get("geometry", {}) or {}
	for rack in geom.get("racking", []) or []:
		if rack.get("aabb"):
			slots.append(("rack", rack, "aabb"))
	for endcap in geom.get("endcaps", []) or []:
		if endcap.get("aabb"):
			slots.append(("endcap", endcap, "aabb"))
	for open_s in geom.get("open_storage", []) or []:
		if open_s.get("aabb"):
			slots.append(("open_storage", open_s, "aabb"))
	for patch in hazards.get("traction", []) or []:
		if patch.get("zone"):
			slots.append(("traction", patch, "zone"))
	for surf in layout.get("floor_surfaces", []) or []:
		if surf.get("zone"):
			slots.append(("floor_surface", surf, "zone"))
	return slots

def _ensure_map_bounds(layout: Dict[str, Any], hazards: Dict[str, Any], margin: float = 0.5) -> tuple[float, float]:
	map_size = layout.get("map_size_m", [20.0, 20.0])
	try:
		Lx, Ly = float(map_size[0]), float(map_size[1])
	except Exception:
		Lx, Ly = 20.0, 20.0
	max_x, max_y = Lx, Ly
	min_x, min_y = 0.0, 0.0
	for pt in (layout.get("start"), layout.get("goal")):
		if isinstance(pt, (list, tuple)) and len(pt) == 2:
			max_x = max(max_x, float(pt[0]))
			max_y = max(max_y, float(pt[1]))
			min_x = min(min_x, float(pt[0]))
			min_y = min(min_y, float(pt[1]))
	for _, obj, key in _iter_aabb_slots(layout, hazards):
		rect = obj.get(key)
		if not rect or len(rect) != 4:
			continue
		rect = list(map(float, rect))
		x0, y0, x1, y1 = rect
		x0, x1 = sorted([x0, x1])
		y0, y1 = sorted([y0, y1])
		obj[key] = [x0, y0, x1, y1]
		max_x = max(max_x, x1)
		max_y = max(max_y, y1)
		min_x = min(min_x, x0)
		min_y = min(min_y, y0)
	if max_x > Lx - 1e-6:
		Lx = max_x + float(margin)
	if max_y > Ly - 1e-6:
		Ly = max_y + float(margin)
	if min_x < 0.0 or min_y < 0.0:
		# keep origin anchored at 0, clamp offending rects instead of shifting frame
		min_x = max(0.0, min_x)
		min_y = max(0.0, min_y)
	layout["map_size_m"] = [Lx, Ly]
	bounds = (Lx, Ly)
	for _, obj, key in _iter_aabb_slots(layout, hazards):
		rect = obj.get(key)
		if not rect or len(rect) != 4:
			continue
		obj[key] = _clamp_rect(rect, bounds)
	if isinstance(layout.get("start"), (list, tuple)) and len(layout["start"]) == 2:
		layout["start"] = _clamp_xy(tuple(layout["start"]), bounds)
	if isinstance(layout.get("goal"), (list, tuple)) and len(layout["goal"]) == 2:
		layout["goal"] = _clamp_xy(tuple(layout["goal"]), bounds)
	return bounds

def _ensure_start_goal_open(layout: Dict[str, Any], bounds: tuple[float, float]) -> None:
	walkable: List[List[float]] = []
	for aisle in layout.get("aisles", []) or []:
		if aisle.get("rect"):
			walkable.append(list(map(float, aisle["rect"])))
	for junc in layout.get("junctions", []) or []:
		rect = junc.get("rect")
		if rect:
			walkable.append(list(map(float, rect)))
	geom = layout.get("geometry", {}) or {}
	for zone in geom.get("open_storage", []) or []:
		if zone.get("aabb"):
			walkable.append(list(map(float, zone["aabb"])))
	solid: List[List[float]] = []
	for rack in geom.get("racking", []) or []:
		if rack.get("aabb"):
			solid.append(list(map(float, rack["aabb"])))
	for endcap in geom.get("endcaps", []) or []:
		if endcap.get("aabb"):
			solid.append(list(map(float, endcap["aabb"])))
	for obs in layout.get("static_obstacles", []) or []:
		if obs.get("aabb"):
			solid.append(list(map(float, obs["aabb"])))

	def _adjust(pt: List[float]) -> List[float]:
		pt = _clamp_xy(tuple(pt), bounds)
		if walkable and not any(_rect_contains_pt(rect, pt) for rect in walkable):
			pt = _project_point_to_rects(pt, walkable)
		# push out of solids if any
		for rect in solid:
			if _rect_contains_pt(rect, pt):
				if walkable:
					pt = _project_point_to_rects(pt, walkable)
				else:
					x0, y0, x1, y1 = rect
					dx = min(abs(pt[0] - x0), abs(pt[0] - x1))
					dy = min(abs(pt[1] - y0), abs(pt[1] - y1))
					if dx < dy:
						pt[0] = x0 - 0.1 if pt[0] < (0.5 * (x0 + x1)) else x1 + 0.1
					else:
						pt[1] = y0 - 0.1 if pt[1] < (0.5 * (y0 + y1)) else y1 + 0.1
				pt = _clamp_xy(tuple(pt), bounds)
		return [float(pt[0]), float(pt[1])]

	for key in ("start", "goal"):
		pt = layout.get(key)
		if isinstance(pt, (list, tuple)) and len(pt) == 2:
			layout[key] = _adjust([float(pt[0]), float(pt[1])])

def _collect_prompt_coords(prompt: str) -> tuple[float, float]:
	max_x = 0.0
	max_y = 0.0

	def _upd(val: float, axis: str) -> None:
		nonlocal max_x, max_y
		if axis == "x":
			max_x = max(max_x, val)
		else:
			max_y = max(max_y, val)

	for rg in (_AISLE_SEG_RE, _HUMAN_SEG_RE, _VEHICLE_SEG_RE, _PATCH_SEG_RE):
		for m in rg.finditer(prompt):
			for key in ("x0", "x1", "y0", "y1"):
				if key in m.groupdict() and m.group(key):
					val = float(m.group(key))
					_upd(val, "x" if key.startswith("x") else "y")
	for m in _STATIC_PT_RE.finditer(prompt):
		if m.group("x") and m.group("y"):
			_upd(float(m.group("x")), "x")
			_upd(float(m.group("y")), "y")
	return max_x, max_y

def _apply_coordinate_overrides(scn: Dict[str, Any], prompt: str, nums: Dict[str, float] | None = None) -> Dict[str, bool]:
	layout = scn.setdefault("layout", {})
	hazards = scn.setdefault("hazards", {})
	Lx, Ly = layout.get("map_size_m", [20.0, 20.0])
	max_prompt_x, max_prompt_y = _collect_prompt_coords(prompt)
	if max_prompt_x > Lx - 0.5 or max_prompt_y > Ly - 0.5:
		Lx = max(Lx, max_prompt_x + 1.0)
		Ly = max(Ly, max_prompt_y + 1.0)
		layout["map_size_m"] = [Lx, Ly]
	bounds = (float(Lx), float(Ly))
	flags = {"aisle": False, "human": False, "vehicle": False, "traction": False, "static": False, "cart_block": False}

	aisles: List[Dict[str, Any]] = []
	vehicle_entries: List[Dict[str, Any]] = []
	traction_entries: List[Dict[str, Any]] = []

	for m in _AISLE_SEG_RE.finditer(prompt):
		x0 = float(m.group("x0")); y0 = float(m.group("y0"))
		x1 = float(m.group("x1")); y1 = float(m.group("y1"))
		start = tuple(_clamp_xy((x0, y0), bounds))
		end = tuple(_clamp_xy((x1, y1), bounds))
		width = 1.4 if "main" in m.group("label").lower() else 1.2
		rect = _clamp_rect(_axis_aligned_rect_from_centerline(start, end, width), bounds)
		aisles.append({
			"id": f"A_coord_{len(aisles)+1:02d}",
			"name": m.group("label").strip().replace(" ", "_"),
			"rect": rect,
			"type": "straight",
			"racking": {"gap_m": 0.15, "depth_m": 0.9, "height_m": 3.2, "type": "rack"},
		})
	if aisles:
		layout["aisles"] = aisles
		layout["lock_aisles"] = True
		layout["_raw_coord_aisles"] = True
		flags["aisle"] = True
		# If still on defaults, align start/goal to the first user aisle
		if layout.get("start") == [2.0, 10.0] and layout.get("goal") == [18.0, 10.0]:
			a0, a1, a2, a3 = aisles[0]["rect"]
			horizontal = (a2 - a0) >= (a3 - a1)
			if horizontal:
				y_mid = 0.5 * (a1 + a3)
				layout["start"] = [_clamp_xy((a0 + 0.6, y_mid), bounds)[0], y_mid]
				layout["goal"] = [_clamp_xy((a2 - 0.6, y_mid), bounds)[0], y_mid]
			else:
				x_mid = 0.5 * (a0 + a2)
				layout["start"] = [x_mid, _clamp_xy((x_mid, a1 + 0.6), bounds)[1]]
				layout["goal"] = [x_mid, _clamp_xy((x_mid, a3 - 0.6), bounds)[1]]

	for m in _HUMAN_SEG_RE.finditer(prompt):
		x0 = float(m.group("x0")); y0 = float(m.group("y0"))
		sx, sy = _clamp_xy((x0, y0), bounds)
		if m.group("x1") and m.group("y1"):
			ex = float(m.group("x1")); ey = float(m.group("y1"))
		else:
			# no end provided: walk 6 m along y, staying inside bounds
			step = 6.0 if sy < bounds[1] * 0.5 else -6.0
			ex, ey = sx, sy + step
		ex, ey = _clamp_xy((ex, ey), bounds)
		rate = (nums or {}).get("human_rate_per_min", 1.0)
		group_size = 1
		if _has_any(prompt, KEYWORDS["busy_crossing"]):
			group_size = 3
		hcfg = {
			"path": "waypoints",
			"waypoints": [[sx, sy], [ex, ey]],
			"group_size": group_size,
			"rate_per_min": float(rate),
			"speed_mps": [0.8, 1.4],
			"motion_patterns": ["linear"],
			"raw_coords": True,
		}
		if _has_any(prompt, KEYWORDS["busy_crossing"]):
			_append_unique(hcfg["motion_patterns"], "close_pass")
		hazards.setdefault("human", []).append(hcfg)
		flags["human"] = True

	for m in _VEHICLE_SEG_RE.finditer(prompt):
		x0 = float(m.group("x0")); y0 = float(m.group("y0"))
		x1 = float(m.group("x1")); y1 = float(m.group("y1"))
		path = [_clamp_xy((x0, y0), bounds), _clamp_xy((x1, y1), bounds)]
		vehicle_entries.append({
			"id": f"Veh_coord_{len(vehicle_entries)+1:02d}",
			"type": m.group(1).lower(),
			"path": path,
			"speed_mps": 1.0,
			"ping_pong": True,
			"raw_coords": True,
		})
	if vehicle_entries:
		hazards.setdefault("vehicles", []).extend(vehicle_entries)
		flags["vehicle"] = True

	for m in _PATCH_SEG_RE.finditer(prompt):
		x0 = float(m.group("x0")); y0 = float(m.group("y0"))
		x1 = float(m.group("x1")); y1 = float(m.group("y1"))
		rect = _clamp_rect([x0, y0, x1, y1], bounds)
		traction_entries.append({
			"id": f"Traction_{len(traction_entries)+1:02d}",
			"zone": rect,
			"mu": 0.45,
			"type": "wet",
		})
	if traction_entries:
		hazards.setdefault("traction", []).extend(traction_entries)
		flags["traction"] = True

	for m in _STATIC_PT_RE.finditer(prompt):
		x = float(m.group("x")); y = float(m.group("y"))
		sx, sy = _clamp_xy((x, y), bounds)
		size = 0.6
		aabb = _clamp_rect([sx - 0.5 * size, sy - 0.5 * size, sx + 0.5 * size, sy + 0.5 * size], bounds)
		match_type = m.group(1).lower()
		obs_type = "standing_pallet"
		obs_extra: Dict[str, Any] = {}
		if "cart block" in match_type:
			obs_type = "cart_block"
			obs_extra = {"occlusion": True}
			flags["cart_block"] = True
		elif "column" in match_type:
			obs_type = "column"
			obs_extra = {"shape": "cylinder", "radius": 0.2, "height": 2.4}
		obs_list = layout.setdefault("static_obstacles", [])
		obs_list.append({
			"id": f"Static_{len(obs_list)+1:02d}",
			"type": obs_type,
			"aabb": aabb,
			"height": obs_extra.pop("height", 1.0),
			**obs_extra,
		})
		flags["static"] = True

	_ensure_map_bounds(layout, hazards)
	_ensure_start_goal_open(layout, tuple(layout["map_size_m"]))
	return flags

def _layout_list(scn: Dict[str, Any], key: str) -> List[Dict[str, Any]]:
	layout = scn.setdefault("layout", {})
	val = layout.get(key)
	if val is None:
		val = []
		layout[key] = val
		return val
	if not isinstance(val, list):
		raise ValueError(f"layout['{key}'] must be a list, got {type(val).__name__}")
	return val


def _geom_list(scn: Dict[str, Any], key: str) -> List[Dict[str, Any]]:
	layout = scn.setdefault("layout", {})
	geom = layout.setdefault("geometry", {})
	val = geom.get(key)
	if val is None:
		val = []
		geom[key] = val
		return val
	if not isinstance(val, list):
		raise ValueError(f"geometry['{key}'] must be a list, got {type(val).__name__}")
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

def _append_unique(lst: List[Any], val: Any) -> None:
	if val not in lst:
		lst.append(val)


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

	# friction (allow 0.x or 1.x up to ~1.5)
	m = re.search(r"(mu|friction)\s*([:=]?\s*)?(?P<val>1?\.[0-9]+)", text_l)
	if m:
		out["mu"] = float(m.group("val"))

	# duration seconds
	m = re.search(r"(duration|time limit|limit)\s*([:=]?\s*)?(?P<sec>\d{1,5})\s*s?", text_l)
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

def _apply_single_straight_layout(scn: Dict[str, Any]) -> None:
	layout = scn.setdefault("layout", {})
	layout["map_size_m"] = [20.0, 12.0]
	geom = layout.setdefault("geometry", {})
	geom.clear()
	aisle = {
		"id": "A_single",
		"type": "straight",
		"rect": [2.0, 4.5, 18.0, 5.9],
		"racking": False,
	}
	layout["aisles"] = [aisle]
	layout["lock_aisles"] = True
	layout["auto_aisles_from_paths"] = False
	layout["paths_define_aisles"] = False
	layout["start"] = [2.5, 5.2]
	layout["goal"] = [17.5, 5.2]
	# flank with racking bands
	geom["racking"] = [
		{"id": "Rack_above", "aabb": [2.0, 5.9, 18.0, 6.7], "height_m": 3.0},
		{"id": "Rack_below", "aabb": [2.0, 3.7, 18.0, 4.5], "height_m": 3.0},
	]
	layout["static_obstacles"] = []
	scn.setdefault("hazards", {})["human"] = []
	scn["hazards"]["vehicles"] = []

def _apply_parallel_pass_layout(scn: Dict[str, Any]) -> None:
	layout = scn.setdefault("layout", {})
	layout["map_size_m"] = [22.0, 16.0]
	geom = layout.setdefault("geometry", {})
	geom.clear()
	y0 = 6.0
	gap = 3.0
	width = 1.4
	def _aisle_rect(yc: float) -> List[float]:
		return [2.0, yc - width * 0.5, 20.0, yc + width * 0.5]
	aisles = [
		{"id": "A_par_01", "rect": _aisle_rect(y0), "type": "straight", "racking": False},
		{"id": "A_par_02", "rect": _aisle_rect(y0 + gap), "type": "straight", "racking": False},
	]
	layout["aisles"] = aisles
	layout["lock_aisles"] = True
	layout["auto_aisles_from_paths"] = False
	layout["paths_define_aisles"] = False
	layout["start"] = [3.0, y0]
	layout["goal"] = [19.0, y0]
	# racks between aisles
	geom["racking"] = [
		{"id": "Rack_mid", "aabb": [2.0, y0 + width * 0.5, 20.0, y0 + gap - width * 0.5], "height_m": 3.0},
		{"id": "Rack_upper", "aabb": [2.0, y0 + gap + width * 0.5, 20.0, y0 + gap + width * 1.5], "height_m": 3.0},
		{"id": "Rack_lower", "aabb": [2.0, y0 - width * 1.5, 20.0, y0 - width * 0.5], "height_m": 3.0},
	]
	# single worker in same aisle
	haz = scn.setdefault("hazards", {})
	haz["human"] = [{
		"path": "waypoints",
		"waypoints": [[5.0, y0], [17.0, y0]],
		"group_size": 1,
		"raw_coords": True,
	}]
	haz["vehicles"] = []

def _apply_narrow_cross_layout(scn: Dict[str, Any]) -> None:
	layout = scn.setdefault("layout", {})
	layout["map_size_m"] = [20.0, 12.0]
	geom = layout.setdefault("geometry", {})
	geom.clear()
	width = 1.0
	aisle = {"id": "A_narrow", "rect": [2.0, 5.5 - width * 0.5, 18.0, 5.5 + width * 0.5], "type": "straight", "racking": False, "high_bay": True}
	layout["aisles"] = [aisle]
	layout["lock_aisles"] = True
	layout["auto_aisles_from_paths"] = False
	layout["paths_define_aisles"] = False
	layout["start"] = [3.0, 5.5]
	layout["goal"] = [17.0, 5.5]
	geom["racking"] = [
		{"id": "Rack_high_top", "aabb": [2.0, 5.5 + width * 0.5, 18.0, 7.0], "height_m": 4.0, "type": "high_bay"},
		{"id": "Rack_high_bottom", "aabb": [2.0, 4.0, 18.0, 5.5 - width * 0.5], "height_m": 4.0, "type": "high_bay"},
	]
	# crossing worker perpendicular
	haz = scn.setdefault("hazards", {})
	haz["human"] = [{
		"path": "waypoints",
		"waypoints": [[10.0, 4.7], [10.0, 6.3]],
		"group_size": 1,
		"raw_coords": True,
	}]
	haz["vehicles"] = []

def _apply_wide_main_layout(scn: Dict[str, Any]) -> None:
	layout = scn.setdefault("layout", {})
	layout["map_size_m"] = [22.0, 14.0]
	geom = layout.setdefault("geometry", {})
	geom.clear()
	width = 2.0
	aisle = {"id": "A_wide", "rect": [2.0, 6.0 - width * 0.5, 20.0, 6.0 + width * 0.5], "type": "straight", "racking": False}
	layout["aisles"] = [aisle]
	layout["lock_aisles"] = True
	layout["auto_aisles_from_paths"] = False
	layout["paths_define_aisles"] = False
	layout["start"] = [3.0, 6.0]
	layout["goal"] = [19.0, 6.0]
	geom["racking"] = [
		{"id": "Rack_wide_top", "aabb": [2.0, 6.0 + width * 0.5, 20.0, 7.0], "height_m": 3.0},
		{"id": "Rack_wide_bottom", "aabb": [2.0, 5.0, 20.0, 6.0 - width * 0.5], "height_m": 3.0},
	]
	haz = scn.setdefault("hazards", {})
	haz["human"] = []
	haz["vehicles"] = []

def prompt_to_scenario(prompt: str, n_runs: int = 100) -> Dict[str, Any]:
	scn = new_scenario(n_runs)
	text = prompt.lower()
	nums = parse_numbers(prompt)
	t_case = _is_t_intersection_prompt(text)
	main_lr_case = _is_main_aisle_lr_prompt(text) and not t_case
	simple_aisle_case = _is_simple_forklift_aisle_prompt(text) and not main_lr_case and not t_case
	parallel_aisle_case = _wants_parallel_aisles(text) and not simple_aisle_case and not main_lr_case and not t_case
	single_straight_case = _is_single_straight_prompt(text)
	parallel_pass_case = _is_parallel_pass_prompt(text)
	narrow_cross_case = _is_narrow_cross_prompt(text)
	wide_main_case = _is_wide_main_prompt(text)
	if t_case:
		_apply_t_intersection_layout(scn)
	elif main_lr_case:
		_apply_main_aisle_lr_layout(scn)
	elif simple_aisle_case:
		_apply_simple_forklift_aisle_layout(scn)
	elif parallel_aisle_case:
		_apply_parallel_aisles_layout(scn)
	elif single_straight_case:
		_apply_single_straight_layout(scn)
	elif parallel_pass_case:
		_apply_parallel_pass_layout(scn)
	elif narrow_cross_case:
		_apply_narrow_cross_layout(scn)
	elif wide_main_case:
		_apply_wide_main_layout(scn)
	coord_flags = _apply_coordinate_overrides(scn, prompt, nums)
	if ("aisle" in text and "no aisle" not in text and "without aisle" not in text and
	    not simple_aisle_case and not main_lr_case and not t_case and not parallel_aisle_case and not coord_flags.get("aisle")):
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

	def _add_default_wet_patch_if_needed(center: tuple[float, float], half: float = 1.5) -> None:
		patches = scn.setdefault("hazards", {}).setdefault("traction", [])
		if patches:
			return
		cx, cy = center
		mu = nums.get("mu", 0.45)
		patches.append({
			"id": f"Traction_{len(patches)+1:02d}",
			"type": "wet",
			"zone": [cx - half, cy - half, cx + half, cy + half],
			"mu": float(mu),
		})
		scn.setdefault("taxonomy", {})["traction"] = True

	# --- Traction (wet patch) ---
	if _has_any(text, KEYWORDS["traction"]) and not coord_flags.get("traction") and not _has_any(text, KEYWORDS["blind_corner"]):
		# prefer geometry-derived locations; fallback only if nothing else was added later
		geom_center = None
		geom = scn.get("layout", {}).get("geometry", {}) or {}
		if scn.get("layout", {}).get("main_lr_case"):
			cx = float(geom.get("main_aisle_lane_x", 10.0))
			cy = float(geom.get("main_aisle_lane_y", 9.0))
			geom_center = (cx, cy)
		elif scn.get("layout", {}).get("t_case"):
			juncs = scn.get("layout", {}).get("junctions") or []
			if juncs and juncs[0].get("rect"):
				x0, y0, x1, y1 = map(float, juncs[0]["rect"])
				geom_center = ((x0 + x1) * 0.5, (y0 + y1) * 0.5)
		else:
			aisles = scn.get("layout", {}).get("aisles") or []
			if len(aisles) >= 2 and aisles[0].get("rect") and aisles[1].get("rect"):
				a0 = aisles[0]["rect"]; a1 = aisles[1]["rect"]
				cx = 0.25 * (a0[0] + a0[2] + a1[0] + a1[2])
				cy = 0.25 * (a0[1] + a0[3] + a1[1] + a1[3])
				geom_center = (cx, cy)
		if geom_center is not None:
			_add_default_wet_patch_if_needed(geom_center, half=1.5)
		else:
			_add_default_wet_patch_if_needed((13.0, 3.5), half=1.0)  # fallback only

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
	if (_has_any(text, KEYWORDS["human"]) and not scn["layout"].get("main_lr_case")
	    and not scn["layout"].get("t_case") and not coord_flags.get("human") and not _has_any(text, KEYWORDS["blind_corner"])):
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
		if scn.get("layout", {}).get("main_lr_case") and not coord_flags.get("vehicle"):
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
		_append_unique(cfg.setdefault("motion_patterns", []), "reduced_visibility")
		scn["taxonomy"]["human_behavior"] = True

	if _has_any(text, KEYWORDS["fast_walker"]):
		cfg = _ensure_human_entry(scn, 0)
		cfg["speed_mps"] = [1.4, 2.2]
		_add_behavior_tag(cfg, "fast_walk")
		_append_unique(cfg.setdefault("motion_patterns", []), "zig_zag")
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
			_append_unique(cfg.setdefault("motion_patterns", []), "close_pass")
			_append_unique(cfg.setdefault("interaction_modes", []), "mutual_yield")

	# --- Blind corner / occluded crosswalks ---
	if _has_any(text, KEYWORDS["blind_corner"]) and not scn["layout"].get("t_case"):
		scn["taxonomy"]["occlusion"] = True
		scn["taxonomy"]["visibility"] = True  # night/low light
		so_root = _so_root()
		so_inj = so_root.setdefault("injectors", {})
		blk = so_inj.setdefault("lidar_blackout", {})
		blk.setdefault("p", 0.90)
		layout = scn["layout"]
		# Clear geometry to avoid leftover racks/endcaps
		geom = layout.setdefault("geometry", {})
		geom.clear()
		# Anchor the symmetric "+" in positive coordinates while honoring exact relative dimensions.
		base_x = 10.0
		base_y = 10.0
		layout["map_size_m"] = [max(layout.get("map_size_m", [20.0, 20.0])[0], base_x + 10.0),
		                       max(layout.get("map_size_m", [20.0, 20.0])[1], base_y + 10.0)]
		# Aisles: width 3 m (±1.5) and length 16 m (±8)
		h_rect = [base_x - 8.0, base_y - 1.5, base_x + 8.0, base_y + 1.5]
		v_rect = [base_x - 1.5, base_y - 8.0, base_x + 1.5, base_y + 8.0]
		layout["aisles"] = [
			{"id": "A_blind_main", "rect": h_rect, "type": "straight", "racking": False},
			{"id": "A_blind_cross", "rect": v_rect, "type": "straight", "racking": False},
		]
		layout["lock_aisles"] = True
		layout["auto_aisles_from_paths"] = False
		layout["paths_define_aisles"] = False
		geom["lux_level"] = "night"
		geom["blind_corner_lane_x"] = base_x
		# Single occluder block in NE quadrant: footprint [0,2]x[0,2] relative to the intersection center
		geom["racking"] = [
			{"id": "BlindCorner_Block", "type": "high_bay", "aabb": [base_x, base_y, base_x + 2.0, base_y + 2.0], "height_m": 3.0},
		]
		geom["endcaps"] = []
		# Wet patch square 4x4 at the intersection
		if _has_any(text, KEYWORDS["traction"]) and not coord_flags.get("traction"):
			patches = scn.setdefault("hazards", {}).setdefault("traction", [])
			patches.clear()
			patches.append({
				"id": "Traction_01",
				"type": "wet",
				"zone": [base_x - 2.0, base_y - 2.0, base_x + 2.0, base_y + 2.0],
				"mu": float(nums.get("mu", 0.3 if "night" in text else 0.35)),
			})
		# Robot start/goal along horizontal aisle
		layout["start"] = [base_x - 6.0, base_y]
		layout["goal"] = [base_x + 6.0, base_y]
		# Human crossing vertically from south to north (hidden behind the rack at start)
		humans = scn.setdefault("hazards", {}).setdefault("human", [])
		if not humans:
			humans.append({
				"path": "waypoints",
				"waypoints": [[base_x, base_y - 6.0], [base_x, base_y + 6.0]],
				"group_size": 1,
				"speed_mps": [0.7, 1.1],
				"motion_patterns": ["hesitation", "emerge_from_occlusion"],
				"interaction_modes": ["mutual_yield", "robot_yield"],
				"start_delay_s": 0.0,
			})
		agents = scn.get("agents") or []
		if agents:
			agents[0]["max_speed_mps"] = max(float(agents[0].get("max_speed_mps", 1.2)), 1.5)
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
	    not scn["layout"].get("simple_forklift_aisle") and
	    not coord_flags.get("vehicle")):
		scn["taxonomy"]["multi_actor"] = True
		geom = scn["layout"].setdefault("geometry", {})
		lane_x = geom.get("forklift_aisle_lane_x")
		lane_y = geom.get("forklift_aisle_lane_y")
		if lane_x is None:
			lane_x = geom.get("blind_corner_lane_x")
		layout_obj = scn.get("layout", {})
		aisles_existing = layout_obj.get("aisles") or []
		if lane_x is None and lane_y is None and aisles_existing:
			rect = aisles_existing[0].get("rect", [0.0, 0.0, 0.0, 0.0])
			x_span = abs(rect[2] - rect[0])
			y_span = abs(rect[3] - rect[1])
			if x_span >= y_span:
				lane_y = 0.5 * (rect[1] + rect[3])
			else:
				lane_x = 0.5 * (rect[0] + rect[2])
		if lane_x is None and lane_y is None:
			start = layout_obj.get("start", [2.0, 10.0])
			goal = layout_obj.get("goal", [18.0, 10.0])
			if abs(goal[0] - start[0]) >= abs(goal[1] - start[1]):
				lane_y = start[1]
			else:
				lane_x = start[0]
		if lane_x is None:
			lane_x = 9.0
		veh_list = scn.setdefault('hazards', {}).setdefault('vehicles', [])
		veh_id = f"Forklift_{len(veh_list)+1:02d}"
		if lane_y is not None or geom.get("forklift_aisle_lane_x") is not None:
			veh_id = f"ForkliftAisle_{len(veh_list)+1:02d}"
		map_size = layout_obj.get("map_size_m", [20.0, 20.0])
		Lx = float(map_size[0]) if isinstance(map_size, (list, tuple)) else 20.0
		Ly = float(map_size[1]) if isinstance(map_size, (list, tuple)) else 20.0
		path_default = [[2.0, lane_y if lane_y is not None else 2.0], [Lx - 2.0, lane_y if lane_y is not None else Ly - 2.0]] if lane_y is not None else [[lane_x, 2.0], [lane_x, Ly - 2.0]]
		f_default = {
			"id": veh_id,
			"type": "forklift",
			"path": path_default,
			"speed_mps": 1.2,
			"warning_lights": True,
			"reversing_bias": True,
		}
		if lane_y is not None or geom.get("forklift_aisle_lane_x") is not None:
			f_default["reversing_bias"] = False
		_ensure_vehicle(scn, f_default)

	if _has_any(text, KEYWORDS["forklift_reverse"]) and not coord_flags.get("vehicle"):
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
	if _has_any(text, KEYWORDS["pallet_jack"]) and not coord_flags.get("vehicle"):
		scn["taxonomy"]["multi_actor"] = True
		_ensure_vehicle(scn, {
			"id": f"PalletJack_{len(scn.setdefault('hazards', {}).setdefault('vehicles', []))+1:02d}",
			"type": "pallet_jack",
			"path": [[8.5, 2.0], [12.0, 2.0]],
			"speed_mps": 0.9,
		})

	if _has_any(text, KEYWORDS["cart_block"]):
		if coord_flags.get("cart_block"):
			pass  # already handled via coordinates
		else:
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
		if not scn["hazards"].get("human"):
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
				if cfg.get("raw_coords") or "cross_x" in cfg:
					continue
				cfg["cross_x"] = lane_x

	if simple_aisle_case:
		main_cross = geom.get("main_cross_x", 10.0)
		for cfg in scn["hazards"].get("human", []):
			if cfg.get("raw_coords") or "cross_x" in cfg:
				continue
			cfg["cross_x"] = main_cross
	if main_lr_case:
		cross = geom.get("main_aisle_lane_x", 10.0)
		for cfg in scn["hazards"].get("human", []):
			if cfg.get("raw_coords") or "cross_x" in cfg:
				continue
			cfg["cross_x"] = cross

	# If the prompt didn't request vehicles and none were provided via coordinates, strip template forklifts
	wants_any_vehicle = (
		coord_flags.get("vehicle")
		or _has_any(text, KEYWORDS["forklift"])
		or _has_any(text, KEYWORDS["traffic"])
		or _has_any(text, KEYWORDS["busy_aisle"])
		or _has_any(text, KEYWORDS["congestion"])
	)
	if not wants_any_vehicle and (layout.get("main_lr_case") or layout.get("t_case")):
		scn.setdefault("hazards", {})["vehicles"] = []

	if layout.get("t_case"):
		branch_x = geom.get("branch_lane_x", 10.0)
		for cfg in scn["hazards"].get("human", []):
			if cfg.get("raw_coords") or "cross_x" in cfg:
				continue
			cfg["cross_x"] = branch_x

	_ensure_map_bounds(layout, scn.get("hazards", {}))
	_ensure_start_goal_open(layout, tuple(layout["map_size_m"]))
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
		if veh.get("raw_coords"):
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

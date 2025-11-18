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
	"child_actor": ["child-height", "child height", "kid", "youth"],
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

	# convenience handle for overrides
	def _so_root() -> Dict[str, Any]:
		return scn.setdefault("site_overrides", {})

	# --- Visibility ---
	if _has_any(text, KEYWORDS["visibility"]):
		scn["sensors"]["lighting_dim"] = True
		scn["taxonomy"]["visibility"] = True

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
	if _has_any(text, KEYWORDS["human"]):
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

	# --- Traffic → clutter density ---
	if _has_any(text, KEYWORDS["traffic"]):
		scn["hazards"]["clutter"]["aisle_density"] = 0.25

	if _has_any(text, KEYWORDS["busy_aisle"]) or _has_any(text, KEYWORDS["congestion"]):
		cl = scn["hazards"]["clutter"]
		cl["aisle_density"] = max(cl.get("aisle_density", 0.0), 0.45)
		cl["overhang_prob"] = max(cl.get("overhang_prob", 0.0), 0.25)
		scn["taxonomy"]["multi_actor"] = True

	# --- Overhang probability ---
	if _has_any(text, KEYWORDS["overhang"]):
		scn["hazards"]["clutter"]["overhang_prob"] = 0.3

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

		# Make robot slightly less cautious; keep fallen as persistent obstacle; increase chaos
		so_root.setdefault("brake", {}).update({
			"stop_dist_m": 0.8,
			"slow_dist_m": 1.8,
			"stop_hold_s": 0.6,
		})
		so_root.setdefault("human", {}).update({
			"replan_on_fall_p": 0.20,   # less replanning → more conflict
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
		# more likely to slip and less likely for planner to replan nicely
		h.setdefault("slip_prob", 0.95)
		h.setdefault("replan_on_fall_p", 0.25)

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
	if _has_any(text, KEYWORDS["blind_corner"]):
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
	if _has_any(text, KEYWORDS["forklift"]):
		scn["taxonomy"]["multi_actor"] = True
		scn["hazards"]["clutter"]["overhang_prob"] = max(
			scn["hazards"]["clutter"].get("overhang_prob", 0.0),
			0.4,
		)
		lane_x = scn["layout"].setdefault("geometry", {}).get("blind_corner_lane_x", 9.0)
		_ensure_vehicle(scn, {
			"id": f"Forklift_{len(scn.setdefault('hazards', {}).setdefault('vehicles', []))+1:02d}",
			"type": "forklift",
			"path": [[lane_x, 2.0], [lane_x, 18.0]],
			"speed_mps": 1.2,
			"warning_lights": True,
			"reversing_bias": True,
		})

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
		scn["hazards"]["clutter"]["aisle_density"] = max(
			scn["hazards"]["clutter"].get("aisle_density", 0.0),
			0.35,
		)
		_ensure_vehicle(scn, {
			"id": f"PalletJack_{len(scn.setdefault('hazards', {}).setdefault('vehicles', []))+1:02d}",
			"type": "pallet_jack",
			"path": [[8.5, 2.0], [12.0, 2.0]],
			"speed_mps": 0.9,
		})

	if _has_any(text, KEYWORDS["cart_block"]):
		_add_static_obstacle(scn, {
			"type": "cart_block",
			"shape": "box",
			"aabb": [9.2, 7.6, 10.8, 8.8],
			"height": 1.2,
			"occlusion": True,
		})
		scn["hazards"]["clutter"]["aisle_density"] = max(
			scn["hazards"]["clutter"].get("aisle_density", 0.0),
			0.5,
		)
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

	_align_vehicle_paths(scn)

	# Align human crossings with blind-corner geometry so they don't spawn inside walls
	if scn["taxonomy"].get("occlusion"):
		layout = scn.get("layout", {})
		geom = layout.get("geometry", {}) or {}
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

	return scn

def _align_vehicle_paths(scn: Dict[str, Any]) -> None:
	layout = scn.get("layout", {})
	Lx = float(layout.get("map_size_m", [20.0, 20.0])[0])
	geom = layout.get("geometry", {})
	endcaps = geom.get("endcaps", [])
	lane_hint = geom.get("blind_corner_lane_x")
	vehicles = scn.setdefault("hazards", {}).setdefault("vehicles", [])
	for veh in vehicles:
		if veh.get("type") != "forklift":
			continue
		if lane_hint is not None:
			path = veh.get("path") or []
			if len(path) >= 2:
				veh["path"] = [[lane_hint, path[0][1]], [lane_hint, path[1][1]]]
				continue
		path = veh.get("path") or []
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

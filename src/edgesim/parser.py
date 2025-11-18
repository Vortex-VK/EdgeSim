from __future__ import annotations
import re
from typing import Any, Dict

from .schema import new_scenario

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

	# Clothing / reflectivity
	"reflective": ["reflective vest", "hi-vis", "hi vis", "high visibility", "high-visibility"],
	"dark_clothing": ["dark clothing", "dark hoodie", "black jacket", "no vest"],

	# Sensor / lidar issues
	"sensor_fault": ["lidar fault", "sensor glitch", "ghost obstacle",
	                 "false obstacle", "false positive", "sensor issue"],
	"degraded_mode": ["degraded mode", "reduced lidar", "low sensor", "cpu overload",
	                  "failsafe mode", "degraded sensing"],
}


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

	# --- Human crossing (generic, single stream) ---
	if _has_any(text, KEYWORDS["human"]):
		rate = nums.get("human_rate_per_min", 2.0)  # every 30s default
		scn["hazards"]["human"].append({
			"path": "line",
			"rate_per_min": float(rate),
			"speed_mps": [0.8, 1.4],
		})
		scn["taxonomy"]["human_behavior"] = True

	# --- Traffic → clutter density ---
	if _has_any(text, KEYWORDS["traffic"]):
		scn["hazards"]["clutter"]["aisle_density"] = 0.25

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
		scn.setdefault("hazards", {}).setdefault("human", [])
		scn["hazards"]["human"] = [
			{"path": "line", "rate_per_min": nums.get("human_rate_per_min", 2.0), "speed_mps": [0.8, 1.4]},
			{"path": "line", "rate_per_min": nums.get("human_rate_per_min", 2.0), "speed_mps": [0.8, 1.4]},
		]
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
		so_inj.setdefault("thrown_object",  {}).update({"p": 0.60})

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

	# --- Busy crossing / groups / platoons ---
	if _has_any(text, KEYWORDS["busy_crossing"]):
		scn["taxonomy"]["human_behavior"] = True
		scn["taxonomy"]["multi_actor"] = True
		# Nudge chaos via more frequent dynamic events
		so_root = _so_root()
		so_inj = so_root.setdefault("injectors", {})
		fo = so_inj.setdefault("falling_object", {})
		fo.setdefault("p", 0.70)
		to = so_inj.setdefault("thrown_object", {})
		to.setdefault("p", 0.60)

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

	# --- Forklift-related chaos (load overhangs, drops) ---
	if _has_any(text, KEYWORDS["forklift"]):
		scn["taxonomy"]["multi_actor"] = True
		# more overhang-style clutter
		scn["hazards"]["clutter"]["overhang_prob"] = max(
			scn["hazards"]["clutter"].get("overhang_prob", 0.0),
			0.4,
		)
		so_root = _so_root()
		so_inj = so_root.setdefault("injectors", {})
		fo = so_inj.setdefault("falling_object", {})
		fo.setdefault("p", 0.75)
		to = so_inj.setdefault("thrown_object", {})
		to.setdefault("p", 0.50)

	# --- Pallet jack / hand-cart => denser clutter & more nonhuman contacts ---
	if _has_any(text, KEYWORDS["pallet_jack"]):
		scn["taxonomy"]["multi_actor"] = True
		scn["hazards"]["clutter"]["aisle_density"] = max(
			scn["hazards"]["clutter"].get("aisle_density", 0.0),
			0.35,
		)

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

	return scn

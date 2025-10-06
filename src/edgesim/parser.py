from __future__ import annotations
import re
from typing import Any, Dict

from .schema import new_scenario

# --- Keyword map for simple rule-based parsing (V0) ---
KEYWORDS = {
    "traction":   ["wet", "slippery", "spill", "oil"],
    "visibility": ["night", "dim", "dark", "low light", "reflective"],
    "human":      ["human", "picker", "worker", "crossing"],
    "traffic":    ["high pallet traffic", "rush", "busy", "heavy traffic"],
    "overhang":   ["overhang", "irregular load"],
    # New intents
    "two_crosswalks": ["two crosswalks", "double crosswalk", "staggered human crossings", "two crossers"],
    "narrow":     ["narrow aisle", "narrow corridor", "tight aisle"],
    "falling":    ["falling object", "pallet drops", "drop near", "shatter", "spillage"],
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
        # Slightly push robot to tighter margins via a taxonomy flag only
        scn["taxonomy"]["narrow"] = True

    # --- Falling object emphasis (nudges injector probabilities via site_overrides) ---
    if _has_any(text, KEYWORDS["falling"]):
        so_root = scn.setdefault("site_overrides", {})
        so_inj = so_root.setdefault("injectors", {})
        # Bump falling object probability; leave others unchanged unless scenario says otherwise
        f = so_inj.setdefault("falling_object", {})
        f.setdefault("p", 0.80)

    # --- Two-crosswalks special case (edge-focused, self-contained) ---
    if _has_any(text, KEYWORDS["two_crosswalks"]):
        # Ensure we have exactly two crosser configs
        scn.setdefault("hazards", {}).setdefault("human", [])
        scn["hazards"]["human"] = [
            {"path": "line", "rate_per_min": nums.get("human_rate_per_min", 2.0), "speed_mps": [0.8, 1.4]},
            {"path": "line", "rate_per_min": nums.get("human_rate_per_min", 2.0), "speed_mps": [0.8, 1.4]},
        ]
        scn["taxonomy"]["human_behavior"] = True

        # Force overlap and faster crossings; add slip dynamics
        scn["hazards"]["human"][0].update({
            "cross_x": 10.0,
            "duration_s_running": 2.2,
            "p_slip": 0.60,
            "trigger_mu": 0.9, "trigger_sigma": 0.2
        })
        scn["hazards"]["human"][1].update({
            "cross_x": 10.8,
            "duration_s_running": 2.2,
            "p_slip": 0.60,
            "trigger_mu": 0.9, "trigger_sigma": 0.2,
            "start_delay_s": 0.8
        })

        # Scenario-local overrides that keep it “edge” even if the text didn’t say “wet”
        so_root = scn.setdefault("site_overrides", {})
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

    # --- Duration override (if present in text) ---
    if "duration_s" in nums:
        scn["runtime"]["duration_s"] = int(nums["duration_s"])  # type: ignore

    return scn

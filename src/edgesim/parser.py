from __future__ import annotations
import copy
import math
import re
from typing import Any, Dict, List

from .schema import new_scenario, default_vehicle_roster

# --- Keyword map for simple rule-based parsing (V0+) ---
KEYWORDS = {
    "traction":   ["wet", "slippery", "spill", "spillage", "oil"],
    "visibility": ["night", "dim", "dark", "low light", "reflective"],
    "human":      ["human", "picker", "worker", "pedestrian", "crossing"],
    "traffic":    ["high pallet traffic", "rush", "heavy traffic"],
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
    "high_shelf": ["high shelf", "high-bay", "high bay", "high rack", "high-shelf occlusion", "tall rack", "tall racks", "tall shelf"],

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
    "sensor_fault": ["lidar fault", "sensor fault", "sensor glitch", "ghost obstacle",
                     "false obstacle", "false positive", "sensor issue",
                     "lidar blackout", "sensor blackout", "blackout"],
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

_PASSAGE_LABEL_RE = r"(?P<label>(?:main\s+)?(?:aisle|corridor|crosswalk|lane|passage))"
_PASSAGE_SEG_RE = re.compile(rf"{_PASSAGE_LABEL_RE}[^\n]*?(?:from|between)\s+{_SEG_RE}", re.IGNORECASE)
_HUMAN_SEG_RE = re.compile(
    rf"(?:human(?:\s+crossing)?|pedestrian|worker|person|staff)[^\n]*?(?:from|starting at|start at)\s+"
    rf"(?P<x0>{_NUM_RE})\s*,\s*(?P<y0>{_NUM_RE})"
    rf"(?:[^\n]*?(?:to|->|-)\s*(?P<x1>{_NUM_RE})\s*,\s*(?P<y1>{_NUM_RE}))?",
    re.IGNORECASE,
)
_VEHICLE_SEG_RE = re.compile(rf"(forklift|vehicle|cart|tugger|pallet jack)[^\n]*?from\s+{_SEG_RE}", re.IGNORECASE)
_PATCH_SEG_RE = re.compile(rf"(wet\s+patch|spill|oil|traction)[^\n]*?(?:from|covering|zone)\s+{_SEG_RE}", re.IGNORECASE)
_STATIC_PT_RE = re.compile(rf"(pallet|obstacle|box|cart block|cart|column|cone)[^\n]*?(?:at|center(?:ed)? at)\s+{_PT_RE}", re.IGNORECASE)
_WALL_SEG_RE = re.compile(rf"(?P<wlabel>high wall|tall wall|wall|barrier)[^\n]*?from\s+{_SEG_RE}", re.IGNORECASE)
_RACK_SEG_RE = re.compile(rf"(rack|high rack|high shelf|high-bay|high bay)[^\n]*?from\s+{_SEG_RE}", re.IGNORECASE)

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
    has_lr = ("left to right" in text_l) or ("left-right" in text_l) or ("leftright" in text_l)
    return has_main and has_lr

def _is_t_intersection_prompt(text: str) -> bool:
    return False  # disabled: require coordinates/templates for T cases

def _wants_parallel_aisles(text: str) -> bool:
    return False  # parallel aisle presets disabled; use explicit coordinates instead

def _is_no_human(text: str) -> bool:
    text_l = text.lower()
    return ("no human" in text_l) or ("no workers" in text_l) or ("no pedestrians" in text_l)

def _is_no_vehicle(text: str) -> bool:
    text_l = text.lower()
    return ("no vehicle" in text_l) or ("no vehicles" in text_l) or ("no forklift" in text_l) or ("no forklifts" in text_l)

def _is_single_straight_prompt(text: str) -> bool:
    text_l = text.lower()
    return ("single straight aisle" in text_l) or ("one aisle" in text_l)

def _is_parallel_pass_prompt(text: str) -> bool:
    return False  # disabled; require explicit coordinates for parallel aisles

def _mentions_crossing(text: str) -> bool:
    text_l = text.lower()
    if any(term in text_l for term in _HUMAN_MODE_TERMS["move_across"]):
        return True
    return ("crosswalk" in text_l) or ("crossing" in text_l) or ("cross aisle" in text_l) or ("across" in text_l)

def _wants_crosswalk_geom(text: str) -> bool:
    text_l = text.lower()
    return ("crosswalk" in text_l) or ("cross aisle" in text_l) or ("cross-aisle" in text_l) or ("hidden crosswalk" in text_l)

def _is_narrow_cross_prompt(text: str) -> bool:
    return False  # disabled; require explicit coordinates

def _is_wide_main_prompt(text: str) -> bool:
    text_l = text.lower()
    return (("wide aisle" in text_l or "main corridor" in text_l or "wide main aisle" in text_l) and "no human" in text_l and "no vehicle" in text_l)

_WALLED_AISLE_TERMS = [
    "aisle with walls",
    "walled aisle",
    "walled corridor",
    "walled path",
    "walls along the aisle",
]
_BETWEEN_RACKS_TERMS = [
    "between racks",
    "between the racks",
    "between two racks",
]

def _wants_walled_aisles(text: str) -> bool:
    text_l = text.lower()
    return any(term in text_l for term in _WALLED_AISLE_TERMS)

def _wants_between_racks(text: str) -> bool:
    text_l = text.lower()
    return any(term in text_l for term in _BETWEEN_RACKS_TERMS)

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

def _rect_center(rect: List[float] | tuple[float, float, float, float]) -> tuple[float, float]:
    x0, y0, x1, y1 = rect
    return (0.5 * (float(x0) + float(x1)), 0.5 * (float(y0) + float(y1)))

def _shrink_rect_width(rect: List[float], scale: float = 0.7, min_span: float = 0.6) -> List[float]:
    x0, y0, x1, y1 = map(float, rect)
    cx, cy = _rect_center(rect)
    w = abs(x1 - x0)
    h = abs(y1 - y0)
    if w >= h:
        # Horizontal aisle: shrink height only
        h_new = max(min_span, h * scale)
        return [x0, cy - 0.5 * h_new, x1, cy + 0.5 * h_new]
    else:
        w_new = max(min_span, w * scale)
        return [cx - 0.5 * w_new, y0, cx + 0.5 * w_new, y1]

def _apply_narrowing(layout: Dict[str, Any], scale: float = 0.65,
                     aisle_indices: List[int] | None = None, include_junctions: bool = False) -> None:
    """Scale aisle widths in-place. If aisle_indices is None, apply to all aisles."""
    if layout.get("_raw_coord_aisles"):
        return
    aisles = layout.get("aisles") or []
    bounds = tuple(map(float, layout.get("map_size_m", [20.0, 20.0])))
    if aisle_indices is None:
        target_idxs = list(range(len(aisles)))
    else:
        target_idxs = [i for i in aisle_indices if 0 <= i < len(aisles)]
    for idx in target_idxs:
        aisle = aisles[idx]
        rect = aisle.get("rect")
        if not rect or len(rect) != 4:
            continue
        new_rect = _shrink_rect_width(list(map(float, rect)), scale=scale)
        aisle["rect"] = _clamp_rect(new_rect, bounds)
        width_axis = abs(aisle["rect"][3] - aisle["rect"][1]) if abs(aisle["rect"][2] - aisle["rect"][0]) >= abs(aisle["rect"][3] - aisle["rect"][1]) else abs(aisle["rect"][2] - aisle["rect"][0])
        aisle["width_hint_m"] = float(width_axis)
    if include_junctions:
        junctions = layout.get("junctions") or []
        for junc in junctions:
            rect = junc.get("rect") or junc.get("zone")
            if not rect or len(rect) != 4:
                continue
            junc["rect"] = _shrink_rect_width(list(map(float, rect)), scale=scale)

def _split_rect(rect: List[float], cut: List[float], min_span: float = 0.05) -> List[List[float]]:
    rx0, ry0, rx1, ry1 = rect
    cx0, cy0, cx1, cy1 = cut
    # no overlap -> keep as-is
    if cx1 <= rx0 or cx0 >= rx1 or cy1 <= ry0 or cy0 >= ry1:
        return [rect]
    pieces: List[List[float]] = []
    # prefer splitting along the dominant overlap axis
    if (cx0 > rx0):
        pieces.append([rx0, ry0, cx0, ry1])
    if (cx1 < rx1):
        pieces.append([cx1, ry0, rx1, ry1])
    if not pieces:
        if cy0 > ry0:
            pieces.append([rx0, ry0, rx1, cy0])
        if cy1 < ry1:
            pieces.append([rx0, cy1, rx1, ry1])
    return [p for p in pieces if (p[2] - p[0]) > min_span and (p[3] - p[1]) > min_span]

def _carve_racks_for_cut(geom: Dict[str, Any], cut_rect: List[float], pad: float = 0.05) -> None:
    racks = list(geom.get("racking") or [])
    if not racks:
        return
    cx0, cy0, cx1, cy1 = cut_rect
    padded_cut = [cx0 - pad, cy0 - pad, cx1 + pad, cy1 + pad]
    new_racks: List[Dict[str, Any]] = []
    for rack in racks:
        aabb = rack.get("aabb")
        if not aabb or len(aabb) != 4:
            new_racks.append(rack)
            continue
        pieces = _split_rect(list(map(float, aabb)), padded_cut)
        if not pieces:
            continue
        if len(pieces) == 1 and pieces[0] == list(map(float, aabb)):
            new_racks.append(rack)
            continue
        for idx, piece in enumerate(pieces):
            nr = dict(rack)
            nr["aabb"] = [float(piece[0]), float(piece[1]), float(piece[2]), float(piece[3])]
            if len(pieces) > 1:
                nr["id"] = f"{rack.get('id', 'Rack')}_seg{idx+1}"
            new_racks.append(nr)
    geom["racking"] = new_racks

def _add_racking_bands_from_aisle(geom: Dict[str, Any], rect: List[float], rack_cfg: Dict[str, Any],
                                  bounds: tuple[float, float], rid_prefix: str) -> None:
    x0, y0, x1, y1 = map(float, rect)
    Lx, Ly = bounds
    horizontal = abs(x1 - x0) >= abs(y1 - y0)
    gap = float(rack_cfg.get("gap_m", 0.15))
    depth = float(rack_cfg.get("depth_m", 0.9))
    height = float(rack_cfg.get("height_m", 3.0))
    r_type = rack_cfg.get("type", "rack")
    reflective = bool(rack_cfg.get("reflective", r_type == "high_bay"))
    bands: List[List[float]] = []
    if horizontal:
        bands.append([x0, max(0.0, y0 - gap - depth), x1, max(0.0, y0 - gap)])
        bands.append([x0, min(Ly, y1 + gap), x1, min(Ly, y1 + gap + depth)])
    else:
        bands.append([max(0.0, x0 - gap - depth), y0, max(0.0, x0 - gap), y1])
        bands.append([min(Lx, x1 + gap), y0, min(Lx, x1 + gap + depth), y1])
    racks = geom.setdefault("racking", [])
    for idx, band in enumerate(bands):
        aabb = _clamp_rect(band, bounds)
        if (aabb[2] - aabb[0]) < 0.05 or (aabb[3] - aabb[1]) < 0.05:
            continue
        racks.append({
            "id": f"{rid_prefix}_{idx+1:02d}",
            "aabb": aabb,
            "height_m": height,
            "type": r_type,
            "reflective": reflective,
        })

def _add_walls_for_aisle(layout: Dict[str, Any], rect: List[float], aid: str, thickness: float = 0.15, height: float = 2.0,
                         color: List[float] | tuple[float, float, float, float] | None = None) -> None:
    walls = layout.setdefault("walls", [])
    Lx, Ly = map(float, layout.get("map_size_m", [20.0, 20.0]))
    horizontal = abs(rect[2] - rect[0]) >= abs(rect[3] - rect[1])
    if horizontal:
        w_s = _clamp_rect([rect[0], rect[1] - thickness, rect[2], rect[1]], (Lx, Ly))
        w_n = _clamp_rect([rect[0], rect[3], rect[2], rect[3] + thickness], (Lx, Ly))
        entry_s = {"id": f"Wall_{aid}_S", "aabb": w_s, "height_m": height}
        entry_n = {"id": f"Wall_{aid}_N", "aabb": w_n, "height_m": height}
        if color is not None:
            entry_s["color"] = list(color)
            entry_n["color"] = list(color)
        walls.extend([entry_s, entry_n])
    else:
        w_w = _clamp_rect([rect[0] - thickness, rect[1], rect[0], rect[3]], (Lx, Ly))
        w_e = _clamp_rect([rect[2], rect[1], rect[2] + thickness, rect[3]], (Lx, Ly))
        entry_w = {"id": f"Wall_{aid}_W", "aabb": w_w, "height_m": height}
        entry_e = {"id": f"Wall_{aid}_E", "aabb": w_e, "height_m": height}
        if color is not None:
            entry_w["color"] = list(color)
            entry_e["color"] = list(color)
        walls.extend([entry_w, entry_e])

def _carve_wall_gaps(layout: Dict[str, Any], pad: float = 0.05) -> None:
    walls = layout.get("walls") or []
    if not walls:
        return
    cuts: List[List[float]] = []
    for aisle in layout.get("aisles", []) or []:
        rect = aisle.get("rect")
        if rect and len(rect) == 4:
            cuts.append(list(map(float, rect)))
    for junc in layout.get("junctions", []) or []:
        rect = junc.get("rect") or junc.get("zone")
        if rect and len(rect) == 4:
            cuts.append(list(map(float, rect)))
    if not cuts:
        return
    new_walls: List[Dict[str, Any]] = []
    for wall in walls:
        aabb = wall.get("aabb")
        if not aabb or len(aabb) != 4:
            new_walls.append(wall)
            continue
        pieces = [list(map(float, aabb))]
        for cut in cuts:
            padded = [cut[0] - pad, cut[1] - pad, cut[2] + pad, cut[3] + pad]
            next_pieces: List[List[float]] = []
            for piece in pieces:
                split = _split_rect(piece, padded, min_span=0.05)
                if split:
                    next_pieces.extend(split)
                else:
                    next_pieces.append(piece)
            pieces = next_pieces
        if len(pieces) == 1 and pieces[0] == list(map(float, aabb)):
            new_walls.append(wall)
            continue
        for idx, piece in enumerate(pieces):
            nw = dict(wall)
            nw["aabb"] = [float(piece[0]), float(piece[1]), float(piece[2]), float(piece[3])]
            if len(pieces) > 1:
                nw["id"] = f"{wall.get('id', 'Wall')}_seg{idx+1}"
            new_walls.append(nw)
    layout["walls"] = new_walls

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
        "speed_mps": [0.9, 1.5],
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
    # x: 024, y: 022
    # - Main aisle along y=12 (1.4 m), with racks on both sides
    # - Forklift aisle along y=15 (1.5 m, high-bay racks)
    # - Dock/staging band y29 with a 2.2 m dock lane at y=6.5
    # - Northsouth spine at x=12 tying dock  main  forklift, padded at junctions
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
        {"id": "A_main", "name": "main_aisle", "rect": main_rect, "type": "straight", "pad": [0.05, 0.15, 0.05, 0.15], "racking": False},
        {"id": "A_forklift", "name": "forklift_aisle", "rect": forklift_rect, "type": "straight", "pad": [0.05, 0.15, 0.05, 0.15], "racking": False, "high_bay": True},
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
    # explicit racking bands flanking the two upper aisles
    Lx, Ly = map(float, layout["map_size_m"])
    _add_racking_bands_from_aisle(geom, main_rect, {"gap_m": 0.2, "depth_m": 0.95, "height_m": 3.6, "type": "rack"}, (Lx, Ly), "Rack_Main")
    _add_racking_bands_from_aisle(geom, forklift_rect, {"gap_m": 0.2, "depth_m": 0.95, "height_m": 4.2, "type": "high_bay", "reflective": True}, (Lx, Ly), "Rack_Forklift")

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
            "speed_mps": 1.35,
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
        {"id": "A_par_01", "name": "aisle_1", "rect": aisle_1, "type": "straight", "racking": False},
        {"id": "A_par_02", "name": "aisle_2", "rect": aisle_2, "type": "straight", "racking": False},
    ])
    geom = layout.setdefault("geometry", {})
    geom.setdefault("racking", [])
    LxLy = (float(Lx), float(Ly))
    _add_racking_bands_from_aisle(geom, aisle_1, rack_cfg, LxLy, "Rack_A1")
    _add_racking_bands_from_aisle(geom, aisle_2, rack_cfg, LxLy, "Rack_A2")

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
    for wall in layout.get("walls", []) or []:
        if wall.get("aabb"):
            slots.append(("wall", wall, "aabb"))
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
    for wall in layout.get("walls", []) or []:
        if wall.get("aabb"):
            solid.append(list(map(float, wall["aabb"])))
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

def _retreat_goal_from_transitions(layout: Dict[str, Any], margin_extra: float = 0.5) -> None:
    """Pull goal back from transition/doorway zones along the primary aisle."""
    ref = _primary_aisle_ref(layout)
    if not ref:
        return
    goal = layout.get("goal")
    if not (isinstance(goal, (list, tuple)) and len(goal) == 2):
        return
    tzs = layout.get("transition_zones") or []
    if not tzs:
        return
    gx, gy = float(goal[0]), float(goal[1])
    width = max(0.6, float(ref.get("width", 1.0)))
    margin = max(0.8, width * 0.5 + margin_extra)
    for tz in tzs:
        zone = tz.get("zone")
        if not zone or len(zone) != 4:
            continue
        x0, y0, x1, y1 = map(float, zone)
        if ref["horizontal"]:
            if not (y0 <= ref["center"][1] <= y1):
                continue
            if gx > x0 - margin:
                gx = max(ref["rect"][0] + margin, min(x0 - margin, ref["rect"][2] - margin))
        else:
            if not (x0 <= ref["center"][0] <= x1):
                continue
            if gy > y0 - margin:
                gy = max(ref["rect"][1] + margin, min(y0 - margin, ref["rect"][3] - margin))
    layout["goal"] = [gx, gy]

def _normalize_high_racks(layout: Dict[str, Any], min_height: float = 5.0) -> None:
    """Ensure only one rack type when high racks are requested; unify to high_bay with taller height and dedupe."""
    geom = layout.get("geometry", {}) or {}
    racks = geom.get("racking")
    if not isinstance(racks, list):
        return
    new_racks: List[Dict[str, Any]] = []
    seen: List[tuple[float, float, float, float]] = []
    for rack in racks:
        if not isinstance(rack, dict):
            continue
        nr = dict(rack)
        nr["type"] = "high_bay"
        nr["reflective"] = True
        nr["height_m"] = max(min_height, float(nr.get("height_m", min_height)))
        aabb = nr.get("aabb")
        if isinstance(aabb, (list, tuple)) and len(aabb) == 4:
            key = tuple(round(float(v), 3) for v in aabb)
            if key in seen:
                continue
            seen.append(key)
        new_racks.append(nr)
    geom["racking"] = new_racks

def _align_start_goal_to_rect(rect: List[float], bounds: tuple[float, float],
                              segment: tuple[tuple[float, float], tuple[float, float]] | None = None,
                              preserve_endpoints: bool = False) -> tuple[List[float], List[float]]:
    x0, y0, x1, y1 = map(float, rect)
    cx, cy = _rect_center(rect)
    if preserve_endpoints and segment:
        (sx, sy), (ex, ey) = segment
        return _clamp_xy((sx, sy), bounds), _clamp_xy((ex, ey), bounds)
    if segment:
        (sx, sy), (ex, ey) = segment
    else:
        sx, sy, ex, ey = x0, y0, x1, y1
    horizontal = abs(ex - sx) >= abs(ey - sy)
    span = abs(ex - sx) if horizontal else abs(ey - sy)
    span = max(span, abs(x1 - x0) if horizontal else abs(y1 - y0), 0.5)
    inset = 0.35 if segment else min(1.2, max(0.6, 0.2 * span))
    if horizontal:
        left = min(sx, ex, x0, x1)
        right = max(sx, ex, x0, x1)
        start_x = max(x0, left + inset)
        goal_x = min(x1, right - inset)
        if start_x >= goal_x - 0.05:
            mid = 0.5 * (left + right)
            start_x = max(x0, mid - 0.25)
            goal_x = min(x1, mid + 0.25)
        start = [start_x, cy]
        goal = [goal_x, cy]
    else:
        low = min(sy, ey, y0, y1)
        high = max(sy, ey, y0, y1)
        start_y = max(y0, low + inset)
        goal_y = min(y1, high - inset)
        if start_y >= goal_y - 0.05:
            mid = 0.5 * (low + high)
            start_y = max(y0, mid - 0.25)
            goal_y = min(y1, mid + 0.25)
        start = [cx, start_y]
        goal = [cx, goal_y]
    return _clamp_xy(tuple(start), bounds), _clamp_xy(tuple(goal), bounds)

def _collect_prompt_coords(prompt: str) -> tuple[float, float]:
    max_x = 0.0
    max_y = 0.0

    def _upd(val: float, axis: str) -> None:
        nonlocal max_x, max_y
        if axis == "x":
            max_x = max(max_x, val)
        else:
            max_y = max(max_y, val)

    for rg in (_PASSAGE_SEG_RE, _HUMAN_SEG_RE, _VEHICLE_SEG_RE, _PATCH_SEG_RE, _RACK_SEG_RE):
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
    flags = {"aisle": False, "human": False, "vehicle": False, "traction": False, "static": False, "cart_block": False, "wall": False, "rack": False}
    rack_entries: List[Dict[str, Any]] = []

    aisles: List[Dict[str, Any]] = []
    vehicle_entries: List[Dict[str, Any]] = []
    traction_entries: List[Dict[str, Any]] = []
    aisle_segments: List[tuple[tuple[float, float], tuple[float, float]]] = []
    wall_entries: List[Dict[str, Any]] = []

    def _extra_segments(span: str) -> List[List[float]]:
        extras: List[List[float]] = []
        for em in re.finditer(rf"\band\s+(?P<x0>{_NUM_RE})\s*,\s*(?P<y0>{_NUM_RE})\s*(?:to|->|-)\s*(?P<x1>{_NUM_RE})\s*,\s*(?P<y1>{_NUM_RE})", span, re.IGNORECASE):
            extras.append([float(em.group("x0")), float(em.group("y0")), float(em.group("x1")), float(em.group("y1"))])
        return extras

    def _extra_points(span: str) -> List[List[float]]:
        extras: List[List[float]] = []
        for em in re.finditer(rf"\band\s+(?P<x>{_NUM_RE})\s*,\s*(?P<y>{_NUM_RE})", span, re.IGNORECASE):
            extras.append([float(em.group("x")), float(em.group("y"))])
        return extras

    stop_re = re.compile(r"\b(wall|rack|aisle|cross|crosswalk|junction|human|pedestrian|worker|forklift|pallet|cart|tugger|wet\s+patch|spill|doorway|threshold|box|cone|column)\b", re.IGNORECASE)

    def _tail_until_stop(start_idx: int) -> str:
        tail = prompt[start_idx:]
        m = stop_re.search(tail)
        cut = m.start() if m else len(tail)
        return tail[:cut]

    for m in _PASSAGE_SEG_RE.finditer(prompt):
        snip = (m.group(0) or "").lower()
        if ("wall from" in snip or "barrier from" in snip) and ("with wall" not in snip and "with walls" not in snip):
            continue
        label = (m.group("label") or "aisle").strip()
        label_l = label.lower()
        x0 = float(m.group("x0")); y0 = float(m.group("y0"))
        x1 = float(m.group("x1")); y1 = float(m.group("y1"))
        start = tuple(_clamp_xy((x0, y0), bounds))
        end = tuple(_clamp_xy((x1, y1), bounds))
        width = 1.4 if "main" in label_l else 1.2
        if "narrow" in snip:
            width = min(width, 1.0)
        aisle_type = "cross" if "crosswalk" in label_l else "straight"
        rect = _clamp_rect(_axis_aligned_rect_from_centerline(start, end, width), bounds)
        aisle_segments.append((start, end))
        aisles.append({
            "id": f"A_coord_{len(aisles)+1:02d}",
            "name": label.replace(" ", "_"),
            "rect": rect,
            "type": aisle_type,
            "racking": False,
            "pad": [0.05, 0.05, 0.05, 0.05],
        })
    if aisles:
        # If only cross aisles were provided, synthesize a simple through-aisle so crossings have something to span.
        straight_idxs = [i for i, a in enumerate(aisles) if a.get("type") != "cross"]
        if not straight_idxs and aisles and aisles[0].get("rect"):
            rect = list(map(float, aisles[0]["rect"]))
            horizontal = abs(rect[2] - rect[0]) >= abs(rect[3] - rect[1])
            cx, cy = _rect_center(rect)
            width_main = 1.2
            if horizontal:
                main_rect = _axis_aligned_rect_from_centerline((cx, 1.0), (cx, bounds[1] - 1.0), width_main)
            else:
                main_rect = _axis_aligned_rect_from_centerline((1.0, cy), (bounds[0] - 1.0, cy), width_main)
            # keep a matching segment for alignment
            aisle_segments.append(((main_rect[0], 0.5 * (main_rect[1] + main_rect[3])), (main_rect[2], 0.5 * (main_rect[1] + main_rect[3]))) if abs(main_rect[2] - main_rect[0]) >= abs(main_rect[3] - main_rect[1]) else ((0.5 * (main_rect[0] + main_rect[2]), main_rect[1]), (0.5 * (main_rect[0] + main_rect[2]), main_rect[3])))
            aisles.append({
                "id": "A_auto_main_from_cross",
                "name": "main_from_cross",
                "rect": _clamp_rect(main_rect, bounds),
                "type": "straight",
                "racking": False,
                "pad": [0.05, 0.05, 0.05, 0.05],
            })
            straight_idxs.append(len(aisles) - 1)
        layout["aisles"] = aisles
        layout["lock_aisles"] = True
        layout["_raw_coord_aisles"] = True
        layout["auto_aisles_from_paths"] = False
        layout["paths_define_aisles"] = False
        flags["aisle"] = True
        # Clear template geometry that might conflict with user-aligned aisles
        geom = layout.setdefault("geometry", {})
        for key in ("racking", "endcaps", "open_storage", "blind_corners"):
            geom[key] = []
        # Align start/goal to the first straight aisle if available, otherwise the first aisle.
        align_idx = straight_idxs[0] if straight_idxs else 0
        first_rect = aisles[align_idx]["rect"]
        seg = aisle_segments[align_idx] if align_idx < len(aisle_segments) else None
        # avoid preserving crosswalk endpoints when aligning to synthesized main
        if aisles[align_idx].get("type") == "cross":
            seg = None
        layout["start"], layout["goal"] = _align_start_goal_to_rect(first_rect, bounds, seg, preserve_endpoints=bool(seg))

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
            "speed_mps": [0.9, 1.5],
            "motion_patterns": ["linear"],
            "raw_coords": True,
        }
        if _has_any(prompt, KEYWORDS["busy_crossing"]):
            _append_unique(hcfg["motion_patterns"], "close_pass")
        hazards.setdefault("human", []).append(hcfg)
        flags["human"] = True

    prompt_lower = prompt.lower()
    for m in _VEHICLE_SEG_RE.finditer(prompt):
        text_span = (m.group(0) or "").lower()
        vtype = (m.group(1) or "").lower()
        # Avoid mis-parsing "cart blocking ..." followed by unrelated "from ..." (e.g., wet patch)
        if vtype == "cart" and ("block" in text_span or "blocking" in text_span):
            continue
        tail = _tail_until_stop(m.end())
        seg_list = [[float(m.group("x0")), float(m.group("y0")), float(m.group("x1")), float(m.group("y1"))]]
        seg_list.extend(_extra_segments(tail))
        for (x0, y0, x1, y1) in seg_list:
            path = [_clamp_xy((x0, y0), bounds), _clamp_xy((x1, y1), bounds)]
            entry: Dict[str, Any] = {
                "id": f"Veh_coord_{len(vehicle_entries)+1:02d}",
                "type": vtype,
                "path": path,
                "speed_mps": 1.0,
                "ping_pong": True,
                "raw_coords": True,
            }
            # Respect reversing intent even for raw-coordinate forklifts; look in local window too
            window = prompt_lower[max(0, m.start() - 12):min(len(prompt_lower), m.end() + 12)]
            if entry["type"] == "forklift" and ("revers" in text_span or "revers" in window):
                entry["reversing_mode"] = True
                entry["reversing_bias"] = True
                entry["warning_lights"] = True
                entry["alarm"] = entry.get("alarm") or "backup_beeper"
                entry["rear_occlusion_deg"] = entry.get("rear_occlusion_deg", 70.0)
            vehicle_entries.append(entry)
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
        match_type = m.group(1).lower()
        tail = _tail_until_stop(m.end())
        point_list = [[float(m.group("x")), float(m.group("y"))]]
        point_list.extend(_extra_points(tail))
        for (x, y) in point_list:
            sx, sy = _clamp_xy((x, y), bounds)
            size = 0.6
            aabb = _clamp_rect([sx - 0.5 * size, sy - 0.5 * size, sx + 0.5 * size, sy + 0.5 * size], bounds)
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

    for m in _WALL_SEG_RE.finditer(prompt):
        label = (m.group("wlabel") or "wall").lower()
        tail = _tail_until_stop(m.end())
        seg_list = [[float(m.group("x0")), float(m.group("y0")), float(m.group("x1")), float(m.group("y1"))]]
        seg_list.extend(_extra_segments(tail))
        for (x0, y0, x1, y1) in seg_list:
            rect = _clamp_rect(_axis_aligned_rect_from_centerline(_clamp_xy((x0, y0), bounds), _clamp_xy((x1, y1), bounds), 0.3), bounds)
            height = 2.2
            if "high" in label or "tall" in label:
                height = 3.6
            wall_entries.append({
                "id": f"Wall_{len(wall_entries)+1:02d}",
                "aabb": rect,
                "height_m": height,
            })

    if wall_entries:
        layout.setdefault("walls", []).extend(wall_entries)
        flags["wall"] = True

    for m in _RACK_SEG_RE.finditer(prompt):
        label = (m.group(1) or "").lower()
        tail = _tail_until_stop(m.end())
        seg_list = [[float(m.group("x0")), float(m.group("y0")), float(m.group("x1")), float(m.group("y1"))]]
        seg_list.extend(_extra_segments(tail))
        for (x0, y0, x1, y1) in seg_list:
            rect = _clamp_rect(_axis_aligned_rect_from_centerline(_clamp_xy((x0, y0), bounds), _clamp_xy((x1, y1), bounds), 0.9), bounds)
            r_type = "high_bay" if "high" in label else "rack"
            height = 5.0 if "high" in label else 3.0
            rack_entries.append({
                "id": f"Rack_{len(rack_entries)+1:02d}",
                "aabb": rect,
                "height_m": height,
                "type": r_type,
                "reflective": True if r_type == "high_bay" else False,
            })

    if rack_entries:
        # Deduplicate by AABB, keeping the last definition (so lower racks replace stray duplicates)
        dedup: Dict[tuple[float, float, float, float], Dict[str, Any]] = {}
        for r in rack_entries:
            aabb = r.get("aabb")
            if not aabb or len(aabb) != 4:
                continue
            key = tuple(round(float(v), 3) for v in aabb)
            dedup[key] = dict(r)
        final_racks: List[Dict[str, Any]] = []
        for idx, r in enumerate(dedup.values()):
            r = dict(r)
            r["id"] = f"Rack_{idx+1:02d}"
            final_racks.append(r)
        layout.setdefault("geometry", {}).setdefault("racking", []).extend(final_racks)
        flags["rack"] = True

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
            "speed_mps": [0.9, 1.5],
            "motion_patterns": ["linear", "start_stop", "hesitation"],
        })
    cfg = humans[idx]
    cfg.setdefault("motion_patterns", ["linear"])
    cfg.setdefault("interaction_modes", ["mutual_yield", "human_yield", "robot_yield"])
    cfg.setdefault("visibility_awareness", "normal")
    cfg["start_delay_s"] = float(cfg.get("start_delay_s", 0.0))
    cfg["start_delay_s_min"] = 0.0
    cfg["start_delay_s_max"] = 0.0
    return cfg

_HUMAN_MODE_TERMS = {
    "stand_near": ["stand near", "standing near", "near the aisle", "near aisle"],
    "stand_in": ["standing in the aisle", "stand in the aisle", "in the aisle"],
    "move_across": ["moving across", "move across", "crossing the aisle", "across the aisle", "crossing"],
    "move_along": ["moving along", "move along", "walking along", "walk along", "along the aisle", "down the aisle"],
}

def _primary_aisle_ref(layout: Dict[str, Any]) -> Dict[str, Any] | None:
    aisles = layout.get("aisles") or []
    for a in aisles:
        rect = a.get("rect")
        if rect and len(rect) == 4:
            rect_f = list(map(float, rect))
            horizontal = abs(rect_f[2] - rect_f[0]) >= abs(rect_f[3] - rect_f[1])
            width = abs(rect_f[3] - rect_f[1]) if horizontal else abs(rect_f[2] - rect_f[0])
            length = abs(rect_f[2] - rect_f[0]) if horizontal else abs(rect_f[3] - rect_f[1])
            center = _rect_center(rect_f)
            return {"rect": rect_f, "horizontal": horizontal, "width": width, "length": length, "center": center}
    return None

def _human_motion_intent(text: str) -> str:
    text_l = text.lower()
    if any(term in text_l for term in _HUMAN_MODE_TERMS["move_across"]):
        return "move_across"
    if any(term in text_l for term in _HUMAN_MODE_TERMS["move_along"]):
        return "move_along"
    if any(term in text_l for term in _HUMAN_MODE_TERMS["stand_in"]):
        return "stand_in"
    if any(term in text_l for term in _HUMAN_MODE_TERMS["stand_near"]):
        return "stand_near"
    return "stand_near"

def _waypoints_for_human_mode(mode: str, layout: Dict[str, Any]) -> List[List[float]]:
    ref = _primary_aisle_ref(layout)
    Lx, Ly = map(float, layout.get("map_size_m", [20.0, 20.0]))
    bounds = (Lx, Ly)
    if ref is None:
        cx, cy = _clamp_xy((0.5 * Lx, 0.5 * Ly), bounds)
        return [[cx, cy], [cx, cy]]

    def _perp_anchor() -> tuple[float | None, List[float] | None]:
        # Prefer a perpendicular aisle/crosswalk when present to keep human crossings centered on geometry.
        aisles = layout.get("aisles") or []
        for aisle in aisles:
            rect = aisle.get("rect")
            if not rect or len(rect) != 4:
                continue
            rect_f = list(map(float, rect))
            a_horizontal = abs(rect_f[2] - rect_f[0]) >= abs(rect_f[3] - rect_f[1])
            if a_horizontal == ref["horizontal"]:
                continue
            cx_p, cy_p = _rect_center(rect_f)
            anchor_val = cx_p if ref["horizontal"] else cy_p
            return anchor_val, rect_f
        # fall back to geometry hints when no perpendicular aisle is present
        geom = layout.get("geometry", {}) or {}
        keys = ("main_cross_x", "branch_lane_x", "forklift_aisle_lane_x", "blind_corner_lane_x") if ref["horizontal"] else ("main_cross_y", "branch_lane_y", "forklift_aisle_lane_y")
        for k in keys:
            if k in geom and geom[k] is not None:
                try:
                    return float(geom[k]), None
                except Exception:
                    continue
        return None, None

    x0, y0, x1, y1 = ref["rect"]
    cx, cy = ref["center"]
    width = max(0.1, float(ref["width"]))
    length = max(0.5, float(ref["length"]))
    start = layout.get("start")
    goal = layout.get("goal")
    robot_start = None
    robot_goal = None
    try:
        if isinstance(start, (list, tuple)) and isinstance(goal, (list, tuple)) and len(start) == 2 and len(goal) == 2:
            robot_start = (float(start[0]), float(start[1]))
            robot_goal = (float(goal[0]), float(goal[1]))
    except Exception:
        robot_start = robot_goal = None
    if robot_start and robot_goal:
        dx_r = robot_goal[0] - robot_start[0]
        dy_r = robot_goal[1] - robot_start[1]
        robot_len = math.hypot(dx_r, dy_r)
        if robot_len < 1e-6:
            robot_len = None
            robot_dir = None
        else:
            robot_dir = (dx_r / robot_len, dy_r / robot_len)
    else:
        robot_len = None
        robot_dir = None
    # Keep humans away from walls so spawn clearance checks succeed.
    margin = max(0.15, min(width * 0.5 - 0.05, max(0.4, width * 0.45)))
    anchor_hint, cross_rect = _perp_anchor()
    if ref["horizontal"]:
        anchor_major = cx
        if anchor_hint is not None:
            anchor_major = max(x0 + margin, min(x1 - margin, anchor_hint))
        if cross_rect:
            anchor_major = max(cross_rect[0] + 0.15, min(cross_rect[2] - 0.15, anchor_major))
        elif robot_start and robot_goal:
            anchor_major = max(x0 + 0.4, min(x1 - 0.4, robot_start[0] + 0.35 * (robot_goal[0] - robot_start[0])))
        near_side = min(y1 - margin, max(y0 + margin, cy - 0.3))
        center_line = cy
        if mode == "stand_near":
            pt = _clamp_xy((anchor_major, near_side), bounds)
            return [pt, pt]
        if mode == "stand_in":
            pt = _clamp_xy((anchor_major, center_line), bounds)
            return [pt, pt]
        if mode == "move_along":
            # Follow the robot path along the aisle, offset slightly so spawn passes clearance.
            dir_vec = robot_dir if robot_dir else (1.0, 0.0)
            if robot_len is None:
                robot_len = abs(x1 - x0)
            offset = min(0.6, 0.08 * robot_len)
            half_span = max(0.5, 0.5 * robot_len - offset)
            mid = (anchor_major, center_line)
            start_pt = (mid[0] - dir_vec[0] * half_span, mid[1] - dir_vec[1] * half_span)
            end_pt = (mid[0] + dir_vec[0] * half_span, mid[1] + dir_vec[1] * half_span)
            return [_clamp_xy(start_pt, bounds), _clamp_xy(end_pt, bounds)]
        # move_across
        dir_vec = robot_dir if robot_dir else (1.0, 0.0)
        if robot_len is None:
            robot_len = abs(x1 - x0)
        perp = (-dir_vec[1], dir_vec[0])
        mid = (anchor_major, center_line)
        if cross_rect:
            span_cross = max(0.2, float(cross_rect[3] - cross_rect[1]))
            clear = max(0.08, min(0.25, 0.06 * span_cross))
            start_pt = (mid[0], max(cross_rect[1] + clear, cross_rect[1]))
            end_pt = (mid[0], min(cross_rect[3] - clear, cross_rect[3]))
            span_actual = end_pt[1] - start_pt[1]
            if span_actual < 0.8:
                span = max(0.8, robot_len)
                start_pt = (mid[0], mid[1] - 0.5 * span)
                end_pt = (mid[0], mid[1] + 0.5 * span)
        else:
            span = robot_len
            start_pt = (mid[0] - perp[0] * 0.5 * span, mid[1] - perp[1] * 0.5 * span)
            end_pt = (mid[0] + perp[0] * 0.5 * span, mid[1] + perp[1] * 0.5 * span)
        return [_clamp_xy(start_pt, bounds), _clamp_xy(end_pt, bounds)]
    else:
        anchor_major = cy
        if anchor_hint is not None:
            anchor_major = max(y0 + margin, min(y1 - margin, anchor_hint))
        if cross_rect:
            anchor_major = max(cross_rect[1] + 0.15, min(cross_rect[3] - 0.15, anchor_major))
        elif robot_start and robot_goal:
            anchor_major = max(y0 + 0.4, min(y1 - 0.4, robot_start[1] + 0.35 * (robot_goal[1] - robot_start[1])))
        near_side = min(x1 - margin, max(x0 + margin, cx - 0.3))
        center_line = cx
        if mode == "stand_near":
            pt = _clamp_xy((near_side, anchor_major), bounds)
            return [pt, pt]
        if mode == "stand_in":
            pt = _clamp_xy((center_line, anchor_major), bounds)
            return [pt, pt]
        if mode == "move_along":
            dir_vec = robot_dir if robot_dir else (0.0, 1.0)
            if robot_len is None:
                robot_len = abs(y1 - y0)
            offset = min(0.6, 0.08 * robot_len)
            half_span = max(0.5, 0.5 * robot_len - offset)
            mid = (center_line, anchor_major)
            start_pt = (mid[0] - dir_vec[0] * half_span, mid[1] - dir_vec[1] * half_span)
            end_pt = (mid[0] + dir_vec[0] * half_span, mid[1] + dir_vec[1] * half_span)
            return [_clamp_xy(start_pt, bounds), _clamp_xy(end_pt, bounds)]
        dir_vec = robot_dir if robot_dir else (0.0, 1.0)
        if robot_len is None:
            robot_len = abs(y1 - y0)
        perp = (-dir_vec[1], dir_vec[0])
        mid = (center_line, anchor_major)
        if cross_rect:
            span_cross = max(0.2, float(cross_rect[2] - cross_rect[0]))
            clear = max(0.08, min(0.25, 0.06 * span_cross))
            start_pt = (max(cross_rect[0] + clear, cross_rect[0]), mid[1])
            end_pt = (min(cross_rect[2] - clear, cross_rect[2]), mid[1])
            span_actual = end_pt[0] - start_pt[0]
            if span_actual < 0.8:
                span = max(0.8, robot_len)
                start_pt = (mid[0] - 0.5 * span, mid[1])
                end_pt = (mid[0] + 0.5 * span, mid[1])
        else:
            span = robot_len
            start_pt = (mid[0] - perp[0] * 0.5 * span, mid[1] - perp[1] * 0.5 * span)
            end_pt = (mid[0] + perp[0] * 0.5 * span, mid[1] + perp[1] * 0.5 * span)
        return [_clamp_xy(start_pt, bounds), _clamp_xy(end_pt, bounds)]

def _apply_human_motion_intent(scn: Dict[str, Any], prompt: str, rate_hint: float | None = None) -> None:
    # Skip intent override for behaviors that already carry their own spacing/paths (busy crossings, fast walkers, child actors, multi-crosswalks)
    if (_has_any(prompt, KEYWORDS["busy_crossing"])
        or _has_any(prompt, KEYWORDS["fast_walker"])
        or _has_any(prompt, KEYWORDS["child_actor"])
        or _has_any(prompt, KEYWORDS.get("two_crosswalks", []))):
        return
    mode = _human_motion_intent(prompt)
    humans = scn.get("hazards", {}).get("human") or []
    if len(humans) != 1:
        return
    if any(h.get("raw_coords") for h in humans):
        return
    pts = _waypoints_for_human_mode(mode, scn.get("layout", {}))
    if not pts:
        return
    cfg = humans[0]
    cfg["path"] = "waypoints"
    cfg["waypoints"] = pts
    cfg["raw_coords"] = True
    cfg["group_size"] = 1
    cfg["spawn_clearance_m"] = 0.1
    cfg["start_delay_s"] = 0.0
    cfg["start_delay_s_min"] = 0.0
    cfg["start_delay_s_max"] = 0.0
    cfg.pop("cross_x", None)
    if mode in ("stand_near", "stand_in"):
        cfg["motion_patterns"] = ["stationary"]
        cfg["speed_mps"] = [0.0, 0.0]
        cfg["rate_per_min"] = 0.0
    else:
        cfg["motion_patterns"] = ["linear", "start_stop"]
        cfg["rate_per_min"] = float(rate_hint if rate_hint is not None else cfg.get("rate_per_min", 2.0))
        cfg["loop"] = False


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

def _ensure_base_layout(scn: Dict[str, Any]) -> None:
	layout = scn.setdefault("layout", {})
	aisles = layout.get("aisles") or []
	if aisles:
		return
	layout.setdefault("map_size_m", [20.0, 20.0])
	# Simple straight aisle centered vertically, moderate width
	aisle_rect = [3.0, 9.4, 17.0, 10.6]
	aisles.append({"id": "Aisle_Default", "rect": aisle_rect, "type": "straight", "racking": False})
	layout["aisles"] = aisles
	layout.setdefault("start", [4.0, 10.0])
	layout.setdefault("goal", [16.0, 10.0])
	geom = layout.setdefault("geometry", {})
	geom["main_cross_x"] = 10.0


def _add_transition_zone(scn: Dict[str, Any], zone: Dict[str, Any]) -> None:
	tz = _layout_list(scn, "transition_zones")
	if "id" not in zone:
		zone["id"] = f"DoorExtra_{len(tz)+1:02d}"
	tz.append(zone)

def _ensure_default_human_paths(scn: Dict[str, Any]) -> None:
	layout = scn.get("layout", {}) or {}
	Lx, Ly = map(float, layout.get("map_size_m", [20.0, 20.0]))
	border = 0.6
	for cfg in scn.get("hazards", {}).get("human", []) or []:
		if cfg.get("raw_coords"):
			continue
		if cfg.get("waypoints"):
			continue
		if cfg.get("path") not in (None, "line", "waypoints"):
			continue
		cross_x = float(cfg.get("cross_x", 0.5 * Lx))
		cross_x = max(border, min(Lx - border, cross_x))
		start_y = border if (cfg.get("start_side", "south").lower() != "north") else (Ly - border)
		end_y = (Ly - border) if start_y <= border + 1e-3 else border
		cfg["waypoints"] = [[cross_x, start_y], [cross_x, end_y]]
		cfg["path"] = "waypoints"

def _assemble_passages_from_intent(scn: Dict[str, Any], intent: Dict[str, Any]) -> None:
    layout = scn.setdefault("layout", {})
    if layout.get("aisles"):
        return
    map_size = layout.get("map_size_m", [20.0, 20.0])
    try:
        Lx, Ly = float(map_size[0]), float(map_size[1])
    except Exception:
        Lx, Ly = 20.0, 20.0
    wants_any_passage = (
        intent.get("wants_long_main_passage")
        or intent.get("wants_parallel_passages")
        or intent.get("wants_passage_generic")
        or intent.get("mentions_forklift")
        or intent.get("mentions_humans")
        or intent.get("traffic_heavy")
    )
    if not wants_any_passage:
        _ensure_base_layout(scn)
        return
    aisles = layout.setdefault("aisles", [])
    width = 1.0 if intent.get("wants_narrow_passage") else 1.2
    start_margin = max(0.6, 0.08 * Lx)
    end_margin = max(0.6, 0.08 * Lx)
    x0 = start_margin
    x1 = max(x0 + 4.0, Lx - end_margin)
    if intent.get("wants_parallel_passages"):
        gap = max(2.5, 0.18 * Ly)
        mid = 0.5 * Ly
        centers = [max(width, mid - 0.5 * gap), min(Ly - width, mid + 0.5 * gap)]
        for idx, cy in enumerate(centers):
            rect = _axis_aligned_rect_from_centerline((x0, cy), (x1, cy), width)
            aisles.append({"id": f"A_auto_{idx+1:02d}", "name": f"aisle_{idx+1}", "rect": rect, "type": "straight", "racking": False})
    else:
        cy = 0.5 * Ly
        if intent.get("wants_long_main_passage"):
            x0 = max(0.6, 0.05 * Lx)
            x1 = max(x0 + 6.0, Lx - 0.6)
        rect = _axis_aligned_rect_from_centerline((x0, cy), (x1, cy), width)
        aisles.append({"id": "A_auto_main", "name": "aisle_main", "rect": rect, "type": "straight", "racking": False})
    layout["aisles"] = aisles
    layout["lock_aisles"] = True
    layout["auto_aisles_from_paths"] = False
    layout["paths_define_aisles"] = False
    bounds = (float(Lx), float(Ly))
    layout["start"], layout["goal"] = _align_start_goal_to_rect(aisles[0]["rect"], bounds)

def _decorate_passages(scn: Dict[str, Any], intent: Dict[str, Any], prompt: str = "") -> None:
    layout = scn.setdefault("layout", {})
    if layout.get("_raw_coord_aisles"):
        return
    aisles = layout.get("aisles") or []
    if not aisles:
        return
    map_size = layout.get("map_size_m", [20.0, 20.0])
    try:
        Lx, Ly = float(map_size[0]), float(map_size[1])
    except Exception:
        Lx, Ly = 20.0, 20.0
    geom = layout.setdefault("geometry", {})
    if intent.get("endcap_rack"):
        rect = aisles[0].get("rect")
        if rect and len(rect) == 4:
            x0, y0, x1, y1 = map(float, rect)
            horizontal = abs(x1 - x0) >= abs(y1 - y0)
            endcaps = geom.setdefault("endcaps", [])
            if horizontal:
                endcaps.append({"id": f"Endcap_{len(endcaps)+1:02d}", "aabb": _clamp_rect([x0 - 0.4, y0, x0 + 0.4, y1], (Lx, Ly)), "height_m": 2.4})
                endcaps.append({"id": f"Endcap_{len(endcaps)+1:02d}", "aabb": _clamp_rect([x1 - 0.4, y0, x1 + 0.4, y1], (Lx, Ly)), "height_m": 2.4})
            else:
                endcaps.append({"id": f"Endcap_{len(endcaps)+1:02d}", "aabb": _clamp_rect([x0, y0 - 0.4, x1, y0 + 0.4], (Lx, Ly)), "height_m": 2.4})
                endcaps.append({"id": f"Endcap_{len(endcaps)+1:02d}", "aabb": _clamp_rect([x0, y1 - 0.4, x1, y1 + 0.4], (Lx, Ly)), "height_m": 2.4})
    wall_color = (0.55, 0.35, 0.2, 1.0) if intent.get("wants_between_racks") else None
    narrow_already = bool(layout.get("narrow_aisle_hint") or layout.get("narrow_cross_case"))
    if intent.get("wants_walls"):
        for idx, aisle in enumerate(aisles):
            rect = aisle.get("rect")
            if not rect or len(rect) != 4:
                continue
            if _has_any(prompt, KEYWORDS["narrow"]) and not narrow_already:
                x0, y0, x1, y1 = map(float, rect)
                horizontal = abs(x1 - x0) >= abs(y1 - y0)
                if horizontal:
                    cy = 0.5 * (y0 + y1)
                    rect = [x0, cy - 0.5, x1, cy + 0.5]
                else:
                    cx = 0.5 * (x0 + x1)
                    rect = [cx - 0.5, y0, cx + 0.5, y1]
                aisle["rect"] = rect
            _add_walls_for_aisle(layout, list(map(float, rect)), aisle.get("id") or f"Aisle_{idx:02d}", color=wall_color)

def _add_blind_corner_occluder(layout: Dict[str, Any]) -> tuple[str, float] | None:
    aisles = layout.get("aisles") or []
    if not aisles:
        return None
    rect = aisles[0].get("rect")
    if not rect or len(rect) != 4:
        return None
    Lx, Ly = map(float, layout.get("map_size_m", [20.0, 20.0]))
    geom = layout.setdefault("geometry", {})
    endcaps = geom.setdefault("endcaps", [])
    horizontal = abs(rect[2] - rect[0]) >= abs(rect[3] - rect[1])
    pad = 0.4
    if horizontal:
        occl = _clamp_rect([rect[0] - pad, rect[3], rect[0] + 0.8, rect[3] + 1.2], (Lx, Ly))
        lane_val = float(rect[0] + 0.8)
        geom["blind_corner_lane_x"] = lane_val
    else:
        occl = _clamp_rect([rect[2], rect[1] - pad, rect[2] + 1.2, rect[1] + 0.8], (Lx, Ly))
        lane_val = float(rect[1] + 0.8)
        geom["blind_corner_lane_y"] = lane_val
    endcaps.append({"id": f"BlindCornerCap_{len(endcaps)+1:02d}", "aabb": occl, "height_m": 2.4})
    return ("x" if horizontal else "y", lane_val)

def _add_high_racks_from_primary(layout: Dict[str, Any], height: float = 5.0,
                                 gap_m: float = 0.2, depth_m: float = 0.95,
                                 rid_prefix: str = "HighRack") -> bool:
    aisles = layout.get("aisles") or []
    if not aisles:
        return False
    rect = aisles[0].get("rect")
    if not rect or len(rect) != 4:
        return False
    Lx, Ly = map(float, layout.get("map_size_m", [20.0, 20.0]))
    rack_cfg = {"gap_m": gap_m, "depth_m": depth_m, "height_m": height, "type": "high_bay", "reflective": True}
    _add_racking_bands_from_aisle(layout.setdefault("geometry", {}), list(map(float, rect)), rack_cfg, (Lx, Ly), rid_prefix)
    return True

def _anchor_point_from_layout(layout: Dict[str, Any]) -> tuple[float, float]:
    map_size = layout.get("map_size_m", [20.0, 20.0])
    try:
        Lx, Ly = float(map_size[0]), float(map_size[1])
    except Exception:
        Lx, Ly = 20.0, 20.0
    aisles = layout.get("aisles") or []
    for aisle in aisles:
        rect = aisle.get("rect")
        if rect and len(rect) == 4:
            return _rect_center(list(map(float, rect)))
    junctions = layout.get("junctions") or []
    for junc in junctions:
        rect = junc.get("rect") or junc.get("zone")
        if rect and len(rect) == 4:
            return _rect_center(list(map(float, rect)))
    return (0.5 * Lx, 0.5 * Ly)

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

def _detect_coord_mentions(prompt: str) -> Dict[str, bool]:
    return {
        "aisle": bool(_PASSAGE_SEG_RE.search(prompt)),
        "human": bool(_HUMAN_SEG_RE.search(prompt)),
        "vehicle": bool(_VEHICLE_SEG_RE.search(prompt)),
        "traction": bool(_PATCH_SEG_RE.search(prompt)),
        "static": bool(_STATIC_PT_RE.search(prompt)),
        "wall": bool(_WALL_SEG_RE.search(prompt)),
        "rack": bool(_RACK_SEG_RE.search(prompt)),
    }

def _extract_intent(prompt: str, coord_flags: Dict[str, bool] | None = None) -> Dict[str, Any]:
    text_l = prompt.lower()
    coord = coord_flags or _detect_coord_mentions(prompt)
    wants_passage_terms = any(term in text_l for term in ("aisle", "aisles", "corridor", "crosswalk", "lane", "passage"))
    intent = {
        "wants_long_main_passage": _is_main_aisle_lr_prompt(prompt) or "long corridor" in text_l or "long aisle" in text_l,
        "wants_narrow_passage": _has_any(text_l, KEYWORDS["narrow"]),
        "wants_parallel_passages": _wants_parallel_aisles(prompt) or _is_parallel_pass_prompt(text_l),
        "has_coordinate_passages": coord.get("aisle", False),
        "mentions_humans": _has_any(text_l, KEYWORDS["human"]) or coord.get("human", False),
        "busy_crossing": _has_any(text_l, KEYWORDS["busy_crossing"]),
        "distracted_worker": _has_any(text_l, KEYWORDS["distracted"]),
        "mentions_crossing": _mentions_crossing(text_l),
        "no_humans_global": _is_no_human(text_l),
        "mentions_forklift": _has_any(text_l, KEYWORDS["forklift"]) or coord.get("vehicle", False),
        "mentions_pallet_jack": _has_any(text_l, KEYWORDS["pallet_jack"]),
        "traffic_heavy": _has_any(text_l, KEYWORDS["traffic"]) or _has_any(text_l, KEYWORDS["congestion"]) or _has_any(text_l, KEYWORDS["busy_aisle"]),
        "no_vehicles_global": _is_no_vehicle(text_l),
        "forklift_reverse": _has_any(text_l, KEYWORDS["forklift_reverse"]),
        "slippery_surface": _has_any(text_l, KEYWORDS["traction"]),
        "cleaning_in_progress": _has_any(text_l, KEYWORDS["cleaning"]),
        "blind_corner": _has_any(text_l, KEYWORDS["blind_corner"]),
        "high_racks": (_has_any(text_l, KEYWORDS["high_shelf"]) or _wants_between_racks(text_l)) and not coord.get("rack", False),
        "endcap_rack": "endcap" in text_l,
        "low_visibility": _has_any(text_l, KEYWORDS["visibility"]),
        "reflective_clothing": _has_any(text_l, KEYWORDS["reflective"]),
        "dark_clothing": _has_any(text_l, KEYWORDS["dark_clothing"]),
        "sensor_fault": _has_any(text_l, KEYWORDS["sensor_fault"]),
        "degraded_mode": _has_any(text_l, KEYWORDS["degraded_mode"]),
        "falling_object": _has_any(text_l, KEYWORDS["falling"]),
        "wants_walls": _wants_walled_aisles(text_l),
        "wants_between_racks": _wants_between_racks(text_l),
        "wants_passage_generic": wants_passage_terms,
    }
    return intent

def _add_crosswalk_lane(layout: Dict[str, Any], prefer_axis: float | None = None) -> Dict[str, Any] | None:
    aisles = layout.get("aisles") or []
    if not aisles:
        return None
    main = aisles[0]
    rect = main.get("rect")
    if not rect or len(rect) != 4:
        return None
    Lx, Ly = map(float, layout.get("map_size_m", [20.0, 20.0]))
    bounds = (Lx, Ly)
    # reuse existing auto crosswalk if present
    for a in aisles:
        if a.get("id") == "A_cross_auto" or a.get("name") == "crosswalk_auto":
            c_rect = a.get("rect")
            if c_rect and len(c_rect) == 4:
                return {"center": _rect_center(c_rect), "orientation": "vertical" if (abs(rect[2] - rect[0]) >= abs(rect[3] - rect[1])) else "horizontal"}
    horizontal = abs(rect[2] - rect[0]) >= abs(rect[3] - rect[1])
    width_minor = min(abs(rect[2] - rect[0]), abs(rect[3] - rect[1]))
    cross_width = min(1.8, max(1.0, width_minor * 1.1))
    span = max(2.5, width_minor * 4.0)
    if horizontal:
        cross_x = float(prefer_axis if prefer_axis is not None else 0.5 * (rect[0] + rect[2]))
        cross_rect = _clamp_rect([cross_x - 0.5 * cross_width, max(0.0, rect[1] - span),
                                  cross_x + 0.5 * cross_width, min(Ly, rect[3] + span)], bounds)
        orientation = "vertical"
    else:
        cross_y = float(prefer_axis if prefer_axis is not None else 0.5 * (rect[1] + rect[3]))
        cross_rect = _clamp_rect([max(0.0, rect[0] - span), cross_y - 0.5 * cross_width,
                                  min(Lx, rect[2] + span), cross_y + 0.5 * cross_width], bounds)
        orientation = "horizontal"
    entry = {
        "id": "A_cross_auto",
        "name": "crosswalk_auto",
        "rect": cross_rect,
        "type": "cross",
        "racking": False,
        "pad": [0.1, 0.1, 0.1, 0.1],
    }
    layout.setdefault("aisles", []).append(entry)
    geom = layout.setdefault("geometry", {})
    _carve_racks_for_cut(geom, cross_rect, pad=0.1)
    return {"center": _rect_center(cross_rect), "orientation": orientation}

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
    layout["prevent_auto_crosswalk"] = True
    layout["auto_aisles_from_paths"] = False
    layout["paths_define_aisles"] = False
    layout["start"] = [3.0, 5.2]
    layout["goal"] = [17.0, 5.2]
    geom["racking"] = []
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

def prompt_to_scenario_legacy(prompt: str, n_runs: int = 100) -> Dict[str, Any]:
    scn = new_scenario(n_runs)
    text = prompt.lower()
    nums = parse_numbers(prompt)
    # Disable heavy templates (main LR, T) per request; treat prompts generically instead.
    t_case = False
    main_lr_case = False
    simple_aisle_case = False
    parallel_aisle_case = False
    single_straight_case = _is_single_straight_prompt(text)
    parallel_pass_case = _is_parallel_pass_prompt(text)
    narrow_cross_case = False
    wide_main_case = _is_wide_main_prompt(text)
    overhang_flag = False
    if single_straight_case:
        _apply_single_straight_layout(scn)
    elif wide_main_case:
        _apply_wide_main_layout(scn)
    _ensure_base_layout(scn)
    coord_flags = _apply_coordinate_overrides(scn, prompt, nums)
    # honor explicit exclusions
    if _is_no_human(text):
        scn.setdefault("hazards", {})["human"] = []
        scn.setdefault("taxonomy", {})["no_humans"] = True
    if _is_no_vehicle(text):
        scn.setdefault("hazards", {})["vehicles"] = []
    if ("aisle" in text and "no aisle" not in text and "without aisle" not in text and
        not simple_aisle_case and not main_lr_case and not t_case and not parallel_aisle_case and not coord_flags.get("aisle")):
        scn["layout"]["auto_aisles_from_paths"] = True

    # convenience handle for overrides
    def _so_root() -> Dict[str, Any]:
        return scn.setdefault("site_overrides", {})

    # Add walls flanking aisles only when explicitly requested (not just "between racks")
    wants_walls = _wants_walled_aisles(text)
    wants_between = _wants_between_racks(text)
    wall_color = (0.55, 0.35, 0.2, 1.0) if wants_walls else None
    if wants_walls:
        # If no aisles are defined yet and the user just asked for a walled/between-racks aisle, build a minimal straight aisle to hang the walls on.
        layout_obj = scn.get("layout", {})
        if not layout_obj.get("aisles") and not coord_flags.get("aisle"):
            _apply_single_straight_layout(scn)
        layout_obj = scn.get("layout", {})
        layout_obj = scn.get("layout", {})
        aisles = layout_obj.get("aisles") or []
        for idx, aisle in enumerate(aisles):
            rect = aisle.get("rect")
            if not rect or len(rect) != 4:
                continue
            if _has_any(text, KEYWORDS["narrow"]):
                x0, y0, x1, y1 = map(float, rect)
                horizontal = abs(x1 - x0) >= abs(y1 - y0)
                if horizontal:
                    cy = 0.5 * (y0 + y1)
                    aisle["rect"] = [x0, cy - 0.5, x1, cy + 0.5]
                else:
                    cx = 0.5 * (x0 + x1)
                    aisle["rect"] = [cx - 0.5, y0, cx + 0.5, y1]
            _add_walls_for_aisle(layout_obj, list(map(float, rect)), aisle.get("id") or f"Aisle_{idx:02d}", color=wall_color)
    else:
        # If high racks/ between racks were requested but walls were not, ensure no stray walls remain.
        if _has_any(text, KEYWORDS["high_shelf"]) or wants_between:
            scn.get("layout", {}).pop("walls", None)

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
    if _has_any(text, KEYWORDS["traction"]) and not coord_flags.get("traction") and not _has_any(text, KEYWORDS["blind_corner"]) and not _has_any(text, KEYWORDS["cleaning"]):
        # place a single neutral patch adjacent to the primary aisle, not covering its centerline
        geom_center = None
        patch_half = 1.0

        def _centered_patch_from_rect(rect: List[float]) -> tuple[float, float, float]:
            cx, cy = _rect_center(rect)
            span_major = max(abs(rect[2] - rect[0]), abs(rect[3] - rect[1]))
            span_minor = min(abs(rect[2] - rect[0]), abs(rect[3] - rect[1]))
            half_local = min(1.2, max(0.5, 0.35 * span_minor))
            return cx, cy, half_local

        aisles = scn.get("layout", {}).get("aisles") or []
        if aisles and aisles[0].get("rect"):
            cx, cy, patch_half = _centered_patch_from_rect(list(map(float, aisles[0]["rect"])))
            geom_center = (cx, cy)
        elif scn.get("layout", {}).get("junctions"):
            rect = scn["layout"]["junctions"][0].get("rect")
            if rect and len(rect) == 4:
                cx, cy, patch_half = _centered_patch_from_rect(list(map(float, rect)))
                geom_center = (cx, cy)
        if geom_center is None:
            map_size = scn.get("layout", {}).get("map_size_m", [20.0, 20.0])
            geom_center = (0.5 * float(map_size[0]), 0.5 * float(map_size[1]))
        _add_default_wet_patch_if_needed(geom_center, half=patch_half)

    if _has_any(text, KEYWORDS["cleaning"]):
        layout_obj = scn.get("layout", {})
        clean_zone = [5.0, 1.0, 9.0, 2.4]
        aisles = layout_obj.get("aisles") or []
        if aisles and isinstance(aisles[0].get("rect"), (list, tuple)) and len(aisles[0]["rect"]) == 4:
            rect = list(map(float, aisles[0]["rect"]))
            horizontal = abs(rect[2] - rect[0]) >= abs(rect[3] - rect[1])
            if horizontal:
                cx = 0.5 * (rect[0] + rect[2])
                y0, y1 = rect[1], rect[3]
                clean_zone = [cx - 2.0, y0, cx + 2.0, y1]
            else:
                cy = 0.5 * (rect[1] + rect[3])
                x0, x1 = rect[0], rect[2]
                clean_zone = [x0, cy - 2.0, x1, cy + 2.0]
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
            "spawn_time_s": 5.0,
            "spread_rate_mps": 0.12,
            "mu": 0.33,
        })
        scn["taxonomy"]["traction"] = True
        # avoid also adding generic traction patches when cleaning is present
        coord_flags["traction"] = True
        # add a small threshold bump for the doorway
        _add_transition_zone(scn, {
            "type": "threshold_bump",
            "zone": clean_zone,
            "threshold_cm": 2.0,
            "slope_deg": 4.0,
            "sensor_shadow": False,
        })

    # --- Human crossing (generic, single stream) ---
    if (_has_any(text, KEYWORDS["human"]) and not scn["layout"].get("main_lr_case")
        and not scn["layout"].get("t_case") and not coord_flags.get("human")
        and not _has_any(text, KEYWORDS["blind_corner"]) and not _is_no_human(text)):
        rate = nums.get("human_rate_per_min", 2.0)  # every 30s default
        cfg = _ensure_human_entry(scn, len(scn["hazards"]["human"]))
        cfg.update({
            "path": "line",
            "rate_per_min": float(rate),
            "speed_mps": [0.9, 1.5],
        })
        cfg["start_delay_s"] = 0.0
        cfg["start_delay_s_min"] = 0.0
        cfg["start_delay_s_max"] = 0.0
        cfg.setdefault("motion_patterns", ["linear", "start_stop", "hesitation", "emerge_from_occlusion"])
        cfg.setdefault("interaction_modes", ["mutual_yield", "human_yield", "robot_yield", "close_pass"])
        cfg.setdefault("visibility_awareness", "normal")
        layout_obj = scn.get("layout", {})
        wants_crosswalk_geom = _wants_crosswalk_geom(text)
        if (wants_crosswalk_geom and not layout_obj.get("_raw_coord_aisles") and not cfg.get("raw_coords")
            and not layout_obj.get("prevent_auto_crosswalk", False)):
            aisles = layout_obj.get("aisles") or []
            if len(aisles) == 1 and aisles[0].get("rect"):
                cross = _add_crosswalk_lane(layout_obj)
                if cross and cross.get("orientation") == "vertical" and "cross_x" not in cfg:
                    cfg["cross_x"] = float(cross["center"][0])
        scn["taxonomy"]["human_behavior"] = True

    # --- Traffic / congestion keywords imply multi-actor environments ---
    if ((_has_any(text, KEYWORDS["traffic"]) or
        _has_any(text, KEYWORDS["busy_aisle"]) or
        _has_any(text, KEYWORDS["congestion"])) and not scn.setdefault("hazards", {}).get("vehicles")):
        scn["taxonomy"]["multi_actor"] = True
        # Do not auto-spawn vehicle fleets; leave vehicles empty unless explicitly specified.

    # --- Overhangs / irregular loads increase occlusion pressure ---
    if _has_any(text, KEYWORDS["overhang"]):
        scn["taxonomy"]["occlusion"] = True
        overhang_flag = True
        veh_list = scn.setdefault("hazards", {}).setdefault("vehicles", [])
        if not veh_list and not coord_flags.get("vehicle"):
            map_size = scn.get("layout", {}).get("map_size_m", [20.0, 20.0])
            Lx = float(map_size[0]) if isinstance(map_size, (list, tuple)) else 20.0
            Ly = float(map_size[1]) if isinstance(map_size, (list, tuple)) else 20.0
            _ensure_vehicle(scn, {
                "id": "Forklift_Overhang_01",
                "type": "forklift",
                "path": [[2.0, 0.5 * Ly], [Lx - 2.0, 0.5 * Ly]],
                "speed_mps": 0.85,
                "ping_pong": True,
                "warning_lights": True,
                "reversing_bias": False,
            })
            veh_list = scn["hazards"]["vehicles"]
        elif veh_list:
            overhang_flag = True
        for veh in veh_list:
            if veh.get("type") != "forklift":
                continue
            half = list(map(float, veh.get("half_extents", [0.55, 0.45, 0.55])))
            half[0] = max(0.85, half[0] + 0.25)  # extend forward/rear footprint
            veh["half_extents"] = half
            veh["carrying_pallet"] = True
            veh["reflective"] = True
            veh.setdefault("load_overhang_m", 0.4)
            veh.setdefault("rear_occlusion_deg", 80.0)

    # --- Narrow aisle hint (keep lightweight; world builder may interpret later) ---
    if _has_any(text, KEYWORDS["narrow"]):
        scn["layout"]["narrow_aisle_hint"] = True  # advisory flag for world builder
        # note: taxonomy doesn't have "narrow" in schema, but dict is open
        scn["taxonomy"]["narrow"] = True  # type: ignore[index]
        _apply_narrowing(scn["layout"], scale=0.65, aisle_indices=[0])
        # add racking flanking the primary aisle when "between racks" is implied
        if _wants_between_racks(text):
            layout_obj = scn.get("layout", {})
            aisles = layout_obj.get("aisles") or []
            if aisles and aisles[0].get("rect"):
                rect = list(map(float, aisles[0]["rect"]))
                Lx, Ly = map(float, layout_obj.get("map_size_m", [20.0, 20.0]))
                _add_racking_bands_from_aisle(layout_obj.setdefault("geometry", {}), rect,
                                              {"gap_m": 0.15, "depth_m": 0.8, "height_m": 3.2, "type": "rack"},
                                              (Lx, Ly), "Rack_Narrow")

    # --- Falling object emphasis (nudges injector probabilities via site_overrides) ---
    if _has_any(text, KEYWORDS["falling"]):
        so_root = _so_root()
        so_inj = so_root.setdefault("injectors", {})
        f = so_inj.setdefault("falling_object", {})
        # Bump falling object probability; leave others unchanged unless scenario says otherwise
        f["p"] = 1.0
        # Aim the drop into an aisle center if available; otherwise map center
        layout_obj = scn.get("layout", {})
        drop_xy = (10.0, 10.0)
        aisles = layout_obj.get("aisles") or []
        if aisles and isinstance(aisles[0].get("rect"), (list, tuple)) and len(aisles[0]["rect"]) == 4:
            rect = list(map(float, aisles[0]["rect"]))
            cx, cy = _rect_center(rect)
            horizontal = abs(rect[2] - rect[0]) >= abs(rect[3] - rect[1])
            offset = min(0.8, max(0.35, 0.25 * (abs(rect[3] - rect[1]) if horizontal else abs(rect[2] - rect[0]))))
            if horizontal:
                cy = cy + offset
            else:
                cx = cx + offset
            drop_xy = (cx, cy)
        elif layout_obj.get("geometry", {}).get("racking"):
            rack = layout_obj["geometry"]["racking"][0]
            if isinstance(rack.get("aabb"), (list, tuple)) and len(rack["aabb"]) == 4:
                drop_xy = _rect_center(list(map(float, rack["aabb"])))
        map_size = layout_obj.get("map_size_m", [20.0, 20.0])
        Lx = float(map_size[0]) if isinstance(map_size, (list, tuple)) else 20.0
        Ly = float(map_size[1]) if isinstance(map_size, (list, tuple)) else 20.0
        drop_xy = (max(0.8, min(Lx - 0.8, drop_xy[0])), max(0.8, min(Ly - 0.8, drop_xy[1])))
        f.setdefault("drop_x", drop_xy[0])
        f.setdefault("drop_y", drop_xy[1])
        f.setdefault("drop_z", 3.2)
        f.setdefault("puddle_half", [1.2, 0.7])

    # --- Two-crosswalks special case (disabled to allow manual multi-crosswalk setup) ---
    if _has_any(text, KEYWORDS["two_crosswalks"]):
        pass  # leave humans empty; caller can specify multiple crossings via coordinates/templates

    # --- Distracted human (phone, late reaction) ---
    if _has_any(text, KEYWORDS["distracted"]):
        scn["taxonomy"]["human_behavior"] = True
        so_root = _so_root()
        h = so_root.setdefault("human", {})
        # more likely to slip when distracted
        h.setdefault("slip_prob", 0.95)

    if _has_any(text, KEYWORDS["worker_carry"]) and not _is_no_human(text):
        cfg = _ensure_human_entry(scn, 0)
        _add_behavior_tag(cfg, "carry_payload")
        cfg["carried_mass_kg"] = 12.0
        cfg["speed_mps"] = [0.7, 1.2]
        cfg["posture"] = "leaning"
        _append_unique(cfg.setdefault("motion_patterns", []), "reduced_visibility")
        scn["taxonomy"]["human_behavior"] = True

    if _has_any(text, KEYWORDS["fast_walker"]) and not _is_no_human(text):
        cfg = _ensure_human_entry(scn, 0)
        cfg["speed_mps"] = [1.6, 2.2]
        _add_behavior_tag(cfg, "fast_walk")
        _append_unique(cfg.setdefault("motion_patterns", []), "zig_zag")
        # ensure a moving waypoint path exists
        pts_fw = _waypoints_for_human_mode("move_along", scn.get("layout", {}))
        if pts_fw and len(pts_fw) >= 2:
            cfg["path"] = "waypoints"
            cfg["waypoints"] = pts_fw
            cfg["raw_coords"] = True
            cfg["loop"] = False
        scn["taxonomy"]["human_behavior"] = True

    if _has_any(text, KEYWORDS["child_actor"]) and not _is_no_human(text):
        if not coord_flags.get("human"):
            scn.setdefault("hazards", {})["human"] = []
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
        pts_child = _waypoints_for_human_mode("move_across", scn.get("layout", {}))
        if pts_child and len(pts_child) >= 2:
            cfg["path"] = "waypoints"
            cfg["waypoints"] = pts_child
            cfg["raw_coords"] = True
            cfg["loop"] = False

    # --- Busy crossing / groups / platoons ---
    if _has_any(text, KEYWORDS["busy_crossing"]) and not _is_no_human(text):
        scn["taxonomy"]["human_behavior"] = True
        scn["taxonomy"]["multi_actor"] = True
        if not scn.setdefault("hazards", {}).get("human"):
            cfg = _ensure_human_entry(scn, 0)
            cfg.update({
                "path": "line",
                "rate_per_min": max(nums.get("human_rate_per_min", 2.0), 3.0),
                "speed_mps": [0.8, 1.2],
            })
        for idx in range(len(scn["hazards"]["human"])):
            cfg = _ensure_human_entry(scn, idx)
            cfg["group_size"] = max(3, int(cfg.get("group_size", 4)))
            cfg["group_size_range"] = [cfg["group_size"], max(cfg["group_size"], 6)]
            _append_unique(cfg.setdefault("motion_patterns", []), "close_pass")
            _append_unique(cfg.setdefault("interaction_modes", []), "mutual_yield")

    # --- Blind corner / occluded crosswalks ---
    if _has_any(text, KEYWORDS["blind_corner"]):
        scn["taxonomy"]["occlusion"] = True
        scn["taxonomy"]["visibility"] = True  # night/low light
        so_root = _so_root()
        so_inj = so_root.setdefault("injectors", {})
        blk = so_inj.setdefault("lidar_blackout", {})
        blk.setdefault("p", 0.60)
    if _has_any(text, KEYWORDS["high_shelf"]):
        scn["taxonomy"]["occlusion"] = True

    # --- Forklift-related chaos (load overhangs, drops) ---
    if (_has_any(text, KEYWORDS["forklift"]) and
        not scn["layout"].get("main_lr_case") and
        not scn["layout"].get("t_case") and
        not scn["layout"].get("simple_forklift_aisle") and
        not coord_flags.get("vehicle") and
        not _is_no_vehicle(text) and
        not scn.setdefault("hazards", {}).get("vehicles") and
        not _has_any(text, KEYWORDS["forklift_reverse"]) and
        not overhang_flag):
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

    if (_has_any(text, KEYWORDS["forklift_reverse"]) and not coord_flags.get("vehicle")
        and not scn.setdefault("hazards", {}).get("vehicles") and not _is_no_vehicle(text)):
        ref = _primary_aisle_ref(scn.get("layout", {}))
        path_rev = [[6.0, 4.0], [14.0, 4.0]]
        if ref:
            inset = 0.6
            if ref["horizontal"]:
                path_rev = [[ref["rect"][0] + inset, ref["center"][1]], [ref["rect"][2] - inset, ref["center"][1]]]
            else:
                path_rev = [[ref["center"][0], ref["rect"][1] + inset], [ref["center"][0], ref["rect"][3] - inset]]
        _ensure_vehicle(scn, {
            "id": f"ForkliftRev_{len(scn.setdefault('hazards', {}).setdefault('vehicles', []))+1:02d}",
            "type": "forklift",
            "path": path_rev,
            "speed_mps": 0.75,
            "reversing_mode": True,
            "raw_coords": True,
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
    if (_has_any(text, KEYWORDS["pallet_jack"]) and not coord_flags.get("vehicle")
        and not scn.setdefault("hazards", {}).get("vehicles") and not _is_no_vehicle(text)):
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
            layout_obj = scn.get("layout", {})
            aisles = layout_obj.get("aisles") or []
            aabb = [9.2, 7.6, 10.8, 8.8]
            if aisles and isinstance(aisles[0].get("rect"), (list, tuple)) and len(aisles[0]["rect"]) == 4:
                rect = list(map(float, aisles[0]["rect"]))
                horizontal = abs(rect[2] - rect[0]) >= abs(rect[3] - rect[1])
                if horizontal:
                    cx = 0.5 * (rect[0] + rect[2])
                    y0, y1 = rect[1], rect[3]
                    width = abs(y1 - y0)
                    block_span = min(1.2, max(0.8, width * 0.9))
                    aabb = [cx - 0.6, 0.5 * (y0 + y1) - 0.5 * block_span, cx + 0.6, 0.5 * (y0 + y1) + 0.5 * block_span]
                else:
                    cy = 0.5 * (rect[1] + rect[3])
                    x0, x1 = rect[0], rect[2]
                    width = abs(x1 - x0)
                    block_span = min(1.2, max(0.8, width * 0.9))
                    aabb = [0.5 * (x0 + x1) - 0.5 * block_span, cy - 0.6, 0.5 * (x0 + x1) + 0.5 * block_span, cy + 0.6]
            _add_static_obstacle(scn, {
                "type": "cart_block",
                "shape": "box",
                "aabb": aabb,
                "height": 1.2,
                "occlusion": True,
            })
        scn["taxonomy"]["multi_actor"] = True

    if _has_any(text, KEYWORDS["transition"]):
        layout_obj = scn.get("layout", {})
        zone = [4.0, 0.0, 5.0, 1.4]
        aisles = layout_obj.get("aisles") or []
        if aisles and isinstance(aisles[0].get("rect"), (list, tuple)) and len(aisles[0]["rect"]) == 4:
            rect = list(map(float, aisles[0]["rect"]))
            horizontal = abs(rect[2] - rect[0]) >= abs(rect[3] - rect[1])
            if horizontal:
                x0, x1 = rect[0], rect[2]
                y0, y1 = rect[1], rect[3]
                zone = [x1 - 0.8, y0, x1 + 0.6, y1]
            else:
                x0, x1 = rect[0], rect[2]
                y0, y1 = rect[1], rect[3]
                zone = [x0, y1 - 0.8, x1, y1 + 0.6]
        _add_transition_zone(scn, {
            "type": "fire_door",
            "zone": zone,
            "threshold_cm": 2.0,
            "slope_deg": 4.0,
            "sensor_shadow": True,
        })
        scn["taxonomy"]["occlusion"] = True

    # --- Clothing / reflectivity  affect lidar noise/dropout ---
    if _has_any(text, KEYWORDS["reflective"]):
        scn["taxonomy"]["visibility"] = True
        cfg = _ensure_human_entry(scn, 0)
        cfg["reflectivity"] = 0.8
        cfg.setdefault("visibility_awareness", "normal")
        so_root = _so_root()
        lid = so_root.setdefault("lidar", {})
        lid.setdefault("noise_sigma", 0.01)
        lid.setdefault("dropout", 0.005)

    if _has_any(text, KEYWORDS["dark_clothing"]):
        scn["taxonomy"]["visibility"] = True
        cfg = _ensure_human_entry(scn, 0)
        cfg["reflectivity"] = -0.6
        cfg.setdefault("visibility_awareness", "reduced")
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
        so_inj.setdefault("ghost_obstacle", {}).setdefault("spawn_rate_hz", 0.35)

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
                "speed_mps": [0.9, 1.5],
                "group_size": 3,
                "waypoints": [[10.0, 6.5], [10.0, 12.5]],
            })
            cfg["motion_patterns"] = ["linear", "start_stop", "close_pass"]
            cfg["interaction_modes"] = ["mutual_yield", "human_yield", "robot_yield"]
        scn["taxonomy"]["human_behavior"] = True

    # Re-apply narrowing after any template aisles were created
    if scn.get("layout", {}).get("narrow_aisle_hint"):
        _apply_narrowing(scn["layout"], scale=0.65, aisle_indices=[0])

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

    _ensure_default_human_paths(scn)
    _apply_human_motion_intent(scn, text, nums.get("human_rate_per_min"))

    if layout.get("walls"):
        _carve_wall_gaps(layout, pad=0.08)
        geom = layout.setdefault("geometry", {})
        if not _wants_between_racks(text) and not _has_any(text, KEYWORDS["high_shelf"]) and not coord_flags.get("rack", False):
            geom["racking"] = []  # walls handle occlusion; drop stray rack bands that look like purple walls
    if intent.get("high_racks"):
        _normalize_high_racks(layout)

    _ensure_map_bounds(layout, scn.get("hazards", {}))
    _retreat_goal_from_transitions(layout)
    _ensure_start_goal_open(layout, tuple(layout["map_size_m"]))
    return scn


# --- New object-first parser entrypoint ---
def prompt_to_scenario(prompt: str, n_runs: int = 100) -> Dict[str, Any]:
    scn = new_scenario(n_runs)
    text = prompt.lower()
    nums = parse_numbers(prompt)
    coord_detect = _detect_coord_mentions(prompt)
    intent = _extract_intent(prompt, coord_detect)

    coord_flags = _apply_coordinate_overrides(scn, prompt, nums)
    for k, v in coord_flags.items():
        if v:
            coord_detect[k] = True
    intent["has_coordinate_passages"] = intent.get("has_coordinate_passages") or coord_detect.get("aisle", False)
    if intent.get("no_humans_global"):
        scn.setdefault("taxonomy", {})["no_humans"] = True

    # Prefer explicit templates for canonical demo phrases when no coordinate aisles are provided.
    layout_obj = scn.get("layout", {})
    template_applied = False
    if not coord_flags.get("aisle"):
        if _is_single_straight_prompt(text):
            _apply_single_straight_layout(scn)
            layout_obj = scn.get("layout", {})
            layout_obj["single_case"] = True
            template_applied = True
        elif _is_wide_main_prompt(text):
            _apply_wide_main_layout(scn)
            layout_obj = scn.get("layout", {})
            layout_obj["wide_main_case"] = True
            template_applied = True

    if not template_applied and not coord_flags.get("aisle"):
        _assemble_passages_from_intent(scn, intent)
    if not template_applied:
        _decorate_passages(scn, intent, text)
    blind_lane_hint = None
    if intent.get("blind_corner"):
        # Avoid hardcoded blind-corner geometry; leave layout to coordinates/templates.
        scn["taxonomy"]["occlusion"] = True
        scn["taxonomy"]["visibility"] = True
        so_root = scn.setdefault("site_overrides", {})
        so_inj = so_root.setdefault("injectors", {})
        blk = so_inj.setdefault("lidar_blackout", {})
        blk.setdefault("p", 0.60)
    _ensure_map_bounds(scn["layout"], scn.get("hazards", {}))
    _retreat_goal_from_transitions(scn["layout"])
    _ensure_start_goal_open(scn["layout"], tuple(scn["layout"]["map_size_m"]))

    if intent.get("low_visibility"):
        scn["taxonomy"]["visibility"] = True
        lid = scn.setdefault("site_overrides", {}).setdefault("lidar", {})
        lid.setdefault("noise_sigma", 0.03)
        lid.setdefault("dropout", 0.05)
        lid.setdefault("latency_ms_max", 70)

    layout_obj = scn.get("layout", {})
    hazards = scn.setdefault("hazards", {})
    anchor = _anchor_point_from_layout(layout_obj)
    primary_rect = None
    if layout_obj.get("aisles") and layout_obj["aisles"][0].get("rect"):
        primary_rect = list(map(float, layout_obj["aisles"][0]["rect"]))

    def _patch_from_rect(rect: List[float]) -> tuple[List[float], float]:
        cx, cy = _rect_center(rect)
        span_minor = min(abs(rect[2] - rect[0]), abs(rect[3] - rect[1]))
        half_local = min(1.2, max(0.5, 0.35 * span_minor))
        return [cx - half_local, cy - half_local, cx + half_local, cy + half_local], half_local

    clean_zone = None
    if intent.get("cleaning_in_progress"):
        clean_zone = [max(0.5, anchor[0] - 2.0), max(0.5, anchor[1] - 1.0), anchor[0] + 2.0, anchor[1] + 1.0]
        if primary_rect:
            horizontal = abs(primary_rect[2] - primary_rect[0]) >= abs(primary_rect[3] - primary_rect[1])
            if horizontal:
                cx = 0.5 * (primary_rect[0] + primary_rect[2])
                clean_zone = [cx - 2.0, primary_rect[1], cx + 2.0, primary_rect[3]]
            else:
                cy = 0.5 * (primary_rect[1] + primary_rect[3])
                clean_zone = [primary_rect[0], cy - 2.0, primary_rect[2], cy + 2.0]
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
            "spawn_time_s": 5.0,
            "spread_rate_mps": 0.12,
            "mu": 0.33,
        })
        _add_transition_zone(scn, {
            "type": "threshold_bump",
            "zone": clean_zone,
            "threshold_cm": 2.0,
            "slope_deg": 4.0,
            "sensor_shadow": False,
        })
        scn["taxonomy"]["traction"] = True

    if intent.get("slippery_surface") and not coord_flags.get("traction"):
        if primary_rect:
            patch_rect, _ = _patch_from_rect(primary_rect)
        elif clean_zone:
            patch_rect = list(clean_zone)
        else:
            half = 1.0
            patch_rect = [anchor[0] - half, anchor[1] - half, anchor[0] + half, anchor[1] + half]
        patches = hazards.setdefault("traction", [])
        mu_val = nums.get("mu", 0.45 if not clean_zone else min(0.45, 0.33))
        patches.append({
            "id": f"Traction_{len(patches)+1:02d}",
            "type": "wet",
            "zone": patch_rect,
            "mu": float(mu_val),
        })
        scn["taxonomy"]["traction"] = True

    wants_generic_humans = intent.get("mentions_humans") or intent.get("busy_crossing") or intent.get("distracted_worker")
    if wants_generic_humans and not coord_flags.get("human") and not hazards.get("human") and not intent.get("no_humans_global"):
        rate = nums.get("human_rate_per_min", 2.0)
        cfg = _ensure_human_entry(scn, len(hazards.get("human", [])))
        cfg.update({
            "path": "line",
            "rate_per_min": float(rate),
            "speed_mps": [0.9, 1.5],
        })
        cfg["start_delay_s"] = 0.0
        cfg["start_delay_s_min"] = 0.0
        cfg["start_delay_s_max"] = 0.0
        cfg.setdefault("motion_patterns", ["linear", "start_stop", "hesitation", "emerge_from_occlusion"])
        cfg.setdefault("interaction_modes", ["mutual_yield", "human_yield", "robot_yield", "close_pass"])
        cfg.setdefault("visibility_awareness", "normal")
        wants_crosswalk_geom = _wants_crosswalk_geom(text)
        wants_crosswalk = wants_crosswalk_geom or bool(blind_lane_hint)
        if (wants_crosswalk and not layout_obj.get("_raw_coord_aisles") and not cfg.get("raw_coords")
            and not layout_obj.get("prevent_auto_crosswalk", False)):
            cross = _add_crosswalk_lane(layout_obj)
            if cross:
                if cross.get("orientation") == "vertical":
                    cfg["cross_x"] = float(cross["center"][0])
                else:
                    cfg["cross_y"] = float(cross["center"][1])
        if blind_lane_hint:
            axis, val = blind_lane_hint
            if axis == "x":
                cfg["cross_x"] = val
            else:
                cfg["cross_y"] = val
            _append_unique(cfg.setdefault("motion_patterns", []), "emerge_from_occlusion")
        scn["taxonomy"]["human_behavior"] = True

    wants_vehicle = (intent.get("mentions_forklift") or intent.get("traffic_heavy") or intent.get("mentions_pallet_jack"))
    if wants_vehicle and not coord_flags.get("vehicle") and not intent.get("no_vehicles_global") and not hazards.get("vehicles"):
        ref = _primary_aisle_ref(layout_obj)
        Lx, Ly = map(float, layout_obj.get("map_size_m", [20.0, 20.0]))
        def _path_along(ref_info: Dict[str, Any] | None) -> List[List[float]]:
            if ref_info:
                if ref_info["horizontal"]:
                    return [[ref_info["rect"][0] + 0.6, ref_info["center"][1]], [ref_info["rect"][2] - 0.6, ref_info["center"][1]]]
                return [[ref_info["center"][0], ref_info["rect"][1] + 0.6], [ref_info["center"][0], ref_info["rect"][3] - 0.6]]
            return [[1.5, 0.5 * Ly], [Lx - 1.5, 0.5 * Ly]]

        forklift_path = _path_along(ref)
        pallet_only = intent.get("mentions_pallet_jack") and not intent.get("mentions_forklift") and not intent.get("traffic_heavy")
        if not intent.get("forklift_reverse") and not pallet_only:
            _ensure_vehicle(scn, {
                "id": f"Forklift_{len(hazards.setdefault('vehicles', []))+1:02d}",
                "type": "forklift",
                "path": forklift_path,
                "speed_mps": 1.0 if intent.get("traffic_heavy") else 1.2,
                "warning_lights": True,
                "ping_pong": True,
            })
        if intent.get("mentions_pallet_jack") or intent.get("traffic_heavy"):
            pj_path = _path_along(ref)
            _ensure_vehicle(scn, {
                "id": f"PalletJack_{len(hazards.setdefault('vehicles', []))+1:02d}",
                "type": "pallet_jack",
                "path": pj_path,
                "speed_mps": 0.9,
                "ping_pong": True,
            })
        if intent.get("traffic_heavy"):
            scn["taxonomy"]["multi_actor"] = True

    if _has_any(text, KEYWORDS["overhang"]):
        scn["taxonomy"]["occlusion"] = True
        veh_list = scn.setdefault("hazards", {}).setdefault("vehicles", [])
        if not veh_list and not coord_flags.get("vehicle"):
            map_size = scn.get("layout", {}).get("map_size_m", [20.0, 20.0])
            Lx = float(map_size[0]) if isinstance(map_size, (list, tuple)) else 20.0
            Ly = float(map_size[1]) if isinstance(map_size, (list, tuple)) else 20.0
            _ensure_vehicle(scn, {
                "id": "Forklift_Overhang_01",
                "type": "forklift",
                "path": [[2.0, 0.5 * Ly], [Lx - 2.0, 0.5 * Ly]],
                "speed_mps": 0.85,
                "ping_pong": True,
                "warning_lights": True,
                "reversing_bias": False,
            })
            veh_list = scn["hazards"]["vehicles"]
        for veh in veh_list:
            if veh.get("type") != "forklift" or veh.get("raw_coords"):
                continue
            half = list(map(float, veh.get("half_extents", [0.55, 0.45, 0.55])))
            half[0] = max(0.85, half[0] + 0.25)
            veh["half_extents"] = half
            veh["carrying_pallet"] = True
            veh["reflective"] = True
            veh.setdefault("load_overhang_m", 0.4)
            veh.setdefault("rear_occlusion_deg", 80.0)

    if _has_any(text, KEYWORDS["forklift_reverse"]) and not coord_flags.get("vehicle"):
        ref = _primary_aisle_ref(layout_obj)
        path_rev = [[6.0, 4.0], [14.0, 4.0]]
        if ref:
            inset = 0.6
            if ref["horizontal"]:
                path_rev = [[ref["rect"][0] + inset, ref["center"][1]], [ref["rect"][2] - inset, ref["center"][1]]]
            else:
                path_rev = [[ref["center"][0], ref["rect"][1] + inset], [ref["center"][0], ref["rect"][3] - inset]]
        _ensure_vehicle(scn, {
            "id": f"ForkliftRev_{len(scn.setdefault('hazards', {}).setdefault('vehicles', []))+1:02d}",
            "type": "forklift",
            "path": path_rev,
            "speed_mps": 0.75,
            "reversing_mode": True,
        })
        for veh in scn["hazards"].setdefault("vehicles", []):
            if veh.get("type") != "forklift":
                continue
            veh["reversing_bias"] = True
            veh["warning_lights"] = True
            veh["alarm"] = "backup_beeper"
            veh["rear_occlusion_deg"] = 70.0
        scn["taxonomy"]["multi_actor"] = True

    if _has_any(text, KEYWORDS["cart_block"]):
        if coord_flags.get("cart_block"):
            pass
        else:
            aisles = layout_obj.get("aisles") or []
            aabb = [max(0.5, anchor[0] - 0.6), max(0.5, anchor[1] - 0.6), anchor[0] + 0.6, anchor[1] + 0.6]
            if aisles and isinstance(aisles[0].get("rect"), (list, tuple)) and len(aisles[0]["rect"]) == 4:
                rect = list(map(float, aisles[0]["rect"]))
                horizontal = abs(rect[2] - rect[0]) >= abs(rect[3] - rect[1])
                if horizontal:
                    cx = 0.5 * (rect[0] + rect[2])
                    span = min(1.2, max(0.8, abs(rect[3] - rect[1]) * 0.9))
                    aabb = [cx - 0.6, anchor[1] - 0.5 * span, cx + 0.6, anchor[1] + 0.5 * span]
                else:
                    cy = 0.5 * (rect[1] + rect[3])
                    span = min(1.2, max(0.8, abs(rect[2] - rect[0]) * 0.9))
                    aabb = [anchor[0] - 0.5 * span, cy - 0.6, anchor[0] + 0.5 * span, cy + 0.6]
            _add_static_obstacle(scn, {
                "type": "cart_block",
                "shape": "box",
                "aabb": aabb,
                "height": 1.2,
                "occlusion": True,
            })
        scn["taxonomy"]["multi_actor"] = True

    if _has_any(text, KEYWORDS["transition"]):
        zone = [anchor[0] - 0.5, max(0.0, anchor[1] - 1.0), anchor[0] + 0.5, anchor[1] + 0.5]
        aisles = layout_obj.get("aisles") or []
        horizontal = True
        if aisles and isinstance(aisles[0].get("rect"), (list, tuple)) and len(aisles[0]["rect"]) == 4:
            rect = list(map(float, aisles[0]["rect"]))
            horizontal = abs(rect[2] - rect[0]) >= abs(rect[3] - rect[1])
            if horizontal:
                zone = [rect[2] - 0.8, rect[1], rect[2] + 0.6, rect[3]]
            else:
                zone = [rect[0], rect[3] - 0.8, rect[2], rect[3] + 0.6]
        _add_transition_zone(scn, {
            "type": "fire_door",
            "zone": zone,
            "threshold_cm": 2.0,
            "slope_deg": 4.0,
            "sensor_shadow": True,
        })
        scn["taxonomy"]["occlusion"] = True

    if intent.get("wants_narrow_passage"):
        scn["layout"]["narrow_aisle_hint"] = True
        scn["taxonomy"]["narrow"] = True  # type: ignore[index]
        _apply_narrowing(scn["layout"], scale=0.65, aisle_indices=[0])

    if intent.get("falling_object"):
        so_root = scn.setdefault("site_overrides", {})
        so_inj = so_root.setdefault("injectors", {})
        f = so_inj.setdefault("falling_object", {})
        f["p"] = 1.0
        drop_xy = anchor
        if primary_rect:
            cx, cy = _rect_center(primary_rect)
            horizontal = abs(primary_rect[2] - primary_rect[0]) >= abs(primary_rect[3] - primary_rect[1])
            offset = min(0.8, max(0.35, 0.25 * (abs(primary_rect[3] - primary_rect[1]) if horizontal else abs(primary_rect[2] - primary_rect[0]))))
            if horizontal:
                cy = cy + offset
            else:
                cx = cx + offset
            drop_xy = (cx, cy)
        map_size = layout_obj.get("map_size_m", [20.0, 20.0])
        Lx = float(map_size[0]) if isinstance(map_size, (list, tuple)) else 20.0
        Ly = float(map_size[1]) if isinstance(map_size, (list, tuple)) else 20.0
        drop_xy = (max(0.8, min(Lx - 0.8, drop_xy[0])), max(0.8, min(Ly - 0.8, drop_xy[1])))
        f.setdefault("drop_x", drop_xy[0])
        f.setdefault("drop_y", drop_xy[1])
        f.setdefault("drop_z", 3.2)
        f.setdefault("puddle_half", [1.2, 0.7])
        # Ensure high racks exist for occlusion if none present.
        geom = layout_obj.setdefault("geometry", {})
        if not geom.get("racking"):
            _add_high_racks_from_primary(layout_obj, height=5.0, rid_prefix="HighRackFall")

    if intent.get("distracted_worker"):
        scn["taxonomy"]["human_behavior"] = True
        so_root = scn.setdefault("site_overrides", {})
        h = so_root.setdefault("human", {})
        h.setdefault("slip_prob", 0.95)

    if _has_any(text, KEYWORDS["worker_carry"]) and not intent.get("no_humans_global"):
        cfg = _ensure_human_entry(scn, 0)
        _add_behavior_tag(cfg, "carry_payload")
        cfg["carried_mass_kg"] = 12.0
        cfg["speed_mps"] = [0.7, 1.2]
        cfg["posture"] = "leaning"
        _append_unique(cfg.setdefault("motion_patterns", []), "reduced_visibility")
        scn["taxonomy"]["human_behavior"] = True

    if _has_any(text, KEYWORDS["fast_walker"]) and not intent.get("no_humans_global"):
        cfg = _ensure_human_entry(scn, 0)
        cfg["speed_mps"] = [1.6, 2.2]
        _add_behavior_tag(cfg, "fast_walk")
        _append_unique(cfg.setdefault("motion_patterns", []), "zig_zag")
        pts_fw = _waypoints_for_human_mode("move_along", scn.get("layout", {}))
        if pts_fw and len(pts_fw) >= 2:
            cfg["path"] = "waypoints"
            cfg["waypoints"] = pts_fw
            cfg["raw_coords"] = True
            cfg["loop"] = False
        scn["taxonomy"]["human_behavior"] = True

    if _has_any(text, KEYWORDS["child_actor"]) and not intent.get("no_humans_global"):
        if not coord_flags.get("human"):
            scn.setdefault("hazards", {})["human"] = []
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
        pts_child = _waypoints_for_human_mode("move_across", scn.get("layout", {}))
        if pts_child and len(pts_child) >= 2:
            cfg["path"] = "waypoints"
            cfg["waypoints"] = pts_child
            cfg["raw_coords"] = True
            cfg["loop"] = False

    if intent.get("busy_crossing") and not intent.get("no_humans_global"):
        scn["taxonomy"]["human_behavior"] = True
        scn["taxonomy"]["multi_actor"] = True
        if not scn.setdefault("hazards", {}).get("human"):
            cfg = _ensure_human_entry(scn, 0)
            cfg.update({
                "path": "line",
                "rate_per_min": max(nums.get("human_rate_per_min", 2.0), 3.0),
                "speed_mps": [0.8, 1.2],
            })
        for idx in range(len(scn["hazards"]["human"])):
            cfg = _ensure_human_entry(scn, idx)
            cfg["group_size"] = max(3, int(cfg.get("group_size", 4)))
            cfg["group_size_range"] = [cfg["group_size"], max(cfg["group_size"], 6)]
            _append_unique(cfg.setdefault("motion_patterns", []), "close_pass")
            _append_unique(cfg.setdefault("interaction_modes", []), "mutual_yield")

    if _has_any(text, KEYWORDS["high_shelf"]):
        scn["taxonomy"]["occlusion"] = True

    if intent.get("reflective_clothing"):
        scn["taxonomy"]["visibility"] = True
        cfg = _ensure_human_entry(scn, 0)
        cfg["reflectivity"] = 0.8
        cfg.setdefault("visibility_awareness", "normal")
        lid = scn.setdefault("site_overrides", {}).setdefault("lidar", {})
        lid.setdefault("noise_sigma", 0.01)
        lid.setdefault("dropout", 0.005)

    if intent.get("dark_clothing"):
        scn["taxonomy"]["visibility"] = True
        cfg = _ensure_human_entry(scn, 0)
        cfg["reflectivity"] = -0.6
        cfg.setdefault("visibility_awareness", "reduced")
        lid = scn.setdefault("site_overrides", {}).setdefault("lidar", {})
        lid.setdefault("noise_sigma", 0.03)
        lid.setdefault("dropout", 0.08)

    if intent.get("sensor_fault"):
        scn["taxonomy"]["sensor_fault"] = True
        lid = scn.setdefault("site_overrides", {}).setdefault("lidar", {})
        lid.setdefault("noise_sigma", 0.05)
        lid.setdefault("dropout", 0.15)
        lid.setdefault("latency_ms_max", 120)
        so_inj = scn.setdefault("site_overrides", {}).setdefault("injectors", {})
        blk = so_inj.setdefault("lidar_blackout", {})
        blk.setdefault("p", 0.60)
        blk.setdefault("duration_s", 5.0)
        blk.setdefault("sector_deg", 120.0)
        so_inj.setdefault("ghost_obstacle", {}).setdefault("spawn_rate_hz", 0.35)

    if intent.get("degraded_mode"):
        scn["taxonomy"]["sensor_fault"] = True
        scn["sensors"].setdefault("lidar", {})["hz"] = 5.0
        lid = scn.setdefault("site_overrides", {}).setdefault("lidar", {})
        lid.setdefault("latency_ms_max", 150)

    if "duration_s" in nums:
        scn["runtime"]["duration_s"] = int(nums["duration_s"])

    if hazards.get("vehicles") or len(hazards.get("human") or []) > 1:
        scn["taxonomy"]["multi_actor"] = True

    _align_vehicle_paths(scn)
    _ensure_default_human_paths(scn)
    _apply_human_motion_intent(scn, text, nums.get("human_rate_per_min"))

    if intent.get("no_humans_global"):
        humans = hazards.get("human", [])
        keep = [h for h in humans if h.get("raw_coords")]
        if len(keep) != len(humans):
            scn.setdefault("_debug", {})["humans_removed_no_humans"] = len(humans) - len(keep)
        hazards["human"] = keep
    if intent.get("no_vehicles_global"):
        vehicles = hazards.get("vehicles", [])
        keep_v = [v for v in vehicles if v.get("raw_coords")]
        if len(keep_v) != len(vehicles):
            scn.setdefault("_debug", {})["vehicles_removed_no_vehicles"] = len(vehicles) - len(keep_v)
        hazards["vehicles"] = keep_v

    if layout_obj.get("walls"):
        _carve_wall_gaps(layout_obj, pad=0.08)
        geom = layout_obj.setdefault("geometry", {})
        if not intent.get("wants_between_racks") and not intent.get("high_racks") and not coord_flags.get("rack", False):
            geom["racking"] = []

    _ensure_map_bounds(layout_obj, hazards)
    _retreat_goal_from_transitions(layout_obj)
    _ensure_start_goal_open(layout_obj, tuple(layout_obj["map_size_m"]))

    scn.setdefault("_debug", {})["parser_intent"] = {
        "intent": intent,
        "coord_flags": coord_detect,
        "counts": {
            "aisles": len(layout_obj.get("aisles") or []),
            "humans": len(hazards.get("human") or []),
            "vehicles": len(hazards.get("vehicles") or []),
            "traction": len(hazards.get("traction") or []),
        },
    }
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
        if len(path) >= 2:
            sx, sy = path[0][0], path[0][1]
            ex, ey = path[1][0], path[1][1]
            dx, dy = ex - sx, ey - sy
            if abs(dx) > 0.2 and abs(dy) > 0.2:
                if abs(dx) >= abs(dy):
                    path = [[sx, sy], [ex, sy]]
                else:
                    path = [[sx, sy], [sx, ey]]
                veh["path"] = path
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

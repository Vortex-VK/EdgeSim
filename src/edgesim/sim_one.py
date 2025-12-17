# EdgeSim V0 — sim_one.py (balanced)

from __future__ import annotations
from typing import Dict, Any, List, Tuple, Optional
from pathlib import Path
import csv, math, time, os, json
from collections import deque

import numpy as np
import pybullet as p

from .world import build_world, spawn_human
from .world_digest import build_world_digest, write_world_digest
from .injectors import (
    LidarSectorBlackoutInjector, FallingObjectInjector,
    GhostObstacleInjector, InjectorState, Injector
)

_HERE = Path(__file__).resolve().parent

# ---------- small I/O helpers ----------

def _write_csv(path: Path, header: List[str], rows: List[List[float | str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)

# ---------- geometry & controls ----------

def _spawn_disc(radius: float = 0.4, height: float | None = None, mass: float = 20.0, color=(0.9, 0.3, 0.3, 1.0)) -> int:
    h_use = max(0.18, radius * 0.6) if height is None else float(height)
    vis = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=h_use, rgbaColor=color)
    col = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=h_use)
    body = p.createMultiBody(
        baseMass=mass,
        baseCollisionShapeIndex=col,
        baseVisualShapeIndex=vis,
        basePosition=[1.0, 1.0, h_use / 2.0],
    )
    p.changeDynamics(body, -1, lateralFriction=0.8)
    return body

def _spawn_wet_patch(x0: float, y0: float, x1: float, y1: float, mu: float = 0.35, rgba=(0.2, 0.6, 1.0, 0.35)) -> int:
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    hx, hy = max(0.01, (x1 - x0) / 2.0), max(0.01, (y1 - y0) / 2.0)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[hx, hy, 0.001], rgbaColor=rgba)
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[hx, hy, 0.001])
    bid = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=col,
        baseVisualShapeIndex=vis,
        basePosition=[cx, cy, 0.001],
    )
    p.changeDynamics(bid, -1, lateralFriction=float(mu))
    return bid

def _aabb_distance_point(aabb: Tuple[float, float, float, float], x: float, y: float) -> float:
    x0, y0, x1, y1 = aabb
    dx = 0.0 if x0 <= x <= x1 else (x0 - x if x < x0 else x - x1)
    dy = 0.0 if y0 <= y <= y1 else (y0 - y if y < y0 else y - y1)
    return math.hypot(dx, dy)

def _clearance_to_aabbs(x: float, y: float, aabbs: List[Tuple[float, float, float, float]]) -> float:
    if not aabbs:
        return 10.0
    return min(_aabb_distance_point(a, x, y) for a in aabbs)

def _pid_follow(current_xy: Tuple[float, float], target_xy: Tuple[float, float], prev_err: np.ndarray,
                kp: float = 1.2, kd: float = 0.4, dt: float = 0.05) -> Tuple[np.ndarray, np.ndarray]:
    err = np.array([target_xy[0] - current_xy[0], target_xy[1] - current_xy[1]], dtype=float)
    derr = (err - prev_err) / max(1e-6, dt)
    ctrl = kp * err + kd * derr
    return ctrl, err

def _pt_in_aabb(x: float, y: float, aabb: Tuple[float, float, float, float] | None) -> bool:
    if aabb is None:
        return False
    x0, y0, x1, y1 = aabb
    return (x0 <= x <= x1) and (y0 <= y <= y1)

def _rect_overlap(a: Tuple[float, float, float, float],
                  b: Tuple[float, float, float, float]) -> Tuple[float, float, float, float] | None:
    x0 = max(a[0], b[0]); y0 = max(a[1], b[1])
    x1 = min(a[2], b[2]); y1 = min(a[3], b[3])
    if (x1 - x0) > 1e-4 and (y1 - y0) > 1e-4:
        return (x0, y0, x1, y1)
    return None

def _intervals_without(start: float, end: float,
                       cuts: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    intervals = [(start, end)]
    for (c0, c1) in cuts:
        c0 = max(start, min(end, c0))
        c1 = max(start, min(end, c1))
        if c1 <= c0:
            continue
        next_intervals: List[Tuple[float, float]] = []
        for (i0, i1) in intervals:
            if c1 <= i0 or c0 >= i1:
                next_intervals.append((i0, i1))
                continue
            if i0 < c0:
                next_intervals.append((i0, c0))
            if c1 < i1:
                next_intervals.append((c1, i1))
        intervals = next_intervals
    return intervals

def _resolve_spawn_point(x: float, y: float, radius: float,
                         bounds: Tuple[float, float], border: float,
                         blockers: List[Tuple[float, float, float, float]],
                         max_iter: int = 25) -> Tuple[float, float]:
    Lx, Ly = bounds
    for _ in range(max_iter):
        moved = False
        for (x0, y0, x1, y1) in blockers:
            closest_x = min(max(x, x0), x1)
            closest_y = min(max(y, y0), y1)
            dx = x - closest_x
            dy = y - closest_y
            dist = math.hypot(dx, dy)
            if dist >= radius or radius <= 0.0:
                continue
            moved = True
            if dist < 1e-6:
                cx = 0.5 * (x0 + x1)
                cy = 0.5 * (y0 + y1)
                dx = x - cx
                dy = y - cy
                norm = math.hypot(dx, dy)
                if norm < 1e-6:
                    dx, dy = 1.0, 0.0
                else:
                    dx /= norm
                    dy /= norm
                dist = 0.0
            else:
                dx /= dist
                dy /= dist
            push = (radius - dist) + 1e-3
            x += dx * push
            y += dy * push
            x = max(border, min(Lx - border, x))
            y = max(border, min(Ly - border, y))
        if not moved:
            break
    return x, y

def _rect_contains_pt(rect: Tuple[float, float, float, float], x: float, y: float) -> bool:
    x0, y0, x1, y1 = rect
    return (x0 <= x <= x1) and (y0 <= y <= y1)

def _project_point_to_rects(pt: Tuple[float, float], rects: List[Tuple[float, float, float, float]]) -> Tuple[float, float]:
    if not rects:
        return pt
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
    return best if best is not None else pt

def _project_path_into_rects(points: List[List[float]],
                             rects: List[Tuple[float, float, float, float]],
                             radius: float,
                             blockers: List[Tuple[float, float, float, float]],
                             bounds: Tuple[float, float],
                             border: float) -> List[List[float]]:
    if not rects or not points:
        return points
    out: List[List[float]] = []
    for pt in points:
        nx, ny = _project_point_to_rects((pt[0], pt[1]), rects)
        nx, ny = _resolve_spawn_point(nx, ny, max(radius, 0.0), bounds, border, blockers, max_iter=10)
        out.append([nx, ny])
    return out

def _vehicle_aabb(v: Dict[str, Any], x: float, y: float) -> Tuple[float, float, float, float]:
    half = v.get("half_extents") or [0.5, 0.4, 0.4]
    hx, hy = abs(half[0]), abs(half[1])
    return (x - hx, y - hy, x + hx, y + hy)

def _fallen_human_aabb(hid: int | None) -> Tuple[float,float,float,float] | None:
    if hid is None: return None
    try:
        a0, a1 = p.getAABB(hid)
        return (a0[0], a0[1], a1[0], a1[1])
    except Exception:
        return None

def _slip_prob_from_mu(mu: float) -> float:
    r = 2.2
    p = 1e-6 * (r ** ((0.40 - float(mu)) / 0.01))
    if p < 0.0:
        p = 0.0
    if p > 1.0:
        p = 1.0
    return p

def _fall_slide_offset(speed_mps: float, mu: float, heading: float, slip_boost: float = 0.0) -> Tuple[float, float]:
    # Simple kinetic friction slide distance: v^2 / (2 * mu * g), scaled so it is visible but bounded
    mu_eff = max(0.05, float(mu) - slip_boost * 0.15)
    g = 9.81
    s = max(0.0, float(speed_mps))
    if mu_eff <= 1e-4 or s <= 1e-4:
        dist = 0.0
    else:
        slip_gain = 1.0 + max(0.0, (0.5 - mu_eff)) * 2.0 + slip_boost * 2.5
        dist = (s * s) / (2.0 * mu_eff * g) * slip_gain
    dist = min(1.0, max(0.12, dist))
    return (math.cos(heading) * dist, math.sin(heading) * dist)


def _floor_mu_at(xr: float, yr: float,
                 floor_zones_meta: List[Dict[str, Any]],
                 dynamic_wet_patches: List[Dict[str, Any]],
                 wet_mu: float,
                 default_mu: float = 0.85) -> float:
    for surface in floor_zones_meta:
        zone = surface.get("zone")
        if zone and _pt_in_aabb(xr, yr, tuple(zone)):
            return float(surface.get("mu", default_mu))
    for patch in dynamic_wet_patches:
        aabb = patch.get("aabb")
        if aabb and _pt_in_aabb(xr, yr, tuple(aabb)):
            return float(patch.get("mu", wet_mu))
    return float(default_mu)

def _human_spawn_ok(start_xy: Tuple[float, float],
                    human_radius: float,
                    robot_xy: Tuple[float, float],
                    robot_radius: float,
                    vehicle_guards: List[Tuple[float, float, float]],
                    humans: List[Dict[str, Any]],
                    clearance: float = 0.15) -> bool:
    sx, sy = start_xy
    clear = max(0.0, clearance)
    if math.hypot(sx - robot_xy[0], sy - robot_xy[1]) < (robot_radius + human_radius + clear):
        return False
    for (vx, vy, vr) in vehicle_guards:
        if math.hypot(sx - vx, sy - vy) < (vr + human_radius + clear):
            return False
    for other in humans:
        other_r = float(other.get("radius", human_radius))
        if other.get("phase") == "running":
            last_pose = other.get("last_pose", (-999.0, -999.0))
            try:
                lx = float(last_pose[0])
                ly = float(last_pose[1])
            except Exception:
                continue
            if lx < -100 or ly < -100 or not math.isfinite(lx) or not math.isfinite(ly):
                continue
            if math.hypot(sx - lx, sy - ly) < (other_r + human_radius + clear):
                return False
        else:
            path = other.get("path") or []
            if not path or len(path) < 1:
                continue
            lx, ly = path[0]
            if math.hypot(sx - lx, sy - ly) < (other_r + human_radius + clear):
                return False
    return True

# ---------- human helpers ----------

_HUMAN_ROLE_PRESETS = {
    "picker": {"radius": 0.25, "length": 1.6, "speed": (0.9, 1.5), "behaviors": {"start_stop", "hesitation"}},
    "fast_walker": {"radius": 0.24, "length": 1.6, "speed": (1.6, 2.2), "behaviors": {"zig_zag"}},
    "distracted": {"radius": 0.25, "length": 1.6, "speed": (0.9, 1.5), "behaviors": {"late_react", "start_stop", "step_back"}},
    "carry": {"radius": 0.3, "length": 1.7, "speed": (0.7, 1.2), "behaviors": {"carry_payload"}},
    "child": {"radius": 0.18, "length": 1.0, "speed": (0.9, 1.4), "behaviors": {"zig_zag"}},
    "supervisor": {"radius": 0.25, "length": 1.6, "speed": (0.0, 0.0), "behaviors": {"stationary"}},
    "group": {"radius": 0.25, "length": 1.6, "speed": (0.9, 1.5), "behaviors": set()},
}

def _human_profile(cfg: Dict[str, Any]) -> Dict[str, Any]:
    role = (cfg.get("role") or cfg.get("type") or "picker").lower()
    profile = _HUMAN_ROLE_PRESETS.get(role, _HUMAN_ROLE_PRESETS["picker"]).copy()
    profile["radius"] = float(cfg.get("radius_m", profile["radius"]))
    profile["length"] = float(cfg.get("length_m", profile["length"]))
    scale = float(cfg.get("height_scale", 1.0))
    if not math.isfinite(scale):
        scale = 1.0
    scale = max(0.55, min(1.5, scale))
    profile["radius"] = max(0.14, profile["radius"] * scale)
    profile["length"] = max(0.6, profile["length"] * scale)
    speed_cfg = cfg.get("speed_mps", profile["speed"])
    if isinstance(speed_cfg, (list, tuple)) and len(speed_cfg) == 2:
        profile["speed"] = (float(speed_cfg[0]), float(speed_cfg[1]))
    behaviors = set(profile.get("behaviors", set()))
    for tag in (cfg.get("motion_patterns") or []):
        behaviors.add(tag)
    if cfg.get("push_cart"):
        behaviors.add("push_cart")
    if cfg.get("carry_payload"):
        behaviors.add("carry_payload")
    if cfg.get("zigzag"):
        behaviors.add("zig_zag")
    profile["behaviors"] = behaviors
    profile["height_scale"] = scale
    return profile

def _clamp_point(pt: List[float], bounds: Tuple[float, float], border: float) -> List[float]:
    return [
        max(border, min(bounds[0] - border, float(pt[0]))),
        max(border, min(bounds[1] - border, float(pt[1]))),
    ]

def _build_human_path(cfg: Dict[str, Any], bounds: Tuple[float, float], border: float) -> List[List[float]]:
    Lx, Ly = bounds
    raw = bool(cfg.get("raw_coords"))
    if cfg.get("waypoints"):
        pts = [_clamp_point(list(pt), bounds, border) for pt in cfg["waypoints"] if isinstance(pt, (list, tuple)) and len(pt) == 2]
        if len(pts) >= 2:
            return pts
    # Fallback only when no explicit waypoints were provided
    start_side = (cfg.get("start_side") or "south").lower()
    direction = -1 if start_side == "north" else 1
    cross_x = max(border, min(Lx - border, float(cfg.get("cross_x", 0.5 * Lx))))
    start_y = border if direction == 1 else (Ly - border)
    end_y = (Ly - border) if direction == 1 else border
    if abs(end_y - start_y) < 1e-3:
        end_y = max(border, min(Ly - border, start_y + (1.5 if direction == 1 else -1.5)))
    return [[cross_x, start_y], [cross_x, end_y]]

def _segments_from_points(points: List[List[float]]) -> Tuple[List[Dict[str, Any]], float]:
    segments: List[Dict[str, Any]] = []
    total = 0.0
    for idx in range(len(points) - 1):
        start = points[idx]
        end = points[idx + 1]
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.hypot(dx, dy)
        if length < 1e-4:
            continue
        segments.append({
            "start": start,
            "end": end,
            "len": length,
            "dir": (dx / length, dy / length),
        })
        total += length
    return segments, total

def _spawn_cart_body(width: float = 0.6, length: float = 1.0, height: float = 0.5, color=(0.3, 0.3, 0.3, 1.0)) -> Tuple[int, float]:
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[length/2, width/2, height/2], rgbaColor=color)
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[length/2, width/2, height/2])
    body = p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                             basePosition=[-20.0, -20.0, height/2])
    return body, height

def _spawn_payload_box(size: float = 0.35, height: float = 0.4, color=(0.7, 0.4, 0.2, 1.0)) -> Tuple[int, float]:
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[size/2, size/2, height/2], rgbaColor=color)
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size/2, size/2, height/2])
    body = p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                             basePosition=[-20.0, -20.0, height/2])
    return body, height

def _offset_path(points: List[List[float]], segments: List[Dict[str, Any]], offset: float,
                 bounds: Tuple[float, float], border: float) -> List[List[float]]:
    if not segments or abs(offset) < 1e-4:
        return points

    def _dir_for_idx(idx: int) -> Tuple[float, float]:
        if idx <= 0:
            return segments[0]["dir"]
        if idx >= len(segments):
            return segments[-1]["dir"]
        d_prev = segments[idx - 1]["dir"]
        d_next = segments[idx]["dir"]
        avg = (d_prev[0] + d_next[0], d_prev[1] + d_next[1])
        n = math.hypot(avg[0], avg[1])
        if n < 1e-6:
            return d_prev
        return (avg[0] / n, avg[1] / n)

    out: List[List[float]] = []
    for idx, pt in enumerate(points):
        dir_vec = _dir_for_idx(idx if idx < len(segments) else len(segments) - 1)
        perp = (-dir_vec[1], dir_vec[0])
        out.append(_clamp_point([pt[0] + perp[0] * offset, pt[1] + perp[1] * offset], bounds, border))
    return out

def _update_human_pose(body_id: int, xh: float, yh: float, z_base: float, heading: float, posture_scale: float = 1.0) -> None:
    z = max(0.1, z_base * posture_scale)
    p.resetBasePositionAndOrientation(body_id, [xh, yh, z], p.getQuaternionFromEuler([0.0, 0.0, heading]))

def _update_human_props(state: Dict[str, Any], xh: float, yh: float, heading: float) -> None:
    props = state.get("props") or {}
    if not props:
        return
    ch = math.cos(heading)
    sh = math.sin(heading)
    if "cart" in props and props["cart"].get("id") is not None:
        cart = props["cart"]
        offset = float(cart.get("offset", 0.9))
        px = xh + ch * offset
        py = yh + sh * offset
        p.resetBasePositionAndOrientation(cart["id"], [px, py, cart["height"] / 2.0], p.getQuaternionFromEuler([0.0, 0.0, heading]))
    if "payload" in props:
        payload = props["payload"]
        if payload.get("id") is not None and not payload.get("dropped", False):
            p.resetBasePositionAndOrientation(payload["id"], [xh, yh, payload["height"] / 2.0], p.getQuaternionFromEuler([0.0, 0.0, heading]))

_BASE_PATH_AISLE_WIDTH = 1.25
_FORKLIFT_WIDTH_SCALE = 1.3
_HUMAN_AISLE_WIDTH = 1.0
_EXTRA_VEHICLE_SCALE = 1.1

def _segment_rect(start: List[float], end: List[float],
                  width: float, bounds: Tuple[float, float]) -> List[float] | None:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    if length < 1e-4:
        return None
    pad = max(0.25, width * 0.5)
    perp = (-dy / length, dx / length)
    span_x = pad * abs(perp[0])
    span_y = pad * abs(perp[1])
    x0 = min(start[0], end[0]) - span_x
    x1 = max(start[0], end[0]) + span_x
    y0 = min(start[1], end[1]) - span_y
    y1 = max(start[1], end[1]) + span_y
    Lx, Ly = bounds
    x0 = max(0.0, min(Lx, x0))
    x1 = max(0.0, min(Lx, x1))
    y0 = max(0.0, min(Ly, y0))
    y1 = max(0.0, min(Ly, y1))
    if (x1 - x0) < 0.2:
        mid = 0.5 * (x0 + x1)
        x0 = max(0.0, mid - 0.1)
        x1 = min(Lx, mid + 0.1)
    if (y1 - y0) < 0.2:
        mid = 0.5 * (y0 + y1)
        y0 = max(0.0, mid - 0.1)
        y1 = min(Ly, mid + 0.1)
    return [x0, y0, x1, y1]

def _apply_path_defined_aisles(prompt: str | None,
                               scn: Dict[str, Any],
                               border: float = 0.6) -> None:
    layout = scn.get("layout") or {}
    if layout.get("lock_aisles"):
        return
    if layout.get("_path_aisles_applied"):
        return
    requested = bool(layout.get("auto_aisles_from_paths") or
                     layout.get("paths_define_aisles") or
                     layout.get("narrow_aisle_hint"))
    existing = layout.get("aisles")
    if isinstance(existing, list) and len(existing) > 0:
        return
    if not requested and isinstance(prompt, str):
        prompt_l = prompt.lower()
        if "aisle" in prompt_l and "no aisle" not in prompt_l and "without aisle" not in prompt_l:
            requested = True
    if not requested:
        return
    map_size = layout.get("map_size_m") or [20.0, 20.0]
    if not isinstance(map_size, (list, tuple)) or len(map_size) < 2:
        map_size = [20.0, 20.0]
    bounds = (float(map_size[0]), float(map_size[1]))
    hazards = scn.get("hazards", {}) or {}
    if not isinstance(existing, list):
        existing = []
        layout["aisles"] = existing
    entry_offset = len(existing)
    new_entries: List[Dict[str, Any]] = []

    def _clamped_points(seq: List[Any]) -> List[List[float]]:
        points: List[List[float]] = []
        for pt in seq:
            if not isinstance(pt, (list, tuple)) or len(pt) != 2:
                continue
            points.append(_clamp_point([float(pt[0]), float(pt[1])], bounds, border))
        return points

    def _add_path(points: List[List[float]], width: float, tag: str, *, high_bay: bool = False) -> None:
        if len(points) < 2:
            return
        segs, _ = _segments_from_points(points)
        for seg in segs:
            zone = _segment_rect(seg["start"], seg["end"], width, bounds)
            if zone is None:
                continue
            entry_id = f"PathAisle_{entry_offset + len(new_entries) + 1:02d}"
            new_entries.append({
                "id": entry_id,
                "name": f"{tag}_lane",
                "rect": zone,
                "type": "straight",
                "racking": False,
                "high_bay": high_bay,
                "width_hint_m": width,
            })

    start = layout.get("start")
    goal = layout.get("goal")
    if isinstance(start, (list, tuple)) and isinstance(goal, (list, tuple)):
        path_pts = [_clamp_point(list(start), bounds, border)]
        agent_waypoints: List[Any] = []
        if scn.get("agents"):
            wps = scn["agents"][0].get("waypoints")
            if isinstance(wps, list):
                agent_waypoints = wps
        path_pts.extend(_clamped_points(agent_waypoints))
        path_pts.append(_clamp_point(list(goal), bounds, border))
        _add_path(path_pts, _BASE_PATH_AISLE_WIDTH, "robot", high_bay=False)

    for veh in hazards.get("vehicles", []):
        if veh.get("raw_coords"):
            continue  # honor raw coordinate paths; do not infer aisles
        path = _clamped_points(veh.get("path") or [])
        if len(path) < 2:
            continue
        vtype = (veh.get("type") or "vehicle").lower()
        if vtype == "forklift":
            width = _BASE_PATH_AISLE_WIDTH * _FORKLIFT_WIDTH_SCALE
            _add_path(path, width, vtype, high_bay=True)
        else:
            width = _BASE_PATH_AISLE_WIDTH * _EXTRA_VEHICLE_SCALE
            _add_path(path, width, vtype or "vehicle", high_bay=False)

    for idx, cfg in enumerate(hazards.get("human", [])):
        path = _build_human_path(cfg, bounds, border)
        if len(path) >= 2:
            _add_path(path, _HUMAN_AISLE_WIDTH, f"human_{idx+1}", high_bay=False)

    if new_entries:
        existing.extend(new_entries)
    layout["_path_aisles_applied"] = True

def _apply_dynamic_reflections(ranges: np.ndarray, robot_xy: Tuple[float, float],
                               vehicle_states: List[Dict[str, Any]], rng: np.random.Generator) -> np.ndarray:
    if not vehicle_states:
        return ranges
    mod = ranges
    cloned = False
    for v in vehicle_states:
        if not v.get("reflective"):
            continue
        pose = v.get("last_pose")
        if not pose:
            continue
        dist = math.hypot(pose[0] - robot_xy[0], pose[1] - robot_xy[1])
        if dist > 3.0:
            continue
        count = max(1, int(round(3.0 / max(dist, 0.3))))
        idx = rng.choice(len(mod), size=count, replace=False)
        if not cloned:
            mod = mod.copy()
            cloned = True
        mod[idx] = np.clip(mod[idx] * rng.uniform(0.1, 0.4, size=count), 0.05, mod[idx])
    return mod

# ---------- Site profile loader ----------

def _load_site_tuned() -> Dict[str, Any]:
    site = os.environ.get("EDGESIM_SITE", "").strip()
    if not site:
        return {}
    path = _HERE / "site_profiles" / f"{site}.json"
    if not path.exists():
        return {}
    try:
        with path.open("r", encoding="utf-8") as f:
            obj = json.load(f)
        return obj.get("tuned", {})
    except Exception:
        return {}

# ---------- Sensors ----------

def _child_rng(rng: np.random.Generator | None) -> np.random.Generator:
    if rng is None:
        return np.random.default_rng()
    return np.random.default_rng(rng.integers(2**63 - 1))


class _LidarSim:
    def __init__(self, beams: int = 360, hz: float = 10.0, max_range: float = 8.0,
                 dropout_pct_range=(0.01, 0.02), noise_sigma: float = 0.02,
                 jitter_ms_max: int = 50, fog_density: float = 0.0,
                 reflective_zones: List[Dict[str, Any]] | None = None,
                 rng: np.random.Generator | None = None):
        self.beams = int(beams)
        self.hz = float(hz)
        self.dt = 1.0 / max(1e-6, self.hz)
        self.max_range = float(max_range)
        self.dropout_low, self.dropout_high = float(dropout_pct_range[0]), float(dropout_pct_range[1])
        self.noise_sigma = float(noise_sigma)
        self.next_update_t = 0.0
        self.angles = np.linspace(-math.pi, math.pi, self.beams, endpoint=False)
        self._rng = _child_rng(rng)
        self._queue: List[Tuple[float, np.ndarray]] = []
        self._jitter_s = float(jitter_ms_max) / 1000.0
        self.fog_density = max(0.0, min(1.0, float(fog_density)))
        self.reflective_zones = reflective_zones or []

    def maybe_update(self, t: float, x: float, y: float, yaw: float, sensor_z: float, ignore_ids: set[int]) -> None:
        if t < self.next_update_t:
            return
        self.next_update_t = t + self.dt

        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        dir_x = np.cos(self.angles); dir_y = np.sin(self.angles)
        rot_x = cos_yaw * dir_x - sin_yaw * dir_y
        rot_y = sin_yaw * dir_x + cos_yaw * dir_y

        origins = [[x, y, sensor_z]] * self.beams
        ends = [[x + float(rx) * self.max_range, y + float(ry) * self.max_range, sensor_z] for rx, ry in zip(rot_x, rot_y)]

        try:
            results = p.rayTestBatch(origins, ends)
        except TypeError:
            results = []
            for ro, re in zip(origins, ends):
                r = p.rayTest(ro, re)
                results.append(r[0] if r else (-1, -1, 1.0, [0, 0, 0]))

        dists = np.empty(self.beams, dtype=float)
        for i, r in enumerate(results):
            hit_uid = r[0]
            hit_frac = r[2]
            d = self.max_range if (hit_uid in ignore_ids or hit_uid == -1) else float(hit_frac) * self.max_range
            dists[i] = d

        if self.fog_density > 0.0:
            fog_sigma = self.noise_sigma + self.fog_density * 0.08
            dists = np.clip(dists + self._rng.normal(0.0, fog_sigma, size=self.beams), 0.0, self.max_range * (1.0 - 0.35 * self.fog_density))

        dropout_pct = float(self._rng.uniform(self.dropout_low, self.dropout_high + self.fog_density * 0.1))
        if dropout_pct > 0.0:
            k = max(1, int(round(dropout_pct * self.beams)))
            idx = self._rng.choice(self.beams, size=k, replace=False)
            dists[idx] = self.max_range
        if self.noise_sigma > 0:
            dists = np.clip(dists + self._rng.normal(0.0, self.noise_sigma, size=self.beams), 0.0, self.max_range)

        if self.reflective_zones:
            for zone in self.reflective_zones:
                aabb = zone.get("aabb")
                if not aabb:
                    continue
                dist = _aabb_distance_point(tuple(aabb), x, y)
                trigger = float(zone.get("trigger", 1.5))
                if dist <= trigger:
                    strength = float(zone.get("strength", 0.5))
                    count = max(1, int(round(strength * 4)))
                    idx = self._rng.choice(self.beams, size=count, replace=False)
                    multipath = dists[idx] * self._rng.uniform(0.15, 0.5, size=count)
                    dists[idx] = np.clip(multipath, 0.05, self.max_range)

        avail_t = t + float(self._rng.uniform(0.0, self._jitter_s))
        self._queue.append((avail_t, dists))

    def try_pop_ready(self, t: float) -> np.ndarray | None:
        if not self._queue:
            return None
        self._queue.sort(key=lambda x: x[0])
        if self._queue[0][0] <= t:
            return self._queue.pop(0)[1]
        return None


class _LidarLogger:
    def __init__(self, angles: np.ndarray, enabled: bool, log_xyz: bool, events_only: bool,
                 pre_event_s: float, post_event_s: float, dt: float):
        self.enabled = bool(enabled)
        self.log_xyz = bool(log_xyz)
        self.events_only = bool(events_only)
        self.angles = np.asarray(angles, dtype=np.float32) if angles is not None else np.array([], dtype=np.float32)
        self.dt = float(dt)
        self.pre_steps = int(max(0, round(pre_event_s / max(1e-6, self.dt))))
        self.post_steps = int(max(0, round(post_event_s / max(1e-6, self.dt))))
        # keep a short buffer so we can dump a window around interesting events
        self._buffer: deque[Tuple[float, Tuple[float, float, float], np.ndarray]] = deque(maxlen=max(1, self.pre_steps))
        self._frames: List[Tuple[float, Tuple[float, float, float], np.ndarray]] = []
        self._active_steps = 0

    def record(self, t: float, pose: Tuple[float, float, float], ranges: np.ndarray | None) -> None:
        if (not self.enabled) or ranges is None:
            return
        scan = np.asarray(ranges, dtype=np.float32)
        frame = (float(t), (float(pose[0]), float(pose[1]), float(pose[2])), scan.copy())
        if self.events_only:
            self._buffer.append(frame)
            if self._active_steps > 0:
                self._frames.append(frame)
                self._active_steps -= 1
        else:
            self._frames.append(frame)

    def mark_event(self) -> None:
        if (not self.enabled) or (not self.events_only):
            return
        if self._buffer:
            self._frames.extend(list(self._buffer))
            self._buffer.clear()
        self._active_steps = max(self._active_steps, self.post_steps if self.post_steps > 0 else 1)

    def write(self, path: Path) -> None:
        if (not self.enabled) or (not self._frames):
            return
        t = np.array([f[0] for f in self._frames], dtype=np.float32)
        pose = np.array([f[1] for f in self._frames], dtype=np.float32)
        ranges = np.stack([f[2] for f in self._frames]).astype(np.float32)
        obj: Dict[str, Any] = {
            "t": t,
            "pose": pose,
            "ranges": ranges,
            "angles": self.angles,
        }
        if self.log_xyz and len(self.angles) and len(self._frames):
            points: List[np.ndarray] = []
            for scan in ranges:
                x = scan * np.cos(self.angles)
                y = scan * np.sin(self.angles)
                z = np.zeros_like(x, dtype=np.float32)
                points.append(np.stack([x, y, z], axis=1))
            obj["xyz"] = np.stack(points).astype(np.float32)
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(path, **obj)


class _OdomSim:
    def __init__(self, bias_v: float = 0.02, bias_w: float = 0.01, noise_v: float = 0.02, noise_w: float = 0.01,
                 rng: np.random.Generator | None = None):
        self.bias_v = float(bias_v); self.bias_w = float(bias_w)
        self.noise_v = float(noise_v); self.noise_w = float(noise_w)
        self.rng = _child_rng(rng)

    def apply(self, v_true: float, w_true: float, slip_factor: float = 0.0) -> Tuple[float, float]:
        v = (v_true * (1.0 + self.bias_v)) + float(self.rng.normal(0.0, self.noise_v + slip_factor * 0.05))
        w = (w_true * (1.0 + self.bias_w)) + float(self.rng.normal(0.0, self.noise_w + slip_factor * 0.03))
        return v, w

class _IMUSim:
    def __init__(self, accel_noise: float = 0.02, gyro_noise: float = 0.015, rng: np.random.Generator | None = None):
        self.accel_noise = float(accel_noise)
        self.gyro_noise = float(gyro_noise)
        self.prev_v = 0.0
        self.prev_yaw = 0.0
        self.rng = _child_rng(rng)

    def measure(self, v_cur: float, yaw: float, dt: float, surface_effects: Dict[str, Any] | None = None) -> Tuple[float, float, float]:
        vib = float((surface_effects or {}).get("imu_vibe_g", 0.0))
        ax = (v_cur - self.prev_v) / max(1e-6, dt)
        ax += float(self.rng.normal(0.0, self.accel_noise + vib * 0.05))
        ay = float(self.rng.normal(0.0, (self.accel_noise * 0.8) + vib * 0.03))
        yaw_rate = (yaw - self.prev_yaw) / max(1e-6, dt)
        gz = yaw_rate + float(self.rng.normal(0.0, self.gyro_noise + vib * 0.02))
        self.prev_v = v_cur
        self.prev_yaw = yaw
        return ax, ay, gz

# ---------- main single-run ----------

def run_one(
    prompt: str,
    scn: Dict[str, Any],
    out_dir: Path,
    dt: float = 0.05,
    realtime: bool = False,
    gui: bool = False,
    sleep_scale: float = 1.0,
) -> Dict[str, Any]:

    if "runtime" in scn and "dt" in scn["runtime"]:
        try:
            dt = float(scn["runtime"]["dt"])
        except Exception:
            pass

    _apply_path_defined_aisles(prompt, scn)
    env = build_world(scn, use_gui=gui)
    # Keep Bullet's timestep aligned with our integration step
    try:
        p.setTimeStep(dt)
    except Exception:
        pass
    client_id = env["client"]
    try:
        Lx, Ly = env["bounds"]
        plane_id = env["plane_id"]
        wall_ids = list(env.get("walls", []))
        patch_ids = list(env.get("patches", []))
        ignored_for_collision = {plane_id, *patch_ids}
        floor_zones_meta: List[Dict[str, Any]] = list(env.get("floor_zones", []))
        transition_zones_meta: List[Dict[str, Any]] = list(env.get("transition_zones", []))
        vehicle_meta: List[Dict[str, Any]] = list(env.get("vehicles_meta", []))
        vehicle_body_ids: set[int] = set()
        for v in vehicle_meta:
            if not isinstance(v, dict):
                continue
            bid = v.get("body_id")
            if bid is not None:
                vehicle_body_ids.add(bid)

        # Static geometry bounds for spawn checks
        static_aabbs: List[Tuple[float, float, float, float]] = []
        for wid in wall_ids:
            if wid in vehicle_body_ids:
                continue
            (a0, a1) = p.getAABB(wid)
            static_aabbs.append((a0[0], a0[1], a1[0], a1[1]))
        for sobj in env.get("static_obstacles", []) or []:
            if not isinstance(sobj, dict):
                continue
            body_id = sobj.get("body_id")
            if body_id in vehicle_body_ids:
                continue
            aabb = None
            if isinstance(sobj.get("aabb"), (list, tuple)) and len(sobj["aabb"]) == 4:
                aabb = sobj["aabb"]
            elif body_id is not None:
                try:
                    a0, a1 = p.getAABB(body_id)
                    aabb = (a0[0], a0[1], a1[0], a1[1])
                except Exception:
                    aabb = None
            if aabb:
                static_aabbs.append((float(aabb[0]), float(aabb[1]), float(aabb[2]), float(aabb[3])))
        static_clearance_body_ids: list[int] = [wid for wid in wall_ids if wid not in vehicle_body_ids]
        for sobj in env.get("static_obstacles", []) or []:
            bid = sobj.get("body_id")
            if isinstance(bid, int) and bid not in vehicle_body_ids:
                static_clearance_body_ids.append(bid)
        walkway_rects: List[Tuple[float, float, float, float]] = []
        for meta in env.get("aisles_meta", []):
            if not isinstance(meta, dict):
                continue
            zone = meta.get("zone")
            if isinstance(zone, (list, tuple)) and len(zone) == 4:
                walkway_rects.append((float(zone[0]), float(zone[1]), float(zone[2]), float(zone[3])))

        # Fallback: if static_aabbs somehow stayed empty, backfill from env metadata to keep clearance valid
        if not static_aabbs:
            for sobj in env.get("static_obstacles", []) or []:
                if not isinstance(sobj, dict):
                    continue
                aa = sobj.get("aabb")
                if isinstance(aa, (list, tuple)) and len(aa) == 4:
                    static_aabbs.append((float(aa[0]), float(aa[1]), float(aa[2]), float(aa[3])))
            if not static_aabbs:
                for wid in wall_ids:
                    if wid in vehicle_body_ids:
                        continue
                    try:
                        a0, a1 = p.getAABB(wid)
                        static_aabbs.append((a0[0], a0[1], a1[0], a1[1]))
                    except Exception:
                        continue
        # Also fold in layout-defined AABBs so clearance never loses fixed geometry
        layout = scn.get("layout", {}) or {}
        for wall in layout.get("walls", []) or []:
            aa = wall.get("aabb")
            if isinstance(aa, (list, tuple)) and len(aa) == 4:
                static_aabbs.append((float(aa[0]), float(aa[1]), float(aa[2]), float(aa[3])))
        for obs in layout.get("static_obstacles", []) or []:
            aa = obs.get("aabb")
            if isinstance(aa, (list, tuple)) and len(aa) == 4:
                static_aabbs.append((float(aa[0]), float(aa[1]), float(aa[2]), float(aa[3])))
        for rack in (layout.get("geometry", {}) or {}).get("racking", []) or []:
            aa = rack.get("aabb")
            if isinstance(aa, (list, tuple)) and len(aa) == 4:
                static_aabbs.append((float(aa[0]), float(aa[1]), float(aa[2]), float(aa[3])))
        geom_clearance_aabbs = list(static_aabbs)

        _tuned = _load_site_tuned()
        if not isinstance(_tuned, dict):
            _tuned = {}

        # Merge scenario-local site overrides (from parser) on top of site profile
        site_over = scn.get("site_overrides", {})

        def _deep_merge(dst: Dict[str, Any], src: Dict[str, Any]) -> None:
            for k, v in src.items():
                if isinstance(v, dict) and isinstance(dst.get(k), dict):
                    _deep_merge(dst[k], v)  # type: ignore[arg-type]
                else:
                    dst[k] = v

        if isinstance(site_over, dict):
            _deep_merge(_tuned, site_over)

        hazards = scn.get("hazards", {}) or {}

        # ---- tuned knobs / defaults ----
        tr_cfg = (_tuned.get("traction") or {})
        mu_wet_min = float(tr_cfg.get("mu_wet_min", 0.35))
        mu_wet_max = float(tr_cfg.get("mu_wet_max", 0.60))
        jitter_mu_default = bool(tr_cfg.get("jitter_mu", False))

        h_cfg = (_tuned.get("human") or {})
        tuned_slip_prob = float(h_cfg.get("slip_prob", 0.85))               # moderate default
        slip_min_exposure_s = float(h_cfg.get("slip_min_exposure_s", 0.06))  # brief exposure
        tuned_fall_duration = float(h_cfg.get("fall_duration_s", 6.0))
        taxonomy = scn.get("taxonomy", {}) or {}
        if taxonomy.get("occlusion"):
            tuned_slip_prob = min(0.7, tuned_slip_prob)
            slip_min_exposure_s = max(0.25, slip_min_exposure_s * 3.0)

        l_cfg = (_tuned.get("lidar") or {})
        tuned_lidar_noise = float(l_cfg.get("noise_sigma", 0.02))
        tuned_lidar_dropout = float(l_cfg.get("dropout", 0.01))
        tuned_lidar_jitter = int(l_cfg.get("latency_ms_max", 50))

        o_cfg = (_tuned.get("odom") or {})
        tuned_bias_v = float(o_cfg.get("bias_v", 0.02))
        tuned_bias_w = float(o_cfg.get("bias_w", 0.01))

        # per-run rng (salted by run id)
        seed_val = int(scn.get("seed", 0))
        try:
            run_salt = int(''.join(ch for ch in out_dir.name if ch.isdigit()) or "0")
        except Exception:
            run_salt = 0
        seed_val = (seed_val * 1664525 + 1013904223 + run_salt) & 0xFFFFFFFF
        rng = np.random.default_rng(seed_val)

        dynamic_wet_patches: List[Dict[str, Any]] = []

        if gui:
            p.resetDebugVisualizerCamera(
                cameraDistance=10.0,
                cameraYaw=45.0,
                cameraPitch=-35.0,
                cameraTargetPosition=[0.5 * Lx, 0.5 * Ly, 0.0],
            )

        # sample a single μ_wet for this run and apply to existing patches (only if jitter is requested)
        wet_mu = float(rng.uniform(mu_wet_min, mu_wet_max))
        wet_like_types = {"wet", "oil", "cleaning_liquid", "smooth_plastic"}
        for surface in floor_zones_meta:
            if not isinstance(surface, dict):
                continue
            s_type = (surface.get("type") or "").lower()
            effects = surface.get("effects") or {}
            is_wet = (s_type in wet_like_types) or (effects.get("slip_boost", 0.0) > 0.2)
            if not is_wet:
                continue
            if not bool(surface.get("jitter_mu", jitter_mu_default)):
                continue
            mu_new = float(rng.uniform(mu_wet_min, mu_wet_max))
            bid = surface.get("body_id")
            if bid is not None:
                p.changeDynamics(bid, -1, lateralFriction=mu_new)
            surface["mu"] = mu_new

        agents_list = scn.get("agents") or [{}]
        agent_cfg = agents_list[0]
        radius = float(agent_cfg.get("radius_m", 0.4))
        robot_height = max(0.18, radius * 0.6)
        amr = _spawn_disc(radius=radius, height=robot_height)

        border = 0.6
        start = list(map(float, scn.get("layout", {}).get("start", [border, border])))
        goal = list(map(float, scn.get("layout", {}).get("goal", [Lx - border, Ly - border])))
        start[0] = max(border, min(Lx - border, start[0]))
        start[1] = max(border, min(Ly - border, start[1]))
        goal[0] = max(border, min(Lx - border, goal[0]))
        goal[1] = max(border, min(Ly - border, goal[1]))
        raw_coord_aisles = bool(scn.get("layout", {}).get("_raw_coord_aisles"))
        if walkway_rects and not raw_coord_aisles and scn.get("layout", {}).get("snap_start_goal_to_walkways", False):
            start[0], start[1] = _project_point_to_rects((start[0], start[1]), walkway_rects)
            goal[0], goal[1] = _project_point_to_rects((goal[0], goal[1]), walkway_rects)
        if not raw_coord_aisles:
            vehicle_spawn_blockers: List[Tuple[float, float, float, float]] = []
            for vmeta in vehicle_meta:
                path_pts = vmeta.get("path") or []
                if not path_pts:
                    continue
                half_ext = vmeta.get("half_extents") or [0.5, 0.4, 0.4]
                hx = abs(half_ext[0]) + 0.1
                hy = abs(half_ext[1]) + 0.1
                vx, vy = path_pts[0]
                vehicle_spawn_blockers.append((vx - hx, vy - hy, vx + hx, vy + hy))
            start[0], start[1] = _resolve_spawn_point(
                start[0], start[1], radius + 0.05, (Lx, Ly), border, static_aabbs + vehicle_spawn_blockers
            )
        if static_aabbs:
            if any(_rect_contains_pt(aabb, start[0], start[1]) for aabb in static_aabbs):
                raise ValueError(f"Robot start {start} lies inside static geometry")
            clearance = _clearance_to_aabbs(start[0], start[1], static_aabbs)
            if clearance < radius + 0.1:
                raise ValueError(f"Robot start {start} too close to geometry (clearance {clearance:.2f} m)")
            if any(_rect_contains_pt(aabb, goal[0], goal[1]) for aabb in static_aabbs):
                raise ValueError(f"Robot goal {goal} lies inside static geometry")
            min_goal_clear = _clearance_to_aabbs(goal[0], goal[1], static_aabbs)
            if min_goal_clear < radius + 0.1:
                raise ValueError(f"Robot goal {goal} too close to geometry (clearance {min_goal_clear:.2f} m)")
        if walkway_rects and not any(_rect_contains_pt(w, start[0], start[1]) for w in walkway_rects):
            print(f"[geometry] start {start} not inside any aisle/junction; continuing with nearest projection")
        dx = goal[0] - start[0]
        dy = goal[1] - start[1]
        yaw0 = math.pi / 2 if abs(dy) > abs(dx) and dy >= 0 else (-math.pi / 2 if abs(dy) > abs(dx) else (0.0 if dx >= 0 else math.pi))
        p.resetBasePositionAndOrientation(amr, [start[0], start[1], robot_height * 0.5], p.getQuaternionFromEuler([0, 0, yaw0]))

        moving_blockers: List[Tuple[float, float, float, float]] = []
        moving_blockers.append((start[0] - radius - 0.1, start[1] - radius - 0.1, start[0] + radius + 0.1, start[1] + radius + 0.1))
        for vmeta in vehicle_meta:
            path_pts = vmeta.get("path") or []
            if not path_pts:
                continue
            half_ext = vmeta.get("half_extents") or [0.5, 0.4, 0.4]
            hx = abs(half_ext[0]) + 0.15
            hy = abs(half_ext[1]) + 0.15
            vx, vy = path_pts[0]
            moving_blockers.append((vx - hx, vy - hy, vx + hx, vy + hy))

        # Optional wet corridor (not every run)
        ensure_corr_p = float(_tuned.get("ensure_wet_corridor_pct", 0.0))
        corr_w = float(_tuned.get("ensure_wet_corridor_width_m", 1.2))
        explicit_traction = bool(scn.get("hazards", {}).get("traction"))
        if ensure_corr_p > 0.0 and not explicit_traction and rng.random() < ensure_corr_p:
            midx = 0.5 * Lx
            x0, x1 = max(0.0, midx - 0.5 * corr_w), min(Lx, midx + 0.5 * corr_w)
            y0, y1 = 0.0 + 0.6, Ly - 0.6
            corr_id = _spawn_wet_patch(x0, y0, x1, y1, mu=wet_mu)
            patch_ids.append(corr_id)
            ignored_for_collision.add(corr_id)
            zone_corr = (x0, y0, x1, y1)
            dynamic_wet_patches.append({"aabb": zone_corr, "mu": wet_mu})
            floor_zones_meta.append({
                "id": "wet_corridor_auto",
                "type": "wet",
                "zone": list(zone_corr),
                "mu": wet_mu,
                "body_id": corr_id,
                "effects": {"brake_scale": 1.4, "slip_boost": 0.6},
            })

        # Human hazard (multiple actors)
        human_cfgs = [cfg for cfg in (scn.get("hazards", {}).get("human") or []) if isinstance(cfg, dict)]
        human_states: List[Dict[str, Any]] = []

        def _scenario_has_wet_surface() -> bool:
            for surface in floor_zones_meta:
                s_type = (surface.get("type") or "").lower()
                effects = surface.get("effects") or {}
                if (s_type in wet_like_types) or effects.get("slip_boost", 0.0) > 0.2:
                    return True
            if scn.get("hazards", {}).get("traction"):
                return True
            if dynamic_wet_patches:
                return True
            return False

        for cfg in human_cfgs:
            profile = _human_profile(cfg)
            base_path = _build_human_path(cfg, (Lx, Ly), border)
            if walkway_rects and not cfg.get("raw_coords"):
                base_path = _project_path_into_rects(base_path, walkway_rects, profile["radius"], static_aabbs, (Lx, Ly), border)
            base_segments, _ = _segments_from_points(base_path)
            base_slip = float(cfg.get("p_slip", tuned_slip_prob))
            has_wet_surface = _scenario_has_wet_surface()
            use_local_wet = bool(cfg.get("force_wet", False))
            if taxonomy.get("occlusion"):
                base_slip = min(0.75, base_slip)
                cfg.setdefault("start_delay_s", 0.2 + rng.uniform(0.2, 0.6))
                cfg.setdefault("slip_min_exposure_s", max(0.35, slip_min_exposure_s))
            if not use_local_wet:
                if not has_wet_surface:
                    base_slip = min(0.05, base_slip * 0.2)
                    cfg["slip_min_exposure_s"] = max(cfg.get("slip_min_exposure_s", slip_min_exposure_s), 0.6)
            cfg["p_slip"] = base_slip
            local_wet_aabb = None
            if use_local_wet and base_path:
                xs = [pt[0] for pt in base_path]
                ys = [pt[1] for pt in base_path]
                wet_zone = [
                    max(0.0, min(xs) - 1.0),
                    max(0.0, min(ys) - 0.5),
                    min(Lx, max(xs) + 1.0),
                    min(Ly, max(ys) + 0.5),
                ]
                wet_id = _spawn_wet_patch(wet_zone[0], wet_zone[1], wet_zone[2], wet_zone[3], mu=wet_mu)
                patch_ids.append(wet_id)
                ignored_for_collision.add(wet_id)
                local_wet_aabb = tuple(wet_zone)
                dynamic_wet_patches.append({"aabb": local_wet_aabb, "mu": wet_mu})
                floor_zones_meta.append({
                    "id": cfg.get("id") or f"human_wet_{len(floor_zones_meta)+1:02d}",
                    "type": cfg.get("type", "wet"),
                    "zone": wet_zone,
                    "mu": wet_mu,
                    "body_id": wet_id,
                    "effects": {"brake_scale": 1.3, "slip_boost": 0.6},
                })
            if isinstance(cfg.get("group_size_range"), (list, tuple)) and len(cfg["group_size_range"]) == 2:
                a, b = cfg["group_size_range"]
                try:
                    group_size = int(rng.integers(min(int(a), int(b)), max(int(a), int(b)) + 1))
                except Exception:
                    group_size = max(1, int(cfg.get("group_size", 1)))
            else:
                group_size = max(1, int(cfg.get("group_size", 1)))
            offset_step = float(cfg.get("group_spacing_m", 0.6))
            for member_idx in range(group_size):
                offset = (member_idx - 0.5 * (group_size - 1)) * offset_step
                member_path = _offset_path(base_path, base_segments, offset, (Lx, Ly), border)
                if walkway_rects and not cfg.get("raw_coords"):
                    member_path = _project_path_into_rects(member_path, walkway_rects, profile["radius"], static_aabbs, (Lx, Ly), border)
                member_segments, _ = _segments_from_points(member_path)
                if member_path:
                    safe_x, safe_y = _resolve_spawn_point(member_path[0][0], member_path[0][1],
                                                          profile["radius"] + 0.05, (Lx, Ly), border,
                                                          moving_blockers + static_aabbs, max_iter=20) if not cfg.get("raw_coords") else (member_path[0][0], member_path[0][1])
                    dx, dy = safe_x - member_path[0][0], safe_y - member_path[0][1]
                    if abs(dx) > 1e-3 or abs(dy) > 1e-3:
                        member_path = [_clamp_point([pt[0] + dx, pt[1] + dy], (Lx, Ly), border) for pt in member_path]
                        member_segments, _ = _segments_from_points(member_path)
                    moving_blockers.append((safe_x - profile["radius"], safe_y - profile["radius"],
                                            safe_x + profile["radius"], safe_y + profile["radius"]))
                delay_min = float(cfg.get("start_delay_s_min", 0.4))
                delay_max = float(cfg.get("start_delay_s_max", 1.2))
                if delay_max < delay_min:
                    delay_max = delay_min
                hid = spawn_human(radius=profile["radius"], length=profile["length"])
                state_behaviors = set(profile["behaviors"])
                max_speed = max(profile["speed"]) if isinstance(profile.get("speed"), (list, tuple)) else 0.0
                state = {
                    "id": hid,
                    "cfg": dict(cfg),
                    "phase": "stationary" if ("stationary" in state_behaviors and max_speed <= 0.0) else "idle",
                    "behaviors": state_behaviors,
                    "path": member_path,
                    "segments": member_segments,
                    "radius": profile["radius"],
                    "length": profile["length"],
                    "loop_path": bool(cfg.get("loop", False)),
                    "speed_range": profile["speed"],
                    "speed_mps": rng.uniform(*profile["speed"]),
                    "next_start_t": rng.uniform(
                        delay_min,
                        delay_max
                    ) + float(cfg.get("start_delay_s", 0.0)),
                    "active_since": 0.0,
                    "seg_idx": 0,
                    "seg_progress": 0.0,
                    "wet_since": None,
                    "fallen_since": None,
                    "wet_aabb": local_wet_aabb,
                    "p_slip": base_slip,
                    "slip_min_exposure": float(cfg.get("slip_min_exposure_s", slip_min_exposure_s)),
                    "z_base": profile["radius"] + 0.5 * profile["length"],
                    "posture_state": "upright",
                    "posture_scale": profile.get("height_scale", 1.0),
                    "props": {},
                    "reflectivity": float(cfg.get("reflectivity", 0.0)),
                    "last_pose": (-10.0, -10.0),
                    "hold_until": None,
                    "late_react_triggered": False,
                }
                if member_segments:
                    seg0 = member_segments[0]
                    start_pt = seg0["start"]
                    heading0 = math.atan2(seg0["dir"][1], seg0["dir"][0])
                    state["last_pose"] = (start_pt[0], start_pt[1], heading0)
                    # Place body at start so it is visible even before stepping
                    _update_human_pose(hid, start_pt[0], start_pt[1], state["z_base"], heading0, state.get("posture_scale", 1.0))
                    state["next_start_t"] = min(float(state.get("next_start_t", 0.0)), 0.1)
                if "push_cart" in state_behaviors:
                    cart_id, cart_h = _spawn_cart_body()
                    state["props"]["cart"] = {"id": cart_id, "height": cart_h, "offset": float(cfg.get("cart_offset_m", 0.9))}
                if "carry_payload" in state_behaviors:
                    payload_id, payload_h = _spawn_payload_box()
                    state["props"]["payload"] = {"id": payload_id, "height": payload_h, "dropped": False}
                human_states.append(state)
        use_human = len(human_states) > 0

        env_cfg = scn.get("environment", {}) or {}
        safety_zones_cfg = (scn.get("layout", {}) or {}).get("safety_zones", [])

        vehicle_states: List[Dict[str, Any]] = []
        for vmeta in vehicle_meta:
            raw_pts = vmeta.get("path") or []
            path_pts = [list(pt) for pt in raw_pts if isinstance(pt, (list, tuple)) and len(pt) == 2]
            half_ext = vmeta.get("half_extents") or [0.5, 0.4, 0.4]
            collision_radius = max(abs(half_ext[0]), abs(half_ext[1])) + 0.1
            if walkway_rects and not vmeta.get("raw_coords"):
                path_pts = _project_path_into_rects(path_pts, walkway_rects, collision_radius, static_aabbs, (Lx, Ly), border)
            segments, _ = _segments_from_points(path_pts)
            state = {
                "id": vmeta.get("id"),
                "body_id": vmeta.get("body_id"),
                "path": path_pts,
                "segments": segments,
                "speed": float(vmeta.get("speed_mps", 0.0)),
                "direction": 1,
                "seg_idx": 0,
                "seg_progress": 0.0,
                "loop": bool(vmeta.get("loop", False)),
                "ping_pong": bool(vmeta.get("ping_pong", True)),
                "half_extents": half_ext,
                "height": float(vmeta.get("base_z", half_ext[2])),
                "reflective": bool(vmeta.get("reflective")),
                "reversing_bias": bool(vmeta.get("reversing_bias", False)),
                "reversing_mode": bool(vmeta.get("reversing_mode", False)),
                "reverse_duty": float(vmeta.get("reverse_duty", 0.6)),
                "reverse_period": float(vmeta.get("reverse_period_s", 6.0)),
                "next_reverse_switch": float(rng.uniform(1.5, 3.5)),
                "rear_occlusion_deg": float(vmeta.get("rear_occlusion_deg", 0.0)),
                "alarm": vmeta.get("alarm"),
                "carrying_pallet": bool(vmeta.get("carrying_pallet", False)),
                "load_overhang_m": float(vmeta.get("load_overhang_m", 0.0)),
            }
            body_id = state["body_id"]
            if state["rear_occlusion_deg"] <= 0.0 and state["reversing_bias"]:
                state["rear_occlusion_deg"] = 70.0
            heading = float(vmeta.get("yaw", 0.0))
            if segments:
                first_dir = segments[0]["dir"]
                heading = math.atan2(first_dir[1], first_dir[0])
            if path_pts:
                sx, sy = path_pts[0]
            else:
                if body_id is not None:
                    try:
                        pos, _orn = p.getBasePositionAndOrientation(body_id)
                        sx, sy = pos[0], pos[1]
                    except Exception:
                        sx, sy = start[0], start[1]
                else:
                    sx, sy = start[0], start[1]
            robot_block = (start[0] - radius - 0.3, start[1] - radius - 0.3, start[0] + radius + 0.3, start[1] + radius + 0.3)
            if not vmeta.get("raw_coords"):
                sx, sy = _resolve_spawn_point(sx, sy, collision_radius, (Lx, Ly), border, static_aabbs + [robot_block])
            state["last_pose"] = (sx, sy, heading)
            state["aabb"] = _vehicle_aabb(state, sx, sy)
            if body_id is not None:
                try:
                    p.resetBasePositionAndOrientation(body_id, [sx, sy, state.get("height", 0.4)],
                                                      p.getQuaternionFromEuler([0.0, 0.0, heading]))
                except Exception:
                    pass
            vehicle_states.append(state)

        floor_event_states: List[Dict[str, Any]] = []
        for evt in hazards.get("floor_events", []):
            if not isinstance(evt, dict):
                continue
            floor_event_states.append({
                "cfg": evt,
                "spawned": False,
                "body_id": None,
                "zone_current": evt.get("zone"),
                "meta": None,
                "dynamic_ref": None,
            })

        injectors: List[Injector] = []
        inj_cfg = (_tuned.get("injectors") or {}) if isinstance(_tuned, dict) else {}

        if isinstance(inj_cfg.get("lidar_blackout"), dict):
            c = inj_cfg["lidar_blackout"]
            if rng.random() < float(c.get("p", 0.5)):
                injectors.append(LidarSectorBlackoutInjector(
                    cx=float(c.get("cx", 10.0)),
                    cy=float(c.get("cy", 10.0)),
                    trigger_radius=float(c.get("trigger_radius", 2.0)),
                    duration_s=float(c.get("duration_s", 4.0)),
                    sector_deg=float(c.get("sector_deg", 60.0)),
                    sector_offset_deg=float(c.get("sector_offset_deg", 0.0)),
                ))

        if isinstance(inj_cfg.get("ghost_obstacle"), dict):
            c = inj_cfg["ghost_obstacle"]
            if rng.random() < float(c.get("p", 0.95)):
                injectors.append(GhostObstacleInjector(
                    spawn_rate_hz=float(c.get("spawn_rate_hz", 0.25)),
                    dist_range=tuple(map(float, c.get("dist_range", [1.5, 6.0]))),
                    ttl_range=tuple(map(float, c.get("ttl_range", [0.6, 2.0]))),
                    width_deg=tuple(map(float, c.get("width_deg", [8.0, 18.0]))),
                    range_jitter=float(c.get("range_jitter", 0.4)),
                    max_clusters=int(c.get("max_clusters", 4)),
                    rng_seed=int(rng.integers(2**31 - 1)),
                ))

        if isinstance(inj_cfg.get("falling_object"), dict):
            c = inj_cfg["falling_object"]
            if rng.random() < float(c.get("p", 0.4)):
                inj = FallingObjectInjector(
                    drop_x=float(c.get("drop_x", 8.0)),
                    drop_y=float(c.get("drop_y", 10.0)),
                    drop_z=float(c.get("drop_z", 3.5)),
                    half_extents=tuple(map(float, c.get("half_extents", [0.25, 0.25, 0.25]))),
                    mass=float(c.get("mass", 2.0)),
                    restitution=float(c.get("restitution", 0.1)),
                    lateral_mu=float(c.get("lateral_mu", 0.6)),
                    shatter_on_impact=bool(c.get("shatter_on_impact", True)),
                    puddle_mu=float(c.get("puddle_mu", 0.32)),
                    puddle_half=tuple(map(float, c.get("puddle_half", [1.2, 0.6]))),
                )
                def _spawn_patch_cb(x0, y0, x1, y1, mu):
                    bid = _spawn_wet_patch(x0, y0, x1, y1, mu=mu)
                    ignored_for_collision.add(bid)
                    zone = (float(x0), float(y0), float(x1), float(y1))
                    dynamic_wet_patches.append({"aabb": zone, "mu": mu})
                    floor_zones_meta.append({
                        "id": f"falling_puddle_{len(floor_zones_meta)+1:02d}",
                        "type": "wet",
                        "zone": list(zone),
                        "mu": mu,
                        "body_id": bid,
                        "effects": {"brake_scale": 1.4, "slip_boost": 0.6},
                    })
                    return bid
                inj.spawn_wet_patch_cb = _spawn_patch_cb
                injectors.append(inj)

        # Logging
        log_cfg = (scn.get("logging") or {})
        out_dir.mkdir(parents=True, exist_ok=True)
        rows: List[List[float | str]] = []
        header = [
            "t", "x", "y", "yaw", "v_cmd", "w_cmd",
            "min_clearance_lidar", "min_clearance_geom",
            "event", "event_detail", "in_wet", "human_phase", "near_stop", "hard_brake",
            "near_miss", "occluded_hazard", "interaction", "sensor_faults",
            "min_ttc", "odom_v", "odom_w", "imu_ax", "imu_ay", "imu_wz"
        ]

        odom_meas = (0.0, 0.0)
        imu_meas = (0.0, 0.0, 0.0)
        actor_rows: List[List[float | str]] = []
        actor_header = ["t", "actor_id", "type", "x", "y", "yaw", "phase", "vx", "vy"]
        actor_prev: Dict[str, Tuple[float, float]] = {}
        log_actors = bool(log_cfg.get("log_actors", True))

        def _log_row(event: str, detail: str, *, human_phase: str | None = None,
                     near_stop: int = 0, hard_brake: int = 0, near_miss: int = 0,
                     occluded: int = 0, interaction: str = "none", sensor_faults: str = "",
                     position: Tuple[float, float, float] | None = None,
                     wet_flag: int | None = None, min_ttc_step: float | None = None) -> None:
            px, py, pyaw = position if position is not None else (new_x, new_y, new_yaw)
            in_wet_flag = wet_flag if wet_flag is not None else int(_compute_in_wet(px, py))
            rows.append([
                t, px, py, pyaw, v_cmd, w_cmd,
                min_clear_lidar, min_clear_geom,
                event, detail,
                in_wet_flag,
                human_phase or _human_phase_str(),
                near_stop,
                hard_brake,
                near_miss,
                occluded,
                interaction,
                sensor_faults,
                (min_ttc_step if (min_ttc_step is not None and min_ttc_step < 1e8) else (min_ttc if min_ttc < 1e8 else "")),
                odom_meas[0], odom_meas[1],
                imu_meas[0], imu_meas[1], imu_meas[2],
            ])

        def _log_actor_row(t_now: float, actor_id: str, a_type: str, x: float, y: float,
                           yaw_val: float, phase: str) -> None:
            if not log_actors:
                return
            prev = actor_prev.get(actor_id)
            vx = vy = 0.0
            if prev:
                vx = (x - prev[0]) / max(1e-6, dt)
                vy = (y - prev[1]) / max(1e-6, dt)
            actor_prev[actor_id] = (x, y)
            actor_rows.append([t_now, actor_id, a_type, x, y, yaw_val, phase, vx, vy])

        reflective_regions: List[Dict[str, Any]] = []
        for sobj in env.get("static_obstacles", []) or []:
            if not isinstance(sobj, dict) or not sobj.get("reflective"):
                continue
            aabb = sobj.get("aabb")
            if not isinstance(aabb, (list, tuple)) or len(aabb) != 4:
                continue
            reflective_regions.append({
                "aabb": [float(aabb[0]), float(aabb[1]), float(aabb[2]), float(aabb[3])],
                "strength": float(sobj.get("reflectivity", 0.65)),
                "trigger": float(sobj.get("trigger_m", 1.6)),
            })

        def _write_digest_snapshot() -> None:
            try:
                hazards_digest = {
                    "traction": [
                        dict(zone=list(zone), mu=float(z.get("mu", 0.45)))
                        for z in scn.get("hazards", {}).get("traction", []) if isinstance(z, dict)
                        for zone in [z.get("zone")] if isinstance(zone, (list, tuple)) and len(zone) == 4
                    ],
                    "human": (scn.get("hazards", {}).get("human") or []),
                    "floor_events": [
                        {key: evt.get(key) for key in ("id", "type", "zone", "spawn_time_s")}
                        for evt in (scn.get("hazards", {}).get("floor_events", []) or []) if isinstance(evt, dict)
                    ],
                    "vehicles": [
                        {"id": v.get("id"), "type": v.get("type"), "path_len": len(v.get("path", []) or [])}
                        for v in (scn.get("hazards", {}).get("vehicles", []) or []) if isinstance(v, dict)
                    ],
                }
                digest = build_world_digest(
                    static_aabbs,
                    hazards_digest,
                    (start[0], start[1]),
                    (goal[0], goal[1]),
                    map_size_m=(Lx, Ly),
                    floor_zones=floor_zones_meta,
                    transition_zones=transition_zones_meta,
                    static_objects=list(env.get("static_obstacles", [])),
                    vehicles=vehicle_meta,
                    safety_zones=(safety_zones_cfg or []),
                    environment=env_cfg,
                    aisles=env.get("aisles_meta"),
                )
                write_world_digest(out_dir, digest)

                batch_root = out_dir.parent
                if batch_root.name == "per_run":
                    batch_root = batch_root.parent
                root_world = batch_root / "world.json"
                root_world.write_text((out_dir / "world.json").read_text(encoding="utf-8"), encoding="utf-8")
                import json as _json, hashlib
                h = hashlib.sha256(root_world.read_bytes()).hexdigest()
                man_path = batch_root / "manifest.json"
                try:
                    man = _json.loads(man_path.read_text(encoding="utf-8"))
                except Exception:
                    man = {}
                man["world_sha256"] = h
                man_path.write_text(_json.dumps(man, indent=2), encoding="utf-8")
            except Exception:
                pass

        # Reference controllers now live outside the simulator. Default to a direct goal waypoint
        # so run_one can still drive the robot without internal planning logic.
        waypoints: List[Tuple[float, float]] = [(goal[0], goal[1])]

        max_speed_nominal = float(agent_cfg.get("max_speed_mps", 1.2))

        # Control state
        prev_err = np.array([0.0, 0.0])
        wp_idx = 0
        success = False
        timeout = float(scn.get("runtime", {}).get("duration_s", 90.0))
        max_steps = int(math.ceil(timeout / max(1e-6, dt))) + 1
        step_idx = 0
        min_clear_geom = 10.0
        min_ttc: float = 1e9
        ttc_step: float = 1e9
        v_cmd = 0.0
        w_cmd = 0.0
        near_stop_flag = 0
        near_miss_flag = 0
        hard_brake_flag = 0
        occluded_flag = 0
        interaction = "none"
        sensor_faults = ""

        def _human_phase_str() -> str:
            if any(h["phase"] == "fallen" for h in human_states):
                return "fallen"
            if any(h["phase"] == "running" for h in human_states):
                return "running"
            return "none"

        near_miss_ttc_thresh = float(log_cfg.get("near_miss_ttc_s", 2.0))
        hard_brake_thresh = float(log_cfg.get("hard_brake_decel_mps2", 1.2))
        interaction_dist = float(log_cfg.get("interaction_radius_m", 2.0))

        def _compute_in_wet(xr: float, yr: float) -> bool:
            for surface in floor_zones_meta:
                zone = surface.get("zone")
                if zone and _pt_in_aabb(xr, yr, tuple(zone)):
                    if (surface.get("type") in {"wet", "oil", "cleaning_liquid", "smooth_plastic"} or
                        (surface.get("effects") or {}).get("slip_boost", 0.0) > 0.2):
                        return True
            for patch in scn.get("hazards", {}).get("traction", []):
                if not isinstance(patch, dict):
                    continue
                zone = patch.get("zone")
                if not isinstance(zone, (list, tuple)) or len(zone) != 4:
                    continue
                x0, y0, x1, y1 = zone
                if _pt_in_aabb(xr, yr, (float(x0), float(y0), float(x1), float(y1))):
                    return True
            for patch in dynamic_wet_patches:
                aabb = patch.get("aabb")
                if aabb and _pt_in_aabb(xr, yr, tuple(aabb)):
                    return True
            return False

        def _surface_at(xr: float, yr: float) -> Dict[str, Any] | None:
            for surface in floor_zones_meta:
                zone = surface.get("zone")
                if zone and _pt_in_aabb(xr, yr, tuple(zone)):
                    return surface
            return None

        def _transition_at(xr: float, yr: float) -> Dict[str, Any] | None:
            for tz in transition_zones_meta:
                zone = tz.get("zone")
                if zone and _pt_in_aabb(xr, yr, tuple(zone)):
                    return tz
            return None

        def _sensor_faults(t_now: float, shadow_flag: bool) -> str:
            tags: List[str] = []
            if shadow_flag:
                tags.append("sensor_shadow")
            for inj in injectors:
                if isinstance(inj, LidarSectorBlackoutInjector) and inj.active_until >= t_now >= 0.0:
                    tags.append(inj.name)
                elif isinstance(inj, GhostObstacleInjector) and getattr(inj, "_clusters", None):
                    if len(getattr(inj, "_clusters", [])) > 0:
                        tags.append(inj.name)
            return ";".join(tags)

        def _update_vehicles(dt: float, t_cur: float) -> None:
            for v in vehicle_states:
                body = v.get("body_id")
                segments = v.get("segments") or []
                speed = float(v.get("speed", 0.0))
                if body is None or speed <= 0.0 or not segments:
                    continue
                initial_seg_idx = int(v.get("seg_idx", 0))
                initial_progress = float(v.get("seg_progress", 0.0))
                prev_pose = v.get("last_pose")
                step_remaining = speed * dt
                while step_remaining > 1e-6:
                    seg_idx = max(0, min(len(segments) - 1, int(v.get("seg_idx", 0))))
                    v["seg_idx"] = seg_idx
                    seg = segments[seg_idx]
                    direction = int(v.get("direction", 1)) or 1
                    progress = float(v.get("seg_progress", 0.0))
                    if direction > 0:
                        remain = seg["len"] - progress
                        if remain <= 1e-6:
                            if seg_idx >= len(segments) - 1:
                                if v.get("loop", False):
                                    v["seg_idx"] = 0
                                    v["seg_progress"] = 0.0
                                    continue
                                if v.get("ping_pong", True):
                                    v["direction"] = -1
                                    continue
                                break
                            else:
                                v["seg_idx"] = seg_idx + 1
                                v["seg_progress"] = 0.0
                                continue
                        move = min(step_remaining, remain)
                        v["seg_progress"] = progress + move
                        step_remaining -= move
                    else:
                        if progress <= 1e-6:
                            if seg_idx <= 0:
                                if v.get("loop", False):
                                    v["seg_idx"] = len(segments) - 1
                                    v["seg_progress"] = segments[-1]["len"]
                                    continue
                                if v.get("ping_pong", True):
                                    v["direction"] = 1
                                    continue
                                break
                            else:
                                v["seg_idx"] = seg_idx - 1
                                v["seg_progress"] = segments[v["seg_idx"]]["len"]
                                continue
                        move = min(step_remaining, progress)
                        v["seg_progress"] = progress - move
                        step_remaining -= move
                seg_idx = max(0, min(len(segments) - 1, int(v.get("seg_idx", 0))))
                v["seg_idx"] = seg_idx
                seg = segments[seg_idx]
                length = max(1e-6, seg["len"])
                frac = float(v.get("seg_progress", 0.0)) / length
                x_start, y_start = seg["start"]
                x_end, y_end = seg["end"]
                new_x = x_start + frac * (x_end - x_start)
                new_y = y_start + frac * (y_end - y_start)
                base_heading = math.atan2(y_end - y_start, x_end - x_start)
                moving_forward = v.get("direction", 1) >= 0
                flip = (not moving_forward) != bool(v.get("reversing_mode", False))
                heading = base_heading + (math.pi if flip else 0.0)
                aabb = _vehicle_aabb(v, new_x, new_y)
                if any(_rect_overlap(aabb, obs) for obs in static_aabbs):
                    v["seg_idx"] = initial_seg_idx
                    v["seg_progress"] = initial_progress
                    if prev_pose:
                        v["last_pose"] = prev_pose
                        try:
                            p.resetBasePositionAndOrientation(body, [prev_pose[0], prev_pose[1], v.get("height", 0.4)],
                                                              p.getQuaternionFromEuler([0.0, 0.0, prev_pose[2]]))
                        except Exception:
                            pass
                    continue
                v["aabb"] = aabb
                v["last_pose"] = (new_x, new_y, heading)
                p.resetBasePositionAndOrientation(body, [new_x, new_y, v.get("height", 0.4)],
                                                  p.getQuaternionFromEuler([0.0, 0.0, heading]))

        # crude TTC helper
        last_xy: Optional[Tuple[float, float]] = None
        last_hxy: Dict[int, Tuple[float, float]] = {}

        def _update_min_ttc(x: float, y: float) -> float:
            nonlocal min_ttc, last_xy, last_hxy
            h_positions: List[Tuple[int, float, float]] = []
            for h in human_states:
                if h["phase"] != "running":
                    continue
                try:
                    hx, hy, _ = p.getBasePositionAndOrientation(h["id"])[0]
                except Exception:
                    continue
                h_positions.append((h["id"], hx, hy))
            if last_xy is not None:
                vx_r = (x - last_xy[0]) / max(1e-6, dt)
                vy_r = (y - last_xy[1]) / max(1e-6, dt)
            else:
                vx_r = vy_r = 0.0
            step_min = float("inf")
            for hid, hx, hy in h_positions:
                last_h = last_hxy.get(hid)
                if last_h is not None:
                    vx_h = (hx - last_h[0]) / max(1e-6, dt)
                    vy_h = (hy - last_h[1]) / max(1e-6, dt)
                else:
                    vx_h = vy_h = 0.0
                dx, dy = (hx - x), (hy - y)
                d2 = dx * dx + dy * dy
                if d2 > 1e-6:
                    L = math.sqrt(d2)
                    ux, uy = dx / L, dy / L
                    rel_v = (vx_h - vx_r) * ux + (vy_h - vy_r) * uy
                    if rel_v < -1e-3:
                        ttc = L / (-rel_v)
                        min_ttc = min(min_ttc, ttc)
                        step_min = min(step_min, ttc)
                last_hxy[hid] = (hx, hy)
            last_xy = (x, y)
            return step_min

        # Sensors
        lidar_cfg = ((scn.get("sensors", {}) or {}).get("lidar") or {})
        lidar_hz = float(lidar_cfg.get("hz", 10.0))
        lidar_max = float(lidar_cfg.get("max_range_m", 8.0))
        fog_density = float(lidar_cfg.get("fog_density", 0.0))
        if taxonomy.get("visibility"):
            fog_density = max(fog_density, 0.3)
        lidar_noise = float(lidar_cfg.get("noise_sigma", tuned_lidar_noise))
        lidar_dropout = float(lidar_cfg.get("dropout_pct", tuned_lidar_dropout))
        lidar_log_cfg = (lidar_cfg.get("log") or {})
        log_full_lidar = bool(lidar_log_cfg.get("full", log_cfg.get("full_lidar", False)))
        log_lidar_events_only = bool(lidar_log_cfg.get("events_only", log_cfg.get("lidar_events_only", False)))
        log_lidar_xyz = bool(lidar_log_cfg.get("xyz", log_cfg.get("lidar_xyz", False)))
        pre_event_s = float(lidar_log_cfg.get("pre_event_s", log_cfg.get("lidar_pre_event_s", 1.0)))
        post_event_s = float(lidar_log_cfg.get("post_event_s", log_cfg.get("lidar_post_event_s", 2.0)))
        lidar_log_enabled = log_full_lidar or log_lidar_events_only
        lidar_rng = _child_rng(rng)
        odom_rng = _child_rng(rng)
        imu_rng = _child_rng(rng)
        lidar = _LidarSim(
            beams=360,
            hz=lidar_hz,
            max_range=lidar_max,
            dropout_pct_range=(lidar_dropout, lidar_dropout + 1e-9),
            noise_sigma=lidar_noise,
            jitter_ms_max=tuned_lidar_jitter,
            fog_density=fog_density,
            reflective_zones=reflective_regions,
            rng=lidar_rng,
        )
        lidar_logger = _LidarLogger(
            lidar.angles,
            lidar_log_enabled,
            log_lidar_xyz,
            log_lidar_events_only,
            pre_event_s,
            post_event_s,
            dt,
        )
        odom_cfg = ((scn.get("sensors", {}) or {}).get("odom") or {})
        imu_cfg = ((scn.get("sensors", {}) or {}).get("imu") or {})
        odom = _OdomSim(
            bias_v=tuned_bias_v,
            bias_w=tuned_bias_w,
            noise_v=float(odom_cfg.get("noise_v", 0.02)),
            noise_w=float(odom_cfg.get("noise_w", 0.01)),
            rng=odom_rng,
        )
        imu = _IMUSim(
            accel_noise=float(imu_cfg.get("noise_std", 0.02)),
            gyro_noise=float(imu_cfg.get("gyro_noise", 0.015)),
            rng=imu_rng,
        )
        sensor_z = float(agent_cfg.get("sensor_z_m", max(0.12, robot_height * 0.8)))
        min_clear_lidar = 10.0
        lidar_has_scan = False
        prev_v_true = 0.0
        prev_cmd_v = 0.0
        vehicle_guard_boxes: List[Tuple[float, float, float, float]] = []

        base_frame = str(agent_cfg.get("id", "base_link"))
        frames_meta = {
            "world_frame": "world",
            "base_frame": base_frame,
            "frames": [
                {"parent": base_frame, "child": "laser", "xyz": [0.0, 0.0, sensor_z], "rpy": [0.0, 0.0, 0.0]},
                {"parent": base_frame, "child": "imu", "xyz": [0.0, 0.0, max(0.05, robot_height * 0.5)], "rpy": [0.0, 0.0, 0.0]},
                {"parent": "world", "child": base_frame, "xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
            ],
            "meta": {
                "radius_m": radius,
                "height_m": robot_height,
            },
        }
        try:
            (out_dir / "frames.json").write_text(json.dumps(frames_meta, indent=2), encoding="utf-8")
        except Exception:
            pass

        def _apply_vehicle_occlusion(ranges: np.ndarray, rx: float, ry: float, ryaw: float) -> np.ndarray:
            if not vehicle_states:
                return ranges
            mod = ranges
            cloned = False
            for v in vehicle_states:
                occ_deg = float(v.get("rear_occlusion_deg", 0.0))
                if occ_deg <= 0.0:
                    continue
                pose = v.get("last_pose")
                if not pose:
                    continue
                fx, fy, heading = pose
                dist = math.hypot(fx - rx, fy - ry)
                if dist > 6.0:
                    continue
                sectors: List[float] = []
                if v.get("carrying_pallet"):
                    sectors.append(heading)  # load blocks forward view
                if v.get("reversing_mode"):
                    sectors.append(heading + math.pi)  # blind when backing
                for center in sectors:
                    rel = ((center - ryaw + math.pi) % (2.0 * math.pi)) - math.pi
                    half = max(0.05, math.radians(occ_deg) * 0.5)
                    rel_angles = ((lidar.angles - rel + math.pi) % (2.0 * math.pi)) - math.pi
                    mask = np.abs(rel_angles) <= half
                    if not np.any(mask):
                        continue
                    if not cloned:
                        mod = mod.copy()
                        cloned = True
                    idx = np.nonzero(mask)[0]
                    drop_k = max(1, int(0.6 * len(idx)))
                    drop_idx = lidar_rng.choice(idx, size=drop_k, replace=False)
                    mod[drop_idx] = lidar.max_range
            return mod

        def _apply_human_reflectivity(ranges: np.ndarray, rx: float, ry: float, ryaw: float) -> np.ndarray:
            if not human_states:
                return ranges
            mod = ranges
            cloned = False
            for h in human_states:
                if h.get("phase") != "running":
                    continue
                refl = float(h.get("reflectivity", 0.0))
                if abs(refl) < 1e-3:
                    continue
                last_pose = h.get("last_pose")
                if not isinstance(last_pose, (list, tuple)) or len(last_pose) < 2:
                    continue
                hx, hy = last_pose[0], last_pose[1]
                dist = math.hypot(hx - rx, hy - ry)
                if dist > 6.0:
                    continue
                rel_ang = ((math.atan2(hy - ry, hx - rx) - ryaw + math.pi) % (2.0 * math.pi)) - math.pi
                width = max(0.04, min(0.25, 0.06 + 0.02 * dist))
                rel_angles = ((lidar.angles - rel_ang + math.pi) % (2.0 * math.pi)) - math.pi
                mask = np.abs(rel_angles) <= width
                if not np.any(mask):
                    continue
                if not cloned:
                    mod = mod.copy()
                    cloned = True
                idx = np.nonzero(mask)[0]
                if refl > 0.0:
                    scale = np.clip(lidar_rng.uniform(0.2, 0.6, size=len(idx)), 0.1, 0.8)
                    mod[idx] = np.clip(mod[idx] * scale, 0.05, mod[idx])
                else:
                    drop_k = max(1, int(0.5 * len(idx)))
                    drop_idx = lidar_rng.choice(idx, size=drop_k, replace=False)
                    mod[drop_idx] = lidar.max_range
            return mod

        # -------- main loop --------
    
        while step_idx < max_steps:
            t = step_idx * dt
            if t >= timeout:
                break
            pos, orn = p.getBasePositionAndOrientation(amr)
            x, y, _ = pos
            yaw = p.getEulerFromQuaternion(orn)[2]
            prev_pose = (x, y, yaw)
    
            target = waypoints[min(wp_idx, len(waypoints) - 1)]
            ctrl, prev_err = _pid_follow((x, y), target, prev_err, dt=dt)
            desired_yaw = math.atan2(target[1] - y, target[0] - x)
            yaw_err = (desired_yaw - yaw + math.pi) % (2 * math.pi) - math.pi
            w_cmd = max(-2.5, min(2.5, 1.8 * yaw_err))
            v_cmd = max(0.0, min(max_speed_nominal * 1.5, float(np.linalg.norm(ctrl))))
    
            # Update dynamic geometry
            _update_vehicles(dt, t)
            vehicle_guards: List[Tuple[float, float, float]] = []
            vehicle_guard_boxes.clear()
            for v in vehicle_states:
                pose = v.get("last_pose")
                if not pose:
                    continue
                half_v = v.get("half_extents") or [0.5, 0.4, 0.4]
                r_guard = max(abs(half_v[0]), abs(half_v[1])) + 0.4
                if v.get("reversing_mode"):
                    r_guard += 0.3
                vehicle_guards.append((pose[0], pose[1], r_guard))
                vehicle_guard_boxes.append((pose[0] - r_guard, pose[1] - r_guard, pose[0] + r_guard, pose[1] + r_guard))
            for evt_state in floor_event_states:
                if evt_state["spawned"]:
                    continue
                spawn_t = float(evt_state["cfg"].get("spawn_time_s", 20.0))
                if t >= spawn_t:
                    zone = evt_state.get("zone_current") or evt_state["cfg"].get("zone")
                    if not isinstance(zone, (list, tuple)) or len(zone) != 4:
                        continue
                    x0, y0, x1, y1 = map(float, zone)
                    mu_evt = float(evt_state["cfg"].get("mu", wet_mu))
                    bid = _spawn_wet_patch(x0, y0, x1, y1, mu=mu_evt)
                    patch_ids.append(bid)
                    ignored_for_collision.add(bid)
                    dyn_rec = {"aabb": (x0, y0, x1, y1), "mu": mu_evt}
                    dynamic_wet_patches.append(dyn_rec)
                    meta_entry = {
                        "id": evt_state["cfg"].get("id"),
                        "type": evt_state["cfg"].get("type", "spill"),
                        "zone": zone,
                        "mu": mu_evt,
                        "body_id": bid,
                        "effects": {"brake_scale": 1.4, "slip_boost": 0.6},
                    }
                    floor_zones_meta.append(meta_entry)
                    evt_state["spawned"] = True
                    evt_state["body_id"] = bid
                    evt_state["zone_current"] = list(zone)
                    evt_state["meta"] = meta_entry
                    evt_state["dynamic_ref"] = dyn_rec
                    _log_row("floor_event_spawn", evt_state["cfg"].get("type", "spill"),
                             position=(x, y, yaw), near_stop=int(v_cmd < 0.2))
        
            # Expand any active spreading events (e.g., cleaning liquid)
            for evt_state in floor_event_states:
                if not evt_state.get("spawned"):
                    continue
                spread = float(evt_state.get("cfg", {}).get("spread_rate_mps", 0.0))
                if spread <= 1e-6:
                    continue
                zone_cur = evt_state.get("zone_current")
                if not isinstance(zone_cur, (list, tuple)) or len(zone_cur) != 4:
                    continue
                grow = spread * dt
                x0, y0, x1, y1 = map(float, zone_cur)
                new_zone = [
                    max(0.0, min(Lx, x0 - grow)),
                    max(0.0, min(Ly, y0 - grow)),
                    max(0.0, min(Lx, x1 + grow)),
                    max(0.0, min(Ly, y1 + grow)),
                ]
                if max(abs(new_zone[i] - zone_cur[i]) for i in range(4)) < 1e-5:
                    continue
                # Replace patch body with expanded patch
                old_bid = evt_state.get("body_id")
                if isinstance(old_bid, int):
                    try:
                        p.removeBody(old_bid)
                    except Exception:
                        pass
                    ignored_for_collision.discard(old_bid)
                mu_evt = float(evt_state.get("cfg", {}).get("mu", wet_mu))
                new_bid = _spawn_wet_patch(*new_zone, mu=mu_evt)
                patch_ids.append(new_bid)
                ignored_for_collision.add(new_bid)
                evt_state["body_id"] = new_bid
                evt_state["zone_current"] = list(new_zone)
                meta_entry = evt_state.get("meta")
                if isinstance(meta_entry, dict):
                    meta_entry["zone"] = list(new_zone)
                    meta_entry["body_id"] = new_bid
                dyn_rec = evt_state.get("dynamic_ref")
                if isinstance(dyn_rec, dict):
                    dyn_rec["aabb"] = (new_zone[0], new_zone[1], new_zone[2], new_zone[3])
        
            transition_here = _transition_at(x, y)
            transition_shadow = bool(((transition_here or {}).get("attributes") or {}).get("sensor_shadow", False))
            surface_here = _surface_at(x, y)
            surface_effects = (surface_here or {}).get("effects") or {}
            slip_factor = float(surface_effects.get("slip_boost", 0.0))
        
            # Human behavior update
            if use_human:
                for idx, h in enumerate(human_states):
                    body_id = h["id"]
                    cfg = h["cfg"]
                    behaviors = h.get("behaviors") or set()
                    if h["phase"] == "fallen":
                        if (t - (h["fallen_since"] or 0.0)) >= float(cfg.get("fall_duration_s", tuned_fall_duration)):
                            h["phase"] = "idle" if "stationary" not in behaviors else "stationary"
                            h["fallen_since"] = None
                            h["wet_since"] = None
                            h["next_start_t"] = t + float(cfg.get("recovery_delay_s", 6.0))
                            _update_human_pose(body_id, -10.0, -10.0, h["z_base"], 0.0, h.get("posture_scale", 1.0))
                            payload = h.get("props", {}).get("payload")
                            if payload:
                                payload["dropped"] = True
                        continue
                    if h["phase"] == "stationary":
                        if h.get("path"):
                            origin = h["path"][0]
                            _update_human_pose(body_id, origin[0], origin[1], h["z_base"], 0.0, h.get("posture_scale", 1.0))
                            _update_human_props(h, origin[0], origin[1], 0.0)
                        continue
                    if h["phase"] == "running":
                        if h.get("hold_until") and t < h["hold_until"]:
                            continue
                        if not vehicle_guards:
                            for v in vehicle_states:
                                pose = v.get("last_pose")
                                if not pose:
                                    continue
                                half_v = v.get("half_extents") or [0.5, 0.4, 0.4]
                                safe = max(abs(half_v[0]), abs(half_v[1])) + 0.4
                                vehicle_guards.append((pose[0], pose[1], safe))
                        path_ref = h.get("path") or []
                        last_pose = h.get("last_pose", (-999.0, -999.0))
                        if isinstance(last_pose, (list, tuple)) and len(last_pose) >= 2:
                            hx_prev, hy_prev = last_pose[0], last_pose[1]
                        else:
                            hx_prev, hy_prev = -999.0, -999.0
                        if hx_prev < -100 or hy_prev < -100:
                            if path_ref:
                                hx_prev, hy_prev = path_ref[0]
                            else:
                                hx_prev = hy_prev = 0.0
                        blocked_by_vehicle = False
                        for (vx_b, vy_b, rad_b) in vehicle_guards:
                            if math.hypot(hx_prev - vx_b, hy_prev - vy_b) < (rad_b + h["radius"] + 0.1):
                                h["hold_until"] = t + 0.6
                                blocked_by_vehicle = True
                                break
                        if blocked_by_vehicle:
                            continue
                        segments = h.get("segments") or []
                        if not segments:
                            continue
                        step = max(0.0, float(h.get("speed_mps", 1.0))) * dt
                        if "step_back" in behaviors and rng.random() < 0.01:
                            step *= -0.5
                        completed = False
                        while abs(step) > 1e-6 and segments:
                            seg = segments[h["seg_idx"]]
                            if step >= 0.0:
                                remain = seg["len"] - h["seg_progress"]
                                if step <= remain:
                                    h["seg_progress"] += step
                                    step = 0.0
                                else:
                                    step -= remain
                                    h["seg_idx"] += 1
                                    h["seg_progress"] = 0.0
                                    if h["seg_idx"] >= len(segments):
                                        if h.get("loop_path"):
                                            h["seg_idx"] = 0
                                        else:
                                            h["phase"] = "idle"
                                            h["wet_since"] = None
                                            h["next_start_t"] = t + max(2.0, 60.0 / max(0.1, float(cfg.get("rate_per_min", 2.0))))
                                            _update_human_pose(body_id, -10.0, -10.0, h["z_base"], 0.0, h.get("posture_scale", 1.0))
                                            completed = True
                                            break
                            else:
                                back = min(abs(step), h["seg_progress"])
                                h["seg_progress"] -= back
                                step += back
                                while h["seg_progress"] <= 0 and h["seg_idx"] > 0:
                                    h["seg_idx"] -= 1
                                    h["seg_progress"] = segments[h["seg_idx"]]["len"]
                        if completed or h["phase"] != "running":
                            continue
                        seg = segments[h["seg_idx"]]
                        frac = seg["len"] and (h["seg_progress"] / seg["len"]) or 0.0
                        xh = seg["start"][0] + frac * (seg["end"][0] - seg["start"][0])
                        yh = seg["start"][1] + frac * (seg["end"][1] - seg["start"][1])
                        heading = math.atan2(seg["end"][1] - seg["start"][1], seg["end"][0] - seg["start"][0])
                        if "zig_zag" in behaviors:
                            amp = float(cfg.get("zigzag_amp_m", 0.3))
                            freq = float(cfg.get("zigzag_freq_hz", 0.6))
                            offset = amp * math.sin(2.0 * math.pi * freq * (t - h.get("active_since", t)))
                            xh += -math.sin(heading) * offset
                            yh += math.cos(heading) * offset
                            if "late_react" in behaviors and not h.get("late_react_triggered"):
                                if math.hypot(xh - x, yh - y) < 2.0 and rng.random() < 0.4:
                                    h["late_react_triggered"] = True
                                    h["hold_until"] = t + rng.uniform(0.2, 0.6)
                                    _log_row("human_behavior", f"late_react:{cfg.get('id', idx)}",
                                             human_phase="running", near_stop=int(v_cmd < 0.2),
                                             position=(x, y, yaw), wet_flag=int(_compute_in_wet(xh, yh)))
                                    continue
                        min_robot_clear = max(0.7, radius + h["radius"] + 0.35)
                        dist_robot = math.hypot(xh - x, yh - y)
                        if dist_robot < min_robot_clear:
                            shift = (min_robot_clear - dist_robot) + 0.05
                            xh += math.cos(heading) * shift
                            yh += math.sin(heading) * shift
                        human_aabb = (xh - h["radius"], yh - h["radius"], xh + h["radius"], yh + h["radius"])
                        if any(_rect_overlap(human_aabb, obs) for obs in static_aabbs):
                            h["hold_until"] = t + 0.5
                            continue
                        _update_human_pose(body_id, xh, yh, h["z_base"], heading, h.get("posture_scale", 1.0))
                        _update_human_props(h, xh, yh, heading)
                        h["last_pose"] = (xh, yh, heading)
                        wet_band = h.get("wet_aabb")
                        in_wet_now = False
                        if wet_band and _pt_in_aabb(xh, yh, wet_band):
                            in_wet_now = True
                        elif _compute_in_wet(xh, yh):
                            in_wet_now = True
                        if in_wet_now:
                            if h["wet_since"] is None:
                                h["wet_since"] = t
                        else:
                            h["wet_since"] = None
                        req_exposure = float(h.get("slip_min_exposure", slip_min_exposure_s))
                        if h["wet_since"] is not None and ((t - h["wet_since"]) >= req_exposure):
                            surface = _surface_at(xh, yh)
                            mu_here = _floor_mu_at(xh, yh, floor_zones_meta, dynamic_wet_patches, wet_mu, default_mu=0.85)
                            slip_boost = max(0.0, ((surface or {}).get("effects") or {}).get("slip_boost", 0.0))
                            base_slip = float(h.get("p_slip", tuned_slip_prob))
                            mu_slip = _slip_prob_from_mu(mu_here)
                            base_weight = max(0.1, min(1.0, ((0.42 - mu_here) / 0.1) + 0.2))
                            base_term = base_slip * base_weight
                            slip_prob = 1.0 - (1.0 - base_term) * (1.0 - mu_slip)
                            if surface is None and not in_wet_now:
                                slip_prob = min(slip_prob, 0.05)
                            slip_prob = min(0.99, slip_prob + slip_boost)
                            if rng.random() < slip_prob:
                                h["phase"] = "fallen"
                                h["fallen_since"] = t
                                h["wet_since"] = None
                                dx_slide, dy_slide = _fall_slide_offset(h.get("speed_mps", 1.0), mu_here, heading, slip_boost=slip_boost)
                                xh_slide = max(border, min(Lx - border, xh + dx_slide))
                                yh_slide = max(border, min(Ly - border, yh + dy_slide))
                                p.resetBasePositionAndOrientation(body_id, [xh_slide, yh_slide, 0.25], p.getQuaternionFromEuler([math.pi / 2.0, 0.0, 0.0]))
                                h["last_pose"] = (xh_slide, yh_slide, heading)
                                lidar_logger.mark_event()
                                _log_row("floor_slip", f"{xh_slide:.2f},{yh_slide:.2f},{cfg.get('id', idx)}",
                                         human_phase="fallen", near_stop=int(v_cmd < 0.2),
                                         position=(x, y, yaw), wet_flag=int(_compute_in_wet(xh_slide, yh_slide)))
                                payload = h.get("props", {}).get("payload")
                                if payload:
                                    payload["dropped"] = True
                                    p.resetBasePositionAndOrientation(payload["id"], [xh_slide, yh_slide, payload["height"] / 2.0],
                                                                       p.getQuaternionFromEuler([0.0, 0.0, 0.0]))
                        continue
                    default_trigger = 0.6 if taxonomy.get("occlusion") else 1.0
                    trigger_distance = float(max(0.3, cfg.get("trigger_distance", default_trigger)))
                    if h["phase"] == "idle" and (t >= h["next_start_t"] or (h.get("path") and math.hypot(x - h["path"][0][0], y - h["path"][0][1]) < trigger_distance)):
                        start_xy = None
                        if h.get("path") and len(h["path"]) >= 1:
                            start_xy = (h["path"][0][0], h["path"][0][1])
                        spawn_clear = max(0.3, float(cfg.get("spawn_clearance_m", 0.3)))
                        if start_xy is not None:
                            other_humans = [ho for ho in human_states if ho is not h]
                            if not _human_spawn_ok(start_xy, h["radius"], (x, y), radius, vehicle_guards, other_humans, clearance=spawn_clear):
                                h["next_start_t"] = t + 0.5
                                continue
                        h["phase"] = "running"
                        h["active_since"] = t
                        h["seg_idx"] = 0
                        h["seg_progress"] = 0.0
                        h["speed_mps"] = rng.uniform(*h.get("speed_range", (0.8, 1.4)))
                        h["hold_until"] = None
                        h["late_react_triggered"] = False
                        h["wet_since"] = None
            # Injectors
            active_human_ids = [h["id"] for h in human_states if h["phase"] == "running"]
            fallen_ids = [h["id"] for h in human_states if h["phase"] == "fallen"]
            state = InjectorState(
                t=t, dt=dt, robot_xy=(x, y), robot_yaw=yaw,
                humans=active_human_ids,
                fallen_human=fallen_ids[0] if fallen_ids else None,
                bounds=(Lx, Ly), border=border
            )
            for inj in injectors:
                inj.on_step(state)
        
            # Sensors
            lidar_ignore = ignored_for_collision | {amr}
            lidar.maybe_update(t, x, y, yaw, sensor_z, lidar_ignore)
            lr = lidar.try_pop_ready(t)
            if lr is not None:
                for inj in injectors:
                    mod = inj.apply_lidar(state, lr, lidar.max_range)
                    if mod is not None:
                        lr = mod
                lr = _apply_dynamic_reflections(lr, (x, y), vehicle_states, rng)
                if transition_shadow:
                    if len(lr) > 0:
                        mask = rng.choice(len(lr), size=max(1, len(lr)//12), replace=False)
                        lr = lr.copy()
                        lr[mask] = lidar.max_range
                lr = _apply_human_reflectivity(lr, x, y, yaw)
                lr = _apply_vehicle_occlusion(lr, x, y, yaw)
                min_clear_lidar = float(np.min(lr))
                lidar_has_scan = True
                lidar_logger.record(t, (x, y, yaw), lr)
        
            # Integrate motion using commanded (ground-truth) velocities; sensors get noisy copies
            slip_scale = max(0.0, 1.0 - slip_factor)
            v_true = v_cmd * slip_scale
            w_true = w_cmd * slip_scale
            decel_cmd = (prev_cmd_v - v_cmd) / max(1e-6, dt)
            decel_true = (prev_v_true - v_true) / max(1e-6, dt)
            hard_brake_flag = 1 if max(decel_cmd, decel_true) > hard_brake_thresh else 0
            heading_motion = desired_yaw  # move along the goal heading even if body yaw is still turning
            new_yaw = yaw + w_true * dt
            new_x = max(border, min(Lx - border, x + (v_true * math.cos(heading_motion)) * dt))
            new_y = max(border, min(Ly - border, y + (v_true * math.sin(heading_motion)) * dt))

            odom_meas = odom.apply(v_true, w_true, slip_factor=slip_factor)
            imu_meas = imu.measure(v_true, new_yaw, dt, surface_effects)
            p.resetBasePositionAndOrientation(amr, [new_x, new_y, robot_height * 0.5], p.getQuaternionFromEuler([0, 0, new_yaw]))
        
            human_body_ids = {h["id"] for h in human_states}
            raw_contacts = p.getContactPoints(bodyA=amr)
            human_contact_flag = False
            nonhuman_contact_count = 0
            for cp in raw_contacts:
                bodyA = cp[1]; bodyB = cp[2]
                if amr not in (bodyA, bodyB):
                    continue
                other = bodyB if bodyA == amr else bodyA
                if other in ignored_for_collision:
                    continue
                if other in human_body_ids:
                    human_contact_flag = True
                    break
                nonhuman_contact_count += 1
        
            ttc_step = _update_min_ttc(new_x, new_y)
        
            # Clearance (geom) vs static + dynamic human footprint
            def _min_clear_geom(px: float, py: float, ry: float) -> float:
                base = geom_clearance_aabbs
                dyn = list(base)
                if not dyn and static_clearance_body_ids:
                    for bid in static_clearance_body_ids:
                        try:
                            a0, a1 = p.getAABB(bid)
                            dyn.append((a0[0], a0[1], a1[0], a1[1]))
                        except Exception:
                            continue
                for fh_id in [h["id"] for h in human_states if h["phase"] == "fallen"]:
                    fh = _fallen_human_aabb(fh_id)
                    if fh is not None:
                        inflate = 0.3 * radius
                        dyn.append((fh[0] - inflate, fh[1] - inflate, fh[2] + inflate, fh[3] + inflate))
                for h in human_states:
                    if h["phase"] != "running":
                        continue
                    last_pose = h.get("last_pose", (-999.0, -999.0))
                    hx, hy = (last_pose[0], last_pose[1]) if isinstance(last_pose, (list, tuple)) and len(last_pose) >= 2 else (-999.0, -999.0)
                    if hx < -100 or hy < -100:
                        continue
                    slice_w = max(0.2, float(h.get("radius", 0.25)))
                    aabb = (hx - slice_w, max(0.0, hy - slice_w), hx + slice_w, min(Ly, hy + slice_w))
                    dyn.append(aabb)
                    dyn.extend(vehicle_guard_boxes)
                    return _clearance_to_aabbs(px, py, dyn)
        
            try:
                cur_geom = float(_min_clear_geom(new_x, new_y, new_y))
            except Exception:
                cur_geom = math.inf
            if geom_clearance_aabbs:
                try:
                    dist_geom = float(_clearance_to_aabbs(new_x, new_y, geom_clearance_aabbs))
                    if math.isfinite(dist_geom):
                        cur_geom = min(cur_geom, dist_geom)
                except Exception:
                    # Keep going; lidar fallback will cover us
                    pass
            try:
                if math.isfinite(min_clear_lidar):
                    # Fallback/bridge: use LiDAR clearance when geometry list is empty or failed
                    cur_geom = min(cur_geom, float(min_clear_lidar))
            except Exception:
                pass
            if not math.isfinite(cur_geom):
                cur_geom = 10.0
            min_clear_geom = min(min_clear_geom, cur_geom)
            near_stop_flag = int(v_cmd < 0.2)
            near_miss_flag = 1 if (ttc_step < near_miss_ttc_thresh and ttc_step < 1e8) else 0
            if near_miss_flag:
                lidar_logger.mark_event()
            occluded_flag = 0
            if lidar_has_scan:
                if ((min_clear_lidar - cur_geom) > 0.3 and cur_geom < 3.0) or (transition_shadow and cur_geom < 2.5) or (taxonomy.get("occlusion") and cur_geom < 1.0):
                    occluded_flag = 1
            sensor_faults = _sensor_faults(t, transition_shadow)
            min_human_dist = 1e9
            human_hold = False
            for h_idx, h in enumerate(human_states):
                lp = h.get("last_pose")
                if not isinstance(lp, (list, tuple)) or len(lp) < 2:
                    continue
                hx, hy = float(lp[0]), float(lp[1])
                min_human_dist = min(min_human_dist, math.hypot(new_x - hx, new_y - hy))
                if h.get("hold_until") and h["phase"] == "running" and t < float(h.get("hold_until") or 0.0):
                    human_hold = True
            interaction = "none"
            if min_human_dist < 0.8 and near_stop_flag:
                interaction = "forced_stop_for_human"
            elif min_human_dist < interaction_dist and near_stop_flag:
                interaction = "robot_yielding"
            elif human_hold and min_human_dist < interaction_dist:
                interaction = "human_yielding"
        
            if log_actors:
                _log_actor_row(t, base_frame, str(agent_cfg.get("type", "robot")), new_x, new_y, new_yaw, "running")
                for h_idx, h in enumerate(human_states):
                    lp = h.get("last_pose")
                    if not isinstance(lp, (list, tuple)) or len(lp) < 2:
                        continue
                    hx, hy = float(lp[0]), float(lp[1])
                    if hx < -50 or hy < -50:
                        continue
                    hyaw = float(lp[2]) if len(lp) >= 3 else 0.0
                    hid = str(h.get("cfg", {}).get("id", f"human_{h_idx+1}"))
                    _log_actor_row(t, hid, "human", hx, hy, hyaw, h.get("phase", "unknown"))
                for v_idx, v in enumerate(vehicle_states):
                    lp = v.get("last_pose")
                    if not lp:
                        continue
                    vyaw = float(lp[2]) if len(lp) >= 3 else 0.0
                    vid = str(v.get("id", f"veh_{v_idx+1}"))
                    vtype = str(v.get("type", "vehicle"))
                    _log_actor_row(t, vid, vtype, float(lp[0]), float(lp[1]), vyaw, "moving" if v.get("speed", 0.0) > 0.0 else "idle")
        
            # waypoint / success
            target = waypoints[min(wp_idx, len(waypoints) - 1)]
            if math.hypot(target[0] - new_x, target[1] - new_y) < 0.5:
                wp_idx += 1
                if wp_idx >= len(waypoints):
                    success = True
                    _log_row("mission_success", "", near_stop=near_stop_flag, hard_brake=hard_brake_flag,
                             near_miss=near_miss_flag, occluded=occluded_flag, interaction=interaction,
                             sensor_faults=sensor_faults, min_ttc_step=ttc_step)
                    break
        
            # contacts & outcomes
            # injector events
            for inj in injectors:
                ev = inj.pop_event()
                if ev:
                    _log_row(ev[0], ev[1], near_stop=near_stop_flag, hard_brake=hard_brake_flag,
                             near_miss=near_miss_flag, occluded=occluded_flag, interaction=interaction,
                             sensor_faults=sensor_faults, min_ttc_step=ttc_step)
        
            if human_contact_flag:
                lidar_logger.mark_event()
                _log_row("collision_human", "1", near_stop=near_stop_flag, hard_brake=hard_brake_flag,
                         near_miss=near_miss_flag, occluded=occluded_flag, interaction=interaction,
                         sensor_faults=sensor_faults, min_ttc_step=ttc_step)
                break
            elif nonhuman_contact_count > 0:
                lidar_logger.mark_event()
                _log_row("collision_static", str(nonhuman_contact_count), near_stop=near_stop_flag, hard_brake=hard_brake_flag,
                         near_miss=near_miss_flag, occluded=occluded_flag, interaction=interaction,
                         sensor_faults=sensor_faults, min_ttc_step=ttc_step)
                break
        
            # regular log row
            _log_row("", "", near_stop=near_stop_flag, hard_brake=hard_brake_flag,
                     near_miss=near_miss_flag, occluded=occluded_flag, interaction=interaction,
                     sensor_faults=sensor_faults, min_ttc_step=ttc_step)
        
            prev_v_true = v_true
            prev_cmd_v = v_cmd
            p.stepSimulation()
            if realtime:
                sleep_dt = max(0.0, dt * max(0.0, sleep_scale))
                if sleep_dt > 0:
                    time.sleep(sleep_dt)
            step_idx += 1
        
        if not success and (not rows or (rows and not rows[-1][8])):
            _log_row("timeout", "elapsed", near_stop=near_stop_flag, hard_brake=hard_brake_flag,
                     near_miss=near_miss_flag, occluded=occluded_flag, interaction=interaction,
                     sensor_faults=sensor_faults, min_ttc_step=ttc_step)
    
        _write_digest_snapshot()
        _write_csv(out_dir / "run_one.csv", header, rows)
        if log_actors and actor_rows:
            _write_csv(out_dir / "actors.csv", actor_header, actor_rows)
        try:
            lidar_logger.write(out_dir / "lidar.npz")
        except Exception:
            pass
        return {"success": success, "time": t, "steps": len(rows)}

    finally:
        try:
            p.disconnect(client_id)
        except Exception:
            pass

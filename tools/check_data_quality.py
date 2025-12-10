#!/usr/bin/env python3
"""
Data quality checks for EdgeSim runs.

What it checks per run:
- run_one.csv sanity: numeric fields are finite, timestamps are non-decreasing, clearances/TTC are non-negative.
- Wet flags: in_wet=1 only when the robot footprint overlaps a wet/traction patch from world.json.
- Events vs geometry: collisions line up with static AABBs; slips happen on low-μ/“wet-like” surfaces.
- Actor spawns (if actors.csv exists): initial positions lie inside the map bounds and outside static geometry.

Usage:
  python tools/check_data_quality.py runs/<run_dir> [runs/<run_dir2> ...]
  # works on single-run folders or batch folders with per_run/run_XXXX subdirs
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

import numpy as np

Rect = Tuple[float, float, float, float]

WET_TYPES = {"wet", "oil", "cleaning_liquid", "smooth_plastic"}


@dataclass
class RunResult:
    run_dir: Path
    counts: Dict[str, int] = field(default_factory=dict)
    details: Dict[str, List[str]] = field(default_factory=dict)
    missing: List[str] = field(default_factory=list)

    @property
    def total_anomalies(self) -> int:
        return sum(self.counts.values())


def _add_issue(res: RunResult, category: str, msg: str, limit: int) -> None:
    res.counts[category] = res.counts.get(category, 0) + 1
    bucket = res.details.setdefault(category, [])
    if len(bucket) < limit:
        bucket.append(msg)


def _normalize_rect(rect: Iterable[Any]) -> Optional[Rect]:
    try:
        x0, y0, x1, y1 = map(float, rect)
    except Exception:
        return None
    if x1 < x0:
        x0, x1 = x1, x0
    if y1 < y0:
        y0, y1 = y1, y0
    return (x0, y0, x1, y1)


def _point_in_rect(pt: Tuple[float, float], rect: Rect) -> bool:
    x, y = pt
    x0, y0, x1, y1 = rect
    return x0 <= x <= x1 and y0 <= y <= y1


def _circle_rect_intersect(cx: float, cy: float, radius: float, rect: Rect) -> bool:
    x0, y0, x1, y1 = rect
    nx = min(max(cx, x0), x1)
    ny = min(max(cy, y0), y1)
    dx = cx - nx
    dy = cy - ny
    return (dx * dx + dy * dy) <= (radius * radius + 1e-9)


def _overlap_area(a: Rect, b: Rect) -> float:
    x0 = max(a[0], b[0])
    y0 = max(a[1], b[1])
    x1 = min(a[2], b[2])
    y1 = min(a[3], b[3])
    if x1 <= x0 or y1 <= y0:
        return 0.0
    return (x1 - x0) * (y1 - y0)


def _parse_bool(val: Any) -> bool:
    if isinstance(val, (int, float)):
        return bool(val)
    s = str(val).strip().lower()
    return s in {"1", "true", "yes", "y", "on"}


def _try_load_yaml(path: Path) -> Optional[Dict[str, Any]]:
    try:
        import yaml  # type: ignore
    except Exception:
        return None
    try:
        obj = yaml.safe_load(path.read_text(encoding="utf-8"))
        return obj if isinstance(obj, dict) else None
    except Exception:
        return None


def _load_scenario(run_dir: Path) -> Optional[Dict[str, Any]]:
    for candidate in (run_dir / "scenario.yaml", run_dir.parent / "scenario.yaml"):
        if candidate.exists():
            scn = _try_load_yaml(candidate)
            if scn is not None:
                return scn
    return None


def _load_world(run_dir: Path) -> Tuple[Optional[Dict[str, Any]], Optional[Path]]:
    for candidate in (run_dir / "world.json", run_dir.parent / "world.json"):
        if candidate.exists():
            try:
                return json.loads(candidate.read_text(encoding="utf-8")), candidate
            except Exception:
                return None, candidate
    return None, None


def _map_bounds(world: Optional[Dict[str, Any]]) -> Tuple[float, float]:
    if not world:
        return 0.0, 0.0
    env = world.get("environment", {}) or {}
    size = env.get("map_size_m")
    if isinstance(size, (list, tuple)) and len(size) >= 2:
        try:
            return float(size[0]), float(size[1])
        except Exception:
            pass
    max_x = 0.0
    max_y = 0.0
    for rect in world.get("aabbs", []) or []:
        norm = _normalize_rect(rect)
        if norm:
            max_x = max(max_x, norm[2])
            max_y = max(max_y, norm[3])
    return max_x, max_y


def _static_rects(world: Optional[Dict[str, Any]]) -> List[Rect]:
    rects: List[Rect] = []
    if not world:
        return rects
    for obj in world.get("static_obstacles", []) or []:
        rect = _normalize_rect(obj.get("aabb", []))
        if rect:
            rects.append(rect)
    for rect in world.get("aabbs", []) or []:
        norm = _normalize_rect(rect)
        if norm:
            rects.append(norm)
    return rects


def _wet_zones(world: Optional[Dict[str, Any]]) -> List[Tuple[Rect, float, str]]:
    zones: List[Tuple[Rect, float, str]] = []
    if not world:
        return zones
    for surface in world.get("floor_zones", []) or []:
        rect = _normalize_rect(surface.get("zone", []))
        if not rect:
            continue
        s_type = str(surface.get("type", "")).lower()
        effects = surface.get("effects") or {}
        mu = float(surface.get("mu", 0.85)) if surface.get("mu") is not None else 0.85
        if s_type in WET_TYPES or effects.get("slip_boost", 0.0) > 0.2:
            zones.append((rect, mu, str(surface.get("id") or surface.get("type") or "floor")))
    for patch in (world.get("hazards", {}) or {}).get("traction", []) or []:
        rect = _normalize_rect(patch.get("zone", []))
        if not rect:
            continue
        mu = float(patch.get("mu", 0.45)) if patch.get("mu") is not None else 0.45
        zones.append((rect, mu, str(patch.get("id") or "traction")))
    return zones


def _mu_at(world: Optional[Dict[str, Any]], x: float, y: float, default: float = 0.9) -> float:
    if not world:
        return default
    for surface in world.get("floor_zones", []) or []:
        rect = _normalize_rect(surface.get("zone", []))
        if rect and _point_in_rect((x, y), rect):
            try:
                return float(surface.get("mu", default))
            except Exception:
                return default
    for patch in (world.get("hazards", {}) or {}).get("traction", []) or []:
        rect = _normalize_rect(patch.get("zone", []))
        if rect and _point_in_rect((x, y), rect):
            try:
                return float(patch.get("mu", default))
            except Exception:
                return default
    return default


def _robot_radius(scn: Optional[Dict[str, Any]], override: Optional[float]) -> float:
    if override is not None:
        return float(override)
    if scn:
        agents = scn.get("agents")
        if isinstance(agents, list) and agents:
            try:
                r = float((agents[0] or {}).get("radius_m", 0.4))
                if math.isfinite(r) and r > 0:
                    return r
            except Exception:
                pass
    return 0.4


def _human_radius_map(world: Optional[Dict[str, Any]], scn: Optional[Dict[str, Any]]) -> Dict[str, float]:
    out: Dict[str, float] = {}
    for src in (world.get("hazards", {}).get("human", []) if world else []) or []:
        if isinstance(src, dict) and src.get("id"):
            try:
                out[str(src["id"])] = float(src.get("radius_m", 0.25))
            except Exception:
                out[str(src["id"])] = 0.25
    if scn:
        for h in (scn.get("hazards", {}) or {}).get("human", []) or []:
            if isinstance(h, dict) and h.get("id"):
                try:
                    out[str(h["id"])] = float(h.get("radius_m", out.get(str(h["id"]), 0.25)))
                except Exception:
                    out[str(h["id"])] = out.get(str(h["id"]), 0.25)
    return out


def _load_actors(run_dir: Path) -> List[Dict[str, Any]]:
    path = run_dir / "actors.csv"
    if not path.exists():
        return []
    rows: List[Dict[str, Any]] = []
    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                t = float(row.get("t", ""))
                x = float(row.get("x", ""))
                y = float(row.get("y", ""))
            except Exception:
                continue
            rows.append({
                "t": t,
                "actor_id": str(row.get("actor_id", "")),
                "type": str(row.get("type", "")).strip().lower(),
                "x": x,
                "y": y,
            })
    return rows


def _dt_median(times: List[float]) -> float:
    if len(times) < 2:
        return 0.0
    diffs = [b - a for a, b in zip(times[:-1], times[1:]) if b >= a]
    return statistics.median(diffs) if diffs else 0.0


def _check_lidar(
    lidar_path: Path,
    res: RunResult,
    times: List[float],
    median_dt: float,
    scn: Optional[Dict[str, Any]],
    detail_limit: int,
) -> None:
    try:
        data = np.load(lidar_path)
    except Exception as exc:  # noqa: BLE001
        _add_issue(res, "lidar_load", f"failed to load lidar.npz ({exc})", detail_limit)
        return

    if "ranges" not in data or "t" not in data:
        _add_issue(res, "lidar_shape", "lidar.npz missing ranges or t arrays", detail_limit)
        return

    ranges = np.asarray(data["ranges"])
    t_arr = np.asarray(data["t"]).ravel()
    angles = np.asarray(data["angles"]) if "angles" in data else np.array([])
    pose = np.asarray(data["pose"]) if "pose" in data else None

    if ranges.ndim != 2:
        _add_issue(res, "lidar_shape", f"ranges has ndim={ranges.ndim}, expected 2", detail_limit)
        return
    frames, beams = ranges.shape
    if frames == 0 or beams == 0:
        _add_issue(res, "lidar_shape", "ranges is empty", detail_limit)
        return

    if t_arr.shape[0] != frames:
        _add_issue(res, "lidar_shape", f"t len {t_arr.shape[0]} != ranges frames {frames}", detail_limit)
    if pose is not None and pose.size > 0 and pose.shape[0] != frames:
        _add_issue(res, "lidar_shape", f"pose len {pose.shape[0]} != ranges frames {frames}", detail_limit)
    if angles.size > 0 and angles.shape[0] != beams:
        _add_issue(res, "lidar_shape", f"angles len {angles.shape[0]} != beams {beams}", detail_limit)

    invalid = np.sum(~np.isfinite(ranges))
    if invalid > 0:
        _add_issue(res, "lidar_nan", f"{invalid} non-finite range values", detail_limit)

    finite_mask = np.isfinite(ranges)
    if np.any(finite_mask):
        finite_vals = ranges[finite_mask]
        min_r = float(np.min(finite_vals))
        max_r = float(np.max(finite_vals))
        lidar_cfg = ((scn or {}).get("sensors", {}) or {}).get("lidar", {}) if scn else {}
        max_range = float(lidar_cfg.get("max_range_m", 8.0))
        if min_r < -1e-3:
            _add_issue(res, "lidar_range", f"min range {min_r:.3f} below 0", detail_limit)
        if max_r > max_range * 1.05:
            _add_issue(res, "lidar_range", f"max range {max_r:.3f} exceeds limit {max_range}", detail_limit)

    if t_arr.ndim != 1:
        _add_issue(res, "lidar_shape", f"t has ndim={t_arr.ndim}, expected 1", detail_limit)
    else:
        if np.any(np.diff(t_arr) < -1e-6):
            _add_issue(res, "lidar_timestamps", "lidar t not non-decreasing", detail_limit)
        if frames and times:
            run_start, run_end = times[0], times[-1]
            tol = max(0.2, median_dt * 4.0 if median_dt > 0 else 0.2)
            t0 = float(t_arr[0]); t1 = float(t_arr[-1])
            if t0 > run_start + tol:
                _add_issue(res, "lidar_align", f"lidar starts after run by {t0 - run_start:.3f}s", detail_limit)
            if t0 < run_start - tol:
                _add_issue(res, "lidar_align", f"lidar starts before run by {run_start - t0:.3f}s", detail_limit)
            if t1 < run_end - tol:
                _add_issue(res, "lidar_align", f"lidar ends before run by {run_end - t1:.3f}s", detail_limit)


def _iter_run_dirs(path: Path) -> List[Path]:
    if (path / "run_one.csv").exists():
        return [path]
    per_run = path / "per_run"
    if per_run.exists():
        return sorted([p for p in per_run.glob("run_*") if (p / "run_one.csv").exists()])
    return []


def check_run(run_dir: Path, *, radius_override: Optional[float] = None, detail_limit: int = 5) -> RunResult:
    res = RunResult(run_dir=run_dir)
    world, world_path = _load_world(run_dir)
    if world is None and world_path is None:
        res.missing.append("world.json")
    scn = _load_scenario(run_dir)
    robot_radius = _robot_radius(scn, radius_override)
    bounds = _map_bounds(world)
    static_rects = _static_rects(world)
    wet_zones = _wet_zones(world)
    human_r_map = _human_radius_map(world, scn)
    actors = _load_actors(run_dir)

    # Actor spawn checks (first occurrence per actor)
    seen_spawn: Dict[str, bool] = {}
    for row in actors:
        aid = row["actor_id"]
        if aid in seen_spawn:
            continue
        seen_spawn[aid] = True
        x, y = row["x"], row["y"]
        if bounds != (0.0, 0.0):
            if x < 0.0 or y < 0.0 or x > bounds[0] or y > bounds[1]:
                _add_issue(res, "actor_spawn", f"{aid}: spawn at ({x:.2f},{y:.2f}) outside map {bounds}", detail_limit)
                continue
        for rect in static_rects:
            if _point_in_rect((x, y), rect):
                _add_issue(res, "actor_spawn", f"{aid}: spawn inside static AABB {rect}", detail_limit)
                break

    csv_path = run_dir / "run_one.csv"
    if not csv_path.exists():
        res.missing.append("run_one.csv")
        return res

    rows: List[Dict[str, Any]] = []
    with csv_path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    if not rows:
        _add_issue(res, "csv", "run_one.csv has no rows", detail_limit)
        return res

    times: List[float] = []
    last_t: Optional[float] = None
    median_dt = 0.0
    first_pose: Optional[Tuple[float, float]] = None

    def _parse_float(field: str, raw: Any, row_idx: int, allow_blank: bool = False) -> Optional[float]:
        if allow_blank and (raw is None or raw == ""):
            return None
        try:
            val = float(raw)
        except Exception:
            _add_issue(res, "csv", f"row {row_idx}: {field} not numeric ({raw})", detail_limit)
            return None
        if not math.isfinite(val):
            _add_issue(res, "csv", f"row {row_idx}: {field} is NaN/inf", detail_limit)
            return None
        return val

    for idx, row in enumerate(rows):
        t = _parse_float("t", row.get("t", ""), idx)
        x = _parse_float("x", row.get("x", ""), idx)
        y = _parse_float("y", row.get("y", ""), idx)
        yaw = _parse_float("yaw", row.get("yaw", ""), idx)
        v_cmd = _parse_float("v_cmd", row.get("v_cmd", ""), idx)
        w_cmd = _parse_float("w_cmd", row.get("w_cmd", ""), idx)
        min_cl_lidar = _parse_float("min_clearance_lidar", row.get("min_clearance_lidar", ""), idx)
        min_cl_geom = _parse_float("min_clearance_geom", row.get("min_clearance_geom", ""), idx)
        min_ttc = _parse_float("min_ttc", row.get("min_ttc", ""), idx, allow_blank=True)
        # Sensor outputs (optional)
        _ = _parse_float("odom_v", row.get("odom_v", ""), idx, allow_blank=True)
        _ = _parse_float("odom_w", row.get("odom_w", ""), idx, allow_blank=True)
        _ = _parse_float("imu_ax", row.get("imu_ax", ""), idx, allow_blank=True)
        _ = _parse_float("imu_ay", row.get("imu_ay", ""), idx, allow_blank=True)
        _ = _parse_float("imu_wz", row.get("imu_wz", ""), idx, allow_blank=True)

        if t is None or x is None or y is None:
            continue
        times.append(t)
        if first_pose is None:
            first_pose = (x, y)
        if last_t is not None and t < last_t - 1e-6:
            _add_issue(res, "timestamps", f"row {idx}: t={t:.4f} < previous {last_t:.4f}", detail_limit)
        last_t = t

        # Non-negative invariants
        for val, label in ((min_cl_lidar, "min_clearance_lidar"), (min_cl_geom, "min_clearance_geom"), (min_ttc, "min_ttc")):
            if val is not None and val < -1e-6:
                _add_issue(res, "negative", f"row {idx}: {label} negative ({val})", detail_limit)

        wet_flag = _parse_bool(row.get("in_wet", 0))
        if wet_flag:
            if not wet_zones:
                _add_issue(res, "wet_flag", f"row {idx}: in_wet=1 but no wet zones in world.json", detail_limit)
            elif not any(_circle_rect_intersect(x, y, robot_radius, rect) for rect, _, _ in wet_zones):
                _add_issue(res, "wet_flag", f"row {idx}: in_wet=1 but robot ({x:.2f},{y:.2f}) not on wet patch", detail_limit)
        else:
            if wet_zones and any(_circle_rect_intersect(x, y, robot_radius, rect) for rect, _, _ in wet_zones):
                _add_issue(res, "wet_missed", f"row {idx}: robot over wet patch but in_wet=0", detail_limit)

        event = (row.get("event") or "").strip()
        if event == "collision_static":
            if static_rects and not any(_circle_rect_intersect(x, y, robot_radius, rect) for rect in static_rects):
                _add_issue(res, "collision_static", f"row {idx}: collision_static but no static AABB overlap at ({x:.2f},{y:.2f})", detail_limit)
            elif not static_rects:
                _add_issue(res, "collision_static", f"row {idx}: collision_static but static geometry missing in world", detail_limit)
        elif event == "collision_human":
            humans = [r for r in actors if r["type"] == "human"]
            if humans:
                human_r = 0.35
                if human_r_map:
                    human_r = max(human_r_map.values())
                tol = max(0.15, _dt_median(times) * 2.5)
                matched = False
                for h in humans:
                    if abs(h["t"] - t) > tol:
                        continue
                    hr = human_r_map.get(h["actor_id"], human_r)
                    if math.hypot(h["x"] - x, h["y"] - y) <= (robot_radius + hr + 0.05):
                        matched = True
                        break
                if not matched:
                    _add_issue(res, "collision_human", f"row {idx}: collision_human but no nearby actor within {tol:.2f}s window", detail_limit)
            else:
                _add_issue(res, "collision_human", f"row {idx}: collision_human but actors.csv missing/empty", detail_limit)
        elif event == "floor_slip":
            loc = None
            detail = row.get("event_detail", "")
            if detail and isinstance(detail, str):
                parts = detail.split(",")
                if len(parts) >= 2:
                    try:
                        loc = (float(parts[0]), float(parts[1]))
                    except Exception:
                        loc = None
            if loc is None:
                loc = (x, y)
            mu_here = _mu_at(world, loc[0], loc[1], default=0.9)
            if mu_here > 0.75:
                _add_issue(res, "slip_mu", f"row {idx}: floor_slip at ({loc[0]:.2f},{loc[1]:.2f}) mu={mu_here:.2f}", detail_limit)

    if len(times) >= 2:
        median_dt = _dt_median(times)
        if median_dt <= 0.0:
            _add_issue(res, "timestamps", "unable to compute positive timestep median", detail_limit)

    # Robot start validity
    if first_pose:
        x0, y0 = first_pose
        if bounds != (0.0, 0.0):
            if x0 < 0.0 or y0 < 0.0 or x0 > bounds[0] or y0 > bounds[1]:
                _add_issue(res, "robot_spawn", f"start ({x0:.2f},{y0:.2f}) outside map {bounds}", detail_limit)
        for rect in static_rects:
            if _circle_rect_intersect(x0, y0, robot_radius, rect):
                _add_issue(res, "robot_spawn", f"start ({x0:.2f},{y0:.2f}) overlaps static AABB {rect}", detail_limit)

    # Static overlap sanity (only named obstacles, skip perimeter aabbs)
    obstacle_rects = [_normalize_rect(obj.get("aabb", [])) for obj in (world.get("static_obstacles", []) if world else []) or []]
    obstacle_rects = [r for r in obstacle_rects if r]
    for i in range(len(obstacle_rects)):
        for j in range(i + 1, len(obstacle_rects)):
            area = _overlap_area(obstacle_rects[i], obstacle_rects[j])
            if area > 1e-3:
                _add_issue(res, "static_overlap", f"static AABBs {i} and {j} overlap (area {area:.4f})", detail_limit)
                break

    # LiDAR quick checks (shape / finite / range / alignment)
    lidar_path = run_dir / "lidar.npz"
    if lidar_path.exists():
        _check_lidar(
            lidar_path,
            res,
            times,
            median_dt,
            scn,
            detail_limit,
        )

    return res


def _print_result(res: RunResult) -> None:
    name = res.run_dir.name
    if res.total_anomalies == 0 and not res.missing:
        print(f"[OK] {name}: no anomalies detected")
        return
    summary_parts = []
    if res.missing:
        summary_parts.append(f"missing={','.join(res.missing)}")
    if res.counts:
        summary_parts.append(", ".join(f"{k}={v}" for k, v in sorted(res.counts.items())))
    print(f"[WARN] {name}: " + "; ".join(summary_parts))
    for cat in sorted(res.details.keys()):
        for msg in res.details[cat]:
            print(f"  - ({cat}) {msg}")


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="EdgeSim data quality checks")
    ap.add_argument("run_dirs", nargs="+", type=Path, help="Run folder(s) or batch folder containing per_run/")
    ap.add_argument("--radius", type=float, default=None, help="Override robot radius for wet/collision checks")
    ap.add_argument("--max-details", type=int, default=5, help="Max detailed messages per category")
    args = ap.parse_args(argv)

    all_runs: List[Path] = []
    for rd in args.run_dirs:
        collected = _iter_run_dirs(rd)
        if not collected:
            print(f"[WARN] {rd}: no run_one.csv found (expected run dir or per_run/run_XXXX)")
        all_runs.extend(collected)
    if not all_runs:
        print("No runs to check.")
        return 1

    aggregate: Dict[str, int] = {}
    for run in all_runs:
        res = check_run(run, radius_override=args.radius, detail_limit=args.max_details)
        _print_result(res)
        for k, v in res.counts.items():
            aggregate[k] = aggregate.get(k, 0) + v

    if len(all_runs) > 1:
        print(f"\nChecked {len(all_runs)} runs.")
        if aggregate:
            agg_summary = ", ".join(f"{k}={v}" for k, v in sorted(aggregate.items()))
            print(f"Aggregate anomalies: {agg_summary}")
        else:
            print("Aggregate anomalies: none")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

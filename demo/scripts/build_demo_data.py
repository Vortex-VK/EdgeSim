#!/usr/bin/env python3
"""Generate static demo data from the current runs/ directory.

This script does two things:
1) Builds a compact JSON payload used by the demo frontend.
2) Copies selected artifacts into demo/public/artifacts for direct download/view.
"""

from __future__ import annotations

import ast
import csv
import json
import math
import os
import re
import shutil
import struct
import zipfile
from datetime import datetime, timezone
from pathlib import Path
from statistics import mean
from typing import Any, Iterable

try:
    import yaml
except Exception:
    yaml = None

RUN_DIR_RE = re.compile(r"^(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})_(.+)$")

TITLE_OVERRIDES = {
    "main_aisle_from_210_to_1810_wall_from_28": {
        "title": "Wall + Wet Patch With Tugger",
        "summary": "Single-lane aisle with a wall constraint, tugger traffic, and a wet patch near the crossing line.",
    },
    "main_aisle_from_210_to_1810_rack_from_26": {
        "title": "Forklift Through Rack Chicane",
        "summary": "Dense rack geometry pushes the AMR into tight-clearance interactions with a crossing human and forklift.",
    },
    "main_aisle_from_210_to_14510_rack_from_1": {
        "title": "Wet Crossing Near Rack Block",
        "summary": "Shorter main aisle with a wet zone and a vertical human crossing lane near rack boundaries.",
    },
    "main_aisle_from_210_to_1710_forklift_mov": {
        "title": "Dual-Forklift Cross Traffic",
        "summary": "Two opposing forklifts plus a crossing human create persistent conflict inside the shared aisle.",
    },
}

TEST_ARTIFACTS = [
    "scenario.yaml",
    "world.json",
    "run_one.csv",
    "lidar.npz",
    "actors.csv",
    "validation.json",
    "dataset_manifest.json",
    "report.html",
]

BATCH_ARTIFACTS = [
    "scenario.yaml",
    "summary.json",
    "coverage.json",
    "perf.json",
    "dataset_manifest.json",
    "report.html",
    "report.md",
    "per_run/index.csv",
]

Rect = tuple[float, float, float, float]


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def read_yaml(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}

    force_no_yaml = str(os.environ.get("EDGESIM_DEMO_NO_YAML", "")).strip().lower() in {"1", "true", "yes", "on"}
    if yaml is None or force_no_yaml:
        return {}

    try:
        obj = yaml.safe_load(path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    return obj if isinstance(obj, dict) else {}


def parse_csv(path: Path) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    with path.open("r", newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            rows.append(dict(row))
    return rows


def as_float(value: Any, default: float = 0.0) -> float:
    try:
        if value is None:
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def as_int(value: Any, default: int = 0) -> int:
    try:
        if value is None:
            return default
        return int(value)
    except (TypeError, ValueError):
        return default


def as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "y"}


def round_or_none(value: float | None, ndigits: int = 3) -> float | None:
    if value is None:
        return None
    if not math.isfinite(value):
        return None
    return round(value, ndigits)


def normalize_rect(rect: Iterable[Any]) -> list[float] | None:
    vals = [as_float(v, float("nan")) for v in rect]
    if len(vals) != 4 or any(not math.isfinite(v) for v in vals):
        return None
    x0, y0, x1, y1 = vals
    if x1 < x0:
        x0, x1 = x1, x0
    if y1 < y0:
        y0, y1 = y1, y0
    return [round(x0, 3), round(y0, 3), round(x1, 3), round(y1, 3)]


def parse_timestamp(ts_raw: str) -> str:
    return datetime.strptime(ts_raw, "%Y-%m-%d_%H-%M-%S").strftime("%Y-%m-%d %H:%M:%S")


def quantile(values: list[float], q: float) -> float:
    if not values:
        return 0.0
    sorted_vals = sorted(values)
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    pos = q * (len(sorted_vals) - 1)
    low = int(math.floor(pos))
    high = int(math.ceil(pos))
    if low == high:
        return sorted_vals[low]
    frac = pos - low
    return sorted_vals[low] * (1.0 - frac) + sorted_vals[high] * frac


def lidar_extrinsics_from_frames(path: Path) -> tuple[float, float, float]:
    if not path.exists():
        return (0.0, 0.0, 0.0)
    try:
        obj = read_json(path)
    except Exception:
        return (0.0, 0.0, 0.0)

    frames = obj.get("frames", []) if isinstance(obj, dict) else []
    for frame in frames:
        if not isinstance(frame, dict) or str(frame.get("child")) != "laser":
            continue
        xyz = frame.get("xyz")
        rpy = frame.get("rpy")
        lx = as_float(xyz[0], 0.0) if isinstance(xyz, (list, tuple)) and len(xyz) >= 1 else 0.0
        ly = as_float(xyz[1], 0.0) if isinstance(xyz, (list, tuple)) and len(xyz) >= 2 else 0.0
        lyaw = as_float(rpy[2], 0.0) if isinstance(rpy, (list, tuple)) and len(rpy) >= 3 else 0.0
        return (lx, ly, lyaw)
    return (0.0, 0.0, 0.0)


def parse_npy_float_array(blob: bytes) -> tuple[tuple[int, ...], list[float]] | None:
    if len(blob) < 12 or not blob.startswith(b"\x93NUMPY"):
        return None

    major = blob[6]
    if major == 1:
        if len(blob) < 10:
            return None
        header_len = struct.unpack("<H", blob[8:10])[0]
        header_start = 10
    elif major in {2, 3}:
        if len(blob) < 12:
            return None
        header_len = struct.unpack("<I", blob[8:12])[0]
        header_start = 12
    else:
        return None

    header_end = header_start + header_len
    if header_end > len(blob):
        return None

    try:
        header = ast.literal_eval(blob[header_start:header_end].decode("latin1").strip())
    except Exception:
        return None

    if not isinstance(header, dict):
        return None

    descr = str(header.get("descr", ""))
    shape_raw = header.get("shape")
    fortran_order = bool(header.get("fortran_order"))
    if fortran_order or not isinstance(shape_raw, tuple):
        return None

    shape: list[int] = []
    for dim in shape_raw:
        if not isinstance(dim, int) or dim < 0:
            return None
        shape.append(dim)

    count = 1
    for dim in shape:
        count *= dim

    if descr.endswith("f4"):
        item_size = 4
        fmt_char = "f"
    elif descr.endswith("f8"):
        item_size = 8
        fmt_char = "d"
    else:
        return None

    payload = blob[header_end:]
    needed = count * item_size
    if needed > len(payload):
        return None

    try:
        values = list(struct.unpack(f"<{count}{fmt_char}", payload[:needed]))
    except struct.error:
        return None

    return tuple(shape), values


def load_lidar_arrays(npz_path: Path) -> dict[str, Any] | None:
    if not npz_path.exists():
        return None

    try:
        with zipfile.ZipFile(npz_path, "r") as zf:
            parsed: dict[str, tuple[tuple[int, ...], list[float]]] = {}
            for key in ("t", "pose", "ranges", "angles"):
                member = f"{key}.npy"
                if member not in zf.namelist():
                    return None
                arr = parse_npy_float_array(zf.read(member))
                if arr is None:
                    return None
                parsed[key] = arr
    except Exception:
        return None

    t_shape, t_vals = parsed["t"]
    pose_shape, pose_vals = parsed["pose"]
    ranges_shape, ranges_vals = parsed["ranges"]
    angles_shape, angles_vals = parsed["angles"]

    if len(t_shape) != 1 or len(pose_shape) != 2 or len(ranges_shape) != 2 or len(angles_shape) != 1:
        return None

    frame_count = t_shape[0]
    beam_count = angles_shape[0]
    if pose_shape[0] != frame_count or pose_shape[1] < 3:
        return None
    if ranges_shape[0] != frame_count or ranges_shape[1] != beam_count:
        return None

    return {
        "t": t_vals,
        "pose": pose_vals,
        "ranges": ranges_vals,
        "angles": angles_vals,
        "frame_count": frame_count,
        "beam_count": beam_count,
    }


def lidar_playback(npz_path: Path, max_range_m: float, laser_xy: tuple[float, float], laser_yaw: float) -> dict[str, Any]:
    arrays = load_lidar_arrays(npz_path)
    if not arrays:
        return {
            "available": False,
            "source_frames": 0,
            "frame_count": 0,
            "beam_count": 0,
            "frame_stride": 0,
            "ray_stride": 0,
            "frames": [],
        }

    frame_count = as_int(arrays["frame_count"], 0)
    beam_count = as_int(arrays["beam_count"], 0)
    t_vals = arrays["t"]
    pose_vals = arrays["pose"]
    ranges_vals = arrays["ranges"]
    angle_vals = arrays["angles"]

    frame_stride = max(1, math.ceil(frame_count / 120)) if frame_count else 1
    ray_stride = max(1, math.ceil(beam_count / 72)) if beam_count else 1
    max_range = max_range_m if math.isfinite(max_range_m) and max_range_m > 0 else None

    sampled_frame_indices = list(range(0, frame_count, frame_stride))
    if frame_count > 0 and sampled_frame_indices and sampled_frame_indices[-1] != frame_count - 1:
        sampled_frame_indices.append(frame_count - 1)

    frames: list[dict[str, Any]] = []
    for frame_idx in sampled_frame_indices:
        pose_base = frame_idx * 3
        if pose_base + 2 >= len(pose_vals):
            continue

        x = pose_vals[pose_base]
        y = pose_vals[pose_base + 1]
        yaw = pose_vals[pose_base + 2]
        laser_x = x + (laser_xy[0] * math.cos(yaw) - laser_xy[1] * math.sin(yaw))
        laser_y = y + (laser_xy[0] * math.sin(yaw) + laser_xy[1] * math.cos(yaw))
        t = t_vals[frame_idx] if frame_idx < len(t_vals) else 0.0

        points: list[list[float]] = []
        range_row_base = frame_idx * beam_count
        for beam_idx in range(0, beam_count, ray_stride):
            range_idx = range_row_base + beam_idx
            if range_idx >= len(ranges_vals) or beam_idx >= len(angle_vals):
                continue
            distance = ranges_vals[range_idx]
            if not math.isfinite(distance) or distance <= 0:
                continue

            if max_range is not None:
                distance = min(distance, max_range)

            ray_angle = yaw + laser_yaw + angle_vals[beam_idx]
            point_x = laser_x + distance * math.cos(ray_angle)
            point_y = laser_y + distance * math.sin(ray_angle)
            if not (math.isfinite(point_x) and math.isfinite(point_y)):
                continue
            points.append([round(point_x, 3), round(point_y, 3)])

        frames.append(
            {
                "t": round(t, 2),
                "pose": [round(x, 3), round(y, 3), round(yaw, 3)],
                "laser_origin": [round(laser_x, 3), round(laser_y, 3)],
                "points": points,
            }
        )

    return {
        "available": len(frames) > 0,
        "source_frames": frame_count,
        "frame_count": len(frames),
        "beam_count": beam_count,
        "frame_stride": frame_stride,
        "ray_stride": ray_stride,
        "frames": frames,
    }


def last_nonempty_event(rows: list[dict[str, str]]) -> str:
    for row in reversed(rows):
        event = (row.get("event") or "").strip()
        if event:
            return event
    return "none"


def list_run_dirs(runs_root: Path) -> list[dict[str, Any]]:
    entries: list[dict[str, Any]] = []
    for child in sorted(runs_root.iterdir()):
        if not child.is_dir() or child.name.startswith("_"):
            continue
        match = RUN_DIR_RE.match(child.name)
        if not match:
            continue
        ts_raw, slug = match.groups()
        entries.append(
            {
                "name": child.name,
                "slug": slug,
                "timestamp": ts_raw,
                "timestamp_human": parse_timestamp(ts_raw),
                "path": child,
                "is_batch": (child / "summary.json").exists() and (child / "per_run" / "index.csv").exists(),
                "is_test": (child / "run_one.csv").exists(),
            }
        )
    return entries


def pair_runs(entries: list[dict[str, Any]]) -> list[dict[str, Any]]:
    grouped: dict[str, list[dict[str, Any]]] = {}
    for entry in entries:
        grouped.setdefault(entry["slug"], []).append(entry)

    pairs: list[dict[str, Any]] = []
    for slug, items in grouped.items():
        tests = sorted([item for item in items if item["is_test"]], key=lambda item: item["timestamp"])
        batches = sorted([item for item in items if item["is_batch"]], key=lambda item: item["timestamp"])
        if not tests or not batches:
            continue
        pairs.append({"slug": slug, "test": tests[0], "batch": batches[0]})

    pairs.sort(key=lambda pair: pair["test"]["timestamp"])
    return pairs


def sanitize_zone_list(objs: Iterable[dict[str, Any]], key: str) -> list[list[float]]:
    out: list[list[float]] = []
    for obj in objs:
        if not isinstance(obj, dict):
            continue
        rect = normalize_rect(obj.get(key, []))
        if rect is not None:
            out.append(rect)
    return out


def rects_overlap_or_touch(a: Rect, b: Rect, gap: float = 0.0) -> bool:
    return not (
        a[2] + gap < b[0]
        or b[2] + gap < a[0]
        or a[3] + gap < b[1]
        or b[3] + gap < a[1]
    )


def merge_rect_clusters(rects: list[Rect], gap: float = 0.25) -> list[list[float]]:
    if not rects:
        return []

    remaining = list(rects)
    merged: list[list[float]] = []
    while remaining:
        seed = remaining.pop(0)
        cluster = [seed]
        changed = True
        while changed:
            changed = False
            still_remaining: list[Rect] = []
            for candidate in remaining:
                if any(rects_overlap_or_touch(candidate, item, gap) for item in cluster):
                    cluster.append(candidate)
                    changed = True
                else:
                    still_remaining.append(candidate)
            remaining = still_remaining

        x0 = min(rect[0] for rect in cluster)
        y0 = min(rect[1] for rect in cluster)
        x1 = max(rect[2] for rect in cluster)
        y1 = max(rect[3] for rect in cluster)
        merged.append([round(x0, 3), round(y0, 3), round(x1, 3), round(y1, 3)])

    merged.sort(key=lambda rect: (rect[0], rect[1], rect[2], rect[3]))
    return merged


def static_rects_by_type(world: dict[str, Any], kind: str) -> list[list[float]]:
    rects: list[list[float]] = []
    for obj in world.get("static_obstacles", []) or []:
        if not isinstance(obj, dict) or str(obj.get("type", "")).lower() != kind:
            continue
        rect = normalize_rect(obj.get("aabb", []))
        if rect is not None:
            rects.append(rect)
    return rects


def world_humans(world: dict[str, Any]) -> list[dict[str, Any]]:
    hazards = world.get("hazards", {}) if isinstance(world.get("hazards"), dict) else {}
    humans: list[dict[str, Any]] = []
    for human in hazards.get("human", []) or []:
        if not isinstance(human, dict):
            continue
        waypoints = [
            [as_float(point[0], 0.0), as_float(point[1], 0.0)]
            for point in (human.get("waypoints") or [])
            if isinstance(point, (list, tuple)) and len(point) >= 2
        ]
        humans.append(
            {
                "waypoints": waypoints,
                "speed_mps": [as_float(v, 0.0) for v in (human.get("speed_mps") or [])[:2]],
            }
        )
    return humans


def world_vehicles(world: dict[str, Any]) -> list[dict[str, Any]]:
    vehicles: list[dict[str, Any]] = []
    for vehicle in world.get("vehicles", []) or []:
        if not isinstance(vehicle, dict):
            continue
        path = [
            [as_float(point[0], 0.0), as_float(point[1], 0.0)]
            for point in (vehicle.get("path") or [])
            if isinstance(point, (list, tuple)) and len(point) >= 2
        ]
        vehicles.append({"type": str(vehicle.get("type", "vehicle")), "path": path})
    return vehicles


def dt_from_rows(rows: list[dict[str, str]], default: float = 0.1) -> float:
    times: list[float] = []
    for row in rows:
        t = as_float(row.get("t"), float("nan"))
        if math.isfinite(t):
            times.append(t)
        if len(times) >= 3:
            break
    if len(times) < 2:
        return default
    dt = times[1] - times[0]
    if not math.isfinite(dt) or dt <= 0:
        return default
    return dt


def duration_from_rows(rows: list[dict[str, str]], default: float = 0.0) -> float:
    if not rows:
        return default
    last = as_float(rows[-1].get("t"), default)
    return last if math.isfinite(last) and last >= 0 else default


def build_timeline(rows: list[dict[str, str]]) -> list[dict[str, Any]]:
    timeline: list[dict[str, Any]] = []
    for row in rows:
        event = (row.get("event") or "").strip()
        near_miss = as_bool(row.get("near_miss"))
        occluded = as_bool(row.get("occluded_hazard"))
        hard_brake = as_bool(row.get("hard_brake"))
        if not event and not near_miss and not occluded and not hard_brake:
            continue
        label = event
        if not label:
            if near_miss:
                label = "near_miss"
            elif occluded:
                label = "occluded_hazard"
            elif hard_brake:
                label = "hard_brake"
        timeline.append(
            {
                "t": round(as_float(row.get("t"), 0.0), 2),
                "label": label,
                "x": round(as_float(row.get("x"), 0.0), 3),
                "y": round(as_float(row.get("y"), 0.0), 3),
            }
        )

    if len(timeline) > 16:
        return timeline[:16]
    return timeline


def downsample_path(rows: list[dict[str, str]], step: int = 2) -> list[dict[str, float]]:
    if not rows:
        return []
    out: list[dict[str, float]] = []
    for idx, row in enumerate(rows):
        is_last = idx == len(rows) - 1
        if idx % step != 0 and not is_last:
            continue
        out.append(
            {
                "t": round(as_float(row.get("t"), 0.0), 2),
                "x": round(as_float(row.get("x"), 0.0), 3),
                "y": round(as_float(row.get("y"), 0.0), 3),
            }
        )
    return out


def actor_tracks(test_dir: Path, max_points_per_actor: int = 240) -> list[dict[str, Any]]:
    actors_path = test_dir / "actors.csv"
    if not actors_path.exists():
        return []

    rows = parse_csv(actors_path)
    grouped: dict[str, dict[str, Any]] = {}
    for row in rows:
        actor_id = str(row.get("actor_id") or row.get("id") or "").strip()
        if not actor_id:
            continue

        t = as_float(row.get("t"), float("nan"))
        x = as_float(row.get("x"), float("nan"))
        y = as_float(row.get("y"), float("nan"))
        yaw = as_float(row.get("yaw"), 0.0)
        if not (math.isfinite(t) and math.isfinite(x) and math.isfinite(y) and math.isfinite(yaw)):
            continue

        actor_type = str(row.get("type") or "actor")
        phase = str(row.get("phase") or "")
        track = grouped.setdefault(
            actor_id,
            {"id": actor_id, "type": actor_type, "samples": []},
        )
        track["samples"].append(
            {
                "t": round(t, 2),
                "x": round(x, 3),
                "y": round(y, 3),
                "yaw": round(yaw, 3),
                "phase": phase,
            }
        )

    output: list[dict[str, Any]] = []
    for actor_id in sorted(grouped.keys()):
        track = grouped[actor_id]
        samples = sorted(track["samples"], key=lambda item: item["t"])
        if not samples:
            continue

        stride = max(1, math.ceil(len(samples) / max_points_per_actor))
        sampled = [sample for idx, sample in enumerate(samples) if idx % stride == 0]
        if sampled[-1]["t"] != samples[-1]["t"]:
            sampled.append(samples[-1])

        output.append(
            {
                "id": track["id"],
                "type": track["type"],
                "samples": sampled,
            }
        )

    return output


def lidar_summary(
    rows: list[dict[str, str]],
    sensors: dict[str, Any],
    manifest: dict[str, Any],
    test_dir: Path,
) -> dict[str, Any]:
    series_raw: list[tuple[float, float]] = []
    for row in rows:
        t = as_float(row.get("t"), float("nan"))
        clearance = as_float(row.get("min_clearance_lidar"), float("nan"))
        if not math.isfinite(t) or not math.isfinite(clearance):
            continue
        if clearance < 0:
            continue
        series_raw.append((t, clearance))

    sample_count = len(series_raw)
    values = [clearance for _, clearance in series_raw]
    step = max(1, sample_count // 120) if sample_count else 1
    sampled_series = [
        {"t": round(t, 2), "clearance_m": round(clearance, 3)}
        for idx, (t, clearance) in enumerate(series_raw)
        if (idx % step == 0) or (idx == sample_count - 1)
    ]

    randomization = manifest.get("randomization", {}) if isinstance(manifest.get("randomization"), dict) else {}
    lidar_cfg = sensors.get("lidar", {}) if isinstance(sensors.get("lidar"), dict) else {}
    log_cfg = lidar_cfg.get("log", {}) if isinstance(lidar_cfg.get("log"), dict) else {}
    npz_exists = (test_dir / "lidar.npz").exists()
    laser_x, laser_y, laser_yaw = lidar_extrinsics_from_frames(test_dir / "frames.json")

    if as_bool(log_cfg.get("full")):
        mode = "full"
    elif as_bool(log_cfg.get("events_only")):
        mode = "events_only"
    else:
        mode = "full" if npz_exists else "none"

    noise_sigma = as_float(lidar_cfg.get("noise_sigma"), as_float(randomization.get("lidar_noise_sigma"), 0.02))
    dropout_pct = as_float(lidar_cfg.get("dropout_pct"), as_float(randomization.get("lidar_dropout_pct"), 0.01))
    max_range = as_float(lidar_cfg.get("max_range_m"), 8.0)

    return {
        "mode": mode,
        "artifact_logged": npz_exists,
        "sample_count": sample_count,
        "min_clearance_m": round_or_none(min(values) if values else None, 3),
        "avg_clearance_m": round_or_none(mean(values) if values else None, 3),
        "p10_clearance_m": round_or_none(quantile(values, 0.10) if values else None, 3),
        "p50_clearance_m": round_or_none(quantile(values, 0.50) if values else None, 3),
        "p90_clearance_m": round_or_none(quantile(values, 0.90) if values else None, 3),
        "noise_sigma": round_or_none(noise_sigma, 4),
        "dropout_pct": round_or_none(dropout_pct, 4),
        "max_range_m": round_or_none(max_range, 2),
        "series": sampled_series,
        "playback": lidar_playback(test_dir / "lidar.npz", max_range, (laser_x, laser_y), laser_yaw),
    }


def copy_artifacts(
    src_root: Path,
    dst_root: Path,
    slug: str,
    kind: str,
    relative_paths: Iterable[str],
) -> dict[str, str]:
    copied: dict[str, str] = {}
    for rel in relative_paths:
        src = src_root / rel
        if not src.exists():
            continue
        dst = dst_root / slug / kind / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
        copied[rel.replace("/", "_")] = f"/artifacts/{slug}/{kind}/{rel}"
    return copied


def build_scenario(pair: dict[str, Any], artifacts_root: Path) -> dict[str, Any]:
    slug = pair["slug"]
    test_dir = pair["test"]["path"]
    batch_dir = pair["batch"]["path"]

    test_scenario = read_yaml(test_dir / "scenario.yaml")
    test_world = read_json(test_dir / "world.json")
    test_manifest = read_json(test_dir / "dataset_manifest.json")
    test_validation = read_json(test_dir / "validation.json")
    test_rows = parse_csv(test_dir / "run_one.csv")

    batch_summary = read_json(batch_dir / "summary.json")
    batch_coverage = read_json(batch_dir / "coverage.json")
    batch_perf = read_json(batch_dir / "perf.json")
    batch_manifest = read_json(batch_dir / "dataset_manifest.json")
    batch_index_rows = parse_csv(batch_dir / "per_run" / "index.csv")

    layout = test_scenario.get("layout", {}) if isinstance(test_scenario.get("layout"), dict) else {}
    geometry = layout.get("geometry", {}) if isinstance(layout.get("geometry"), dict) else {}
    hazards = test_scenario.get("hazards", {}) if isinstance(test_scenario.get("hazards"), dict) else {}
    sensors = test_scenario.get("sensors", {}) if isinstance(test_scenario.get("sensors"), dict) else {}
    runtime = test_scenario.get("runtime", {}) if isinstance(test_scenario.get("runtime"), dict) else {}
    world_hazards_data = test_world.get("hazards", {}) if isinstance(test_world.get("hazards"), dict) else {}

    map_size = layout.get("map_size_m") or test_world.get("environment", {}).get("map_size_m") or [20.0, 20.0]
    start = layout.get("start") or test_world.get("start") or [0.0, 0.0]
    goal = layout.get("goal") or test_world.get("goal") or [0.0, 0.0]

    layout_summary = (
        test_world.get("environment", {}).get("layout_summary", {})
        if isinstance(test_world.get("environment"), dict)
        else {}
    )
    static_summary = layout_summary.get("static_obstacles", {}) if isinstance(layout_summary, dict) else {}

    scenario_aisles = sanitize_zone_list(layout.get("aisles", []), "rect")
    world_aisles = sanitize_zone_list(test_world.get("aisles", []), "zone")
    aisles = scenario_aisles or world_aisles

    scenario_walls = sanitize_zone_list(layout.get("walls", []), "aabb")
    world_walls = static_rects_by_type(test_world, "wall")
    walls = scenario_walls or world_walls

    scenario_racks = sanitize_zone_list(geometry.get("racking", []), "aabb")
    world_rack_rects_raw = static_rects_by_type(test_world, "rack")
    world_racks = merge_rect_clusters([tuple(rect) for rect in world_rack_rects_raw], gap=0.35)
    racks = scenario_racks or world_racks

    scenario_traction = sanitize_zone_list(hazards.get("traction", []), "zone")
    world_traction = sanitize_zone_list(world_hazards_data.get("traction", []), "zone")
    traction = scenario_traction or world_traction

    scenario_humans = [
        {
            "waypoints": [
                [as_float(point[0], 0.0), as_float(point[1], 0.0)]
                for point in human.get("waypoints", [])
                if isinstance(point, (list, tuple)) and len(point) >= 2
            ],
            "speed_mps": [as_float(v, 0.0) for v in human.get("speed_mps", [])[:2]],
        }
        for human in hazards.get("human", [])
        if isinstance(human, dict)
    ]
    humans = scenario_humans or world_humans(test_world)

    scenario_vehicles = [
        {
            "type": str(vehicle.get("type", "vehicle")),
            "path": [
                [as_float(point[0], 0.0), as_float(point[1], 0.0)]
                for point in vehicle.get("path", [])
                if isinstance(point, (list, tuple)) and len(point) >= 2
            ],
        }
        for vehicle in hazards.get("vehicles", [])
        if isinstance(vehicle, dict)
    ]
    vehicles = scenario_vehicles or world_vehicles(test_world)

    test_timeline = build_timeline(test_rows)
    test_path = downsample_path(test_rows, step=2)
    test_actor_tracks = actor_tracks(test_dir)
    final_event = last_nonempty_event(test_rows)
    test_lidar = lidar_summary(test_rows, sensors, test_manifest, test_dir)

    batch_times = [as_float(row.get("time_s"), 0.0) for row in batch_index_rows]
    batch_steps = [as_int(row.get("steps"), 0) for row in batch_index_rows]
    batch_success = [as_bool(row.get("success")) for row in batch_index_rows]

    override = TITLE_OVERRIDES.get(slug, {})
    title = override.get("title") or slug.replace("_", " ").title()
    summary = override.get("summary") or "Scenario generated from prompt-driven warehouse setup."

    test_artifacts = copy_artifacts(test_dir, artifacts_root, slug, "test", TEST_ARTIFACTS)
    batch_artifacts = copy_artifacts(batch_dir, artifacts_root, slug, "batch", BATCH_ARTIFACTS)

    return {
        "id": slug,
        "title": title,
        "summary": summary,
        "prompt": test_manifest.get("prompt", ""),
        "timestamps": {
            "test": pair["test"]["timestamp_human"],
            "batch": pair["batch"]["timestamp_human"],
        },
        "world": {
            "map_size_m": [as_float(map_size[0], 20.0), as_float(map_size[1], 20.0)],
            "start": [as_float(start[0], 0.0), as_float(start[1], 0.0)],
            "goal": [as_float(goal[0], 0.0), as_float(goal[1], 0.0)],
            "aisles": aisles,
            "walls": walls,
            "racks": racks,
            "traction": traction,
            "humans": humans,
            "vehicles": vehicles,
        },
        "test": {
            "outcome": final_event,
            "duration_s": round_or_none(as_float(test_manifest.get("stats", {}).get("duration_avg_s"), 0.0), 3),
            "avg_speed_mps": round_or_none(as_float(test_manifest.get("stats", {}).get("avg_speed_mean_mps"), 0.0), 3),
            "min_ttc_s": round_or_none(as_float(test_manifest.get("stats", {}).get("min_ttc_min_s"), 0.0), 3),
            "events": {
                key: as_int(value)
                for key, value in (test_manifest.get("event_counts", {}) or {}).items()
                if isinstance(key, str)
            },
            "timeline": test_timeline,
            "path": test_path,
            "actors": test_actor_tracks,
            "rows": len(test_rows),
            "lidar": test_lidar,
        },
        "batch": {
            "runs": as_int(batch_summary.get("runs")),
            "successes": as_int(batch_summary.get("successes")),
            "failures": as_int(batch_summary.get("failures")),
            "avg_time_s": round_or_none(as_float(batch_summary.get("avg_time"), 0.0), 3),
            "avg_steps": as_int(batch_summary.get("avg_steps")),
            "coverage_pct": {
                "wet": round_or_none(as_float(batch_coverage.get("traction_pct", {}).get("wet_encountered"), 0.0), 1),
                "tight_clearance": round_or_none(as_float(batch_coverage.get("clearance_bands_pct", {}).get("<0.20"), 0.0), 1),
                "collision_human": round_or_none(as_float(batch_coverage.get("outcomes_pct", {}).get("collision_human"), 0.0), 1),
                "success": round_or_none(as_float(batch_coverage.get("outcomes_pct", {}).get("success"), 0.0), 1),
            },
            "perf": {
                "elapsed_sec": round_or_none(as_float(batch_perf.get("elapsed_sec"), 0.0), 3),
                "sims_per_min": round_or_none(as_float(batch_perf.get("sims_per_min"), 0.0), 2),
                "auto_degraded": as_bool(batch_perf.get("auto_degraded")),
                "profile": str(batch_perf.get("profile", "")),
            },
            "event_counts": {
                key: as_int(value)
                for key, value in (batch_manifest.get("event_counts", {}) or {}).items()
                if isinstance(key, str)
            },
            "time_distribution_s": {
                "min": round_or_none(min(batch_times) if batch_times else 0.0, 3),
                "p10": round_or_none(quantile(batch_times, 0.10), 3),
                "p50": round_or_none(quantile(batch_times, 0.50), 3),
                "p90": round_or_none(quantile(batch_times, 0.90), 3),
                "max": round_or_none(max(batch_times) if batch_times else 0.0, 3),
                "avg": round_or_none(mean(batch_times) if batch_times else 0.0, 3),
            },
            "step_distribution": {
                "min": min(batch_steps) if batch_steps else 0,
                "max": max(batch_steps) if batch_steps else 0,
                "avg": round_or_none(mean(batch_steps) if batch_steps else 0.0, 2),
            },
            "sample_runs": [
                {
                    "run_id": str(row.get("run_id", "")),
                    "success": as_bool(row.get("success")),
                    "time_s": round_or_none(as_float(row.get("time_s"), 0.0), 3),
                    "steps": as_int(row.get("steps"), 0),
                }
                for row in batch_index_rows[:20]
            ],
            "success_curve": [
                {
                    "idx": idx + 1,
                    "success": 1 if ok else 0,
                    "time_s": round_or_none(batch_times[idx], 3),
                }
                for idx, ok in enumerate(batch_success)
            ],
        },
        "validation": {
            "ok": as_bool(test_validation.get("ok")),
            "errors": test_validation.get("errors", []),
        },
        "config": {
            "lidar_hz": round_or_none(as_float(sensors.get("lidar", {}).get("hz"), 10.0), 2),
            "dt": round_or_none(as_float(runtime.get("dt"), dt_from_rows(test_rows, default=0.1)), 3),
            "duration_s": round_or_none(as_float(runtime.get("duration_s"), duration_from_rows(test_rows, default=0.0)), 2),
            "agents": max(len(test_scenario.get("agents", []) or []), 1),
            "humans": len(humans),
            "vehicles": max(len(vehicles), len(world_hazards_data.get("vehicles", []) or [])),
            "traction_patches": len(traction),
            "racks": max(len(scenario_racks), as_int(static_summary.get("rack"), len(racks))),
            "walls": max(len(scenario_walls), as_int(static_summary.get("wall"), len(walls))),
        },
        "artifacts": {
            "test": test_artifacts,
            "batch": batch_artifacts,
        },
    }


def build_global(scenarios: list[dict[str, Any]]) -> dict[str, Any]:
    batch_runs = sum(as_int(scenario["batch"].get("runs")) for scenario in scenarios)
    batch_successes = sum(as_int(scenario["batch"].get("successes")) for scenario in scenarios)
    test_successes = sum(1 for scenario in scenarios if scenario["test"].get("outcome") == "mission_success")

    all_avg_times = [as_float(scenario["batch"].get("avg_time_s"), 0.0) for scenario in scenarios]
    all_sims_per_min = [as_float(scenario["batch"].get("perf", {}).get("sims_per_min"), 0.0) for scenario in scenarios]

    aggregate_events: dict[str, int] = {}
    for scenario in scenarios:
        for key, val in scenario["batch"].get("event_counts", {}).items():
            aggregate_events[key] = aggregate_events.get(key, 0) + as_int(val)

    hardest = min(scenarios, key=lambda item: as_float(item["batch"].get("coverage_pct", {}).get("success"), 100.0)) if scenarios else None
    easiest = max(scenarios, key=lambda item: as_float(item["batch"].get("coverage_pct", {}).get("success"), 0.0)) if scenarios else None

    return {
        "prompt_count": len(scenarios),
        "pair_count": len(scenarios),
        "test_runs": len(scenarios),
        "batch_runs": batch_runs,
        "total_runs": batch_runs + len(scenarios),
        "batch_success_rate_pct": round_or_none((batch_successes / batch_runs * 100.0) if batch_runs else 0.0, 2),
        "test_success_rate_pct": round_or_none((test_successes / len(scenarios) * 100.0) if scenarios else 0.0, 2),
        "avg_batch_time_s": round_or_none(mean(all_avg_times) if all_avg_times else 0.0, 3),
        "avg_sims_per_min": round_or_none(mean(all_sims_per_min) if all_sims_per_min else 0.0, 2),
        "event_totals": aggregate_events,
        "hardest_prompt": hardest["title"] if hardest else None,
        "easiest_prompt": easiest["title"] if easiest else None,
    }


def main() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    runs_root = repo_root / "runs"
    demo_root = repo_root / "demo"

    artifacts_root = demo_root / "public" / "artifacts"
    data_out = demo_root / "src" / "app" / "data" / "demoData.json"

    entries = list_run_dirs(runs_root)
    pairs = pair_runs(entries)

    if not pairs:
        raise RuntimeError("No paired test/batch runs were found under runs/.")

    force_no_yaml = str(os.environ.get("EDGESIM_DEMO_NO_YAML", "")).strip().lower() in {"1", "true", "yes", "on"}
    if yaml is None or force_no_yaml:
        reason = "PyYAML not installed" if yaml is None else "EDGESIM_DEMO_NO_YAML is set"
        print(f"NOTE: scenario.yaml parsing disabled ({reason}); using world.json fallback.")

    if artifacts_root.exists():
        shutil.rmtree(artifacts_root)
    artifacts_root.mkdir(parents=True, exist_ok=True)

    scenarios = [build_scenario(pair, artifacts_root) for pair in pairs]
    payload = {
        "generated_at_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "source": {
            "runs_dir": "runs/",
            "scenario_pairs": len(scenarios),
        },
        "global": build_global(scenarios),
        "scenarios": scenarios,
    }

    data_out.parent.mkdir(parents=True, exist_ok=True)
    data_out.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(f"Wrote {data_out}")
    print(f"Copied artifacts to {artifacts_root}")
    print(f"Scenarios: {len(scenarios)}")


if __name__ == "__main__":
    main()

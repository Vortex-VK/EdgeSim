from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

from tools import check_data_quality as dq


HEADER = [
    "t",
    "x",
    "y",
    "yaw",
    "v_cmd",
    "w_cmd",
    "min_clearance_lidar",
    "min_clearance_geom",
    "event",
    "event_detail",
    "in_wet",
    "human_phase",
    "near_stop",
    "hard_brake",
    "near_miss",
    "occluded_hazard",
    "interaction",
    "sensor_faults",
    "min_ttc",
    "odom_v",
    "odom_w",
    "imu_ax",
    "imu_ay",
    "imu_wz",
]


def _write_world(path: Path, world: dict) -> None:
    path.write_text(json.dumps(world, indent=2), encoding="utf-8")


def _write_run_csv(path: Path, rows: list[list[object]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(HEADER)
        writer.writerows(rows)


def _write_lidar(path: Path, t: list[float], ranges: np.ndarray, angles: np.ndarray | None = None) -> None:
    payload = {
        "t": np.asarray(t, dtype=np.float32),
        "ranges": np.asarray(ranges, dtype=np.float32),
        "pose": np.zeros((len(t), 3), dtype=np.float32),
        "angles": np.asarray(angles if angles is not None else np.linspace(-np.pi, np.pi, ranges.shape[1], endpoint=False), dtype=np.float32),
    }
    np.savez(path, **payload)


def _row(
    t: float,
    x: float,
    y: float,
    *,
    event: str = "",
    event_detail: str = "",
    in_wet: int = 0,
    min_ttc: object = "",
) -> list[object]:
    return [
        t,
        x,
        y,
        0.0,
        0.0,
        0.0,
        1.0,
        1.0,
        event,
        event_detail,
        in_wet,
        "none",
        0,
        0,
        0,
        0,
        "none",
        "",
        min_ttc,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    ]


def test_wet_flag_mismatch(tmp_path: Path) -> None:
    run_dir = tmp_path / "run_ok"
    run_dir.mkdir()
    world = {
        "environment": {"map_size_m": [10.0, 10.0]},
        "floor_zones": [
            {"id": "base", "type": "dry", "zone": [0, 0, 10, 10], "mu": 0.9},
            {"id": "wet", "type": "wet", "zone": [0, 0, 2, 2], "mu": 0.5},
        ],
        "hazards": {"traction": []},
        "aabbs": [],
        "static_obstacles": [],
    }
    _write_world(run_dir / "world.json", world)
    rows = [
        _row(0.0, 1.0, 1.0, in_wet=1),
        _row(1.0, 5.0, 5.0, in_wet=1),
    ]
    _write_run_csv(run_dir / "run_one.csv", rows)

    res = dq.check_run(run_dir)
    assert res.counts.get("wet_flag", 0) == 1
    assert res.total_anomalies >= 1


def test_collision_static_mismatch(tmp_path: Path) -> None:
    run_dir = tmp_path / "run_collision"
    run_dir.mkdir()
    world = {
        "environment": {"map_size_m": [10.0, 10.0]},
        "floor_zones": [{"id": "base", "type": "dry", "zone": [0, 0, 10, 10], "mu": 0.9}],
        "hazards": {"traction": []},
        "aabbs": [],
        "static_obstacles": [{"id": "rack", "type": "rack", "aabb": [1, 1, 2, 2], "height": 2.0}],
    }
    _write_world(run_dir / "world.json", world)
    rows = [_row(0.0, 5.0, 5.0, event="collision_static", event_detail="1")]
    _write_run_csv(run_dir / "run_one.csv", rows)

    res = dq.check_run(run_dir)
    assert res.counts.get("collision_static", 0) == 1


def test_clean_run_has_no_anomalies(tmp_path: Path) -> None:
    run_dir = tmp_path / "run_clean"
    run_dir.mkdir()
    world = {
        "environment": {"map_size_m": [10.0, 10.0]},
        "floor_zones": [{"id": "base", "type": "dry", "zone": [0, 0, 10, 10], "mu": 0.9}],
        "hazards": {"traction": []},
        "aabbs": [],
        "static_obstacles": [],
    }
    _write_world(run_dir / "world.json", world)
    _write_run_csv(run_dir / "run_one.csv", [_row(0.0, 1.0, 1.0)])
    angles = np.linspace(-np.pi, np.pi, 4, endpoint=False)
    ranges = np.ones((1, 4), dtype=np.float32)
    _write_lidar(run_dir / "lidar.npz", [0.0], ranges, angles)

    res = dq.check_run(run_dir)
    assert res.total_anomalies == 0
    assert not res.missing


def test_lidar_flags_nan_and_range(tmp_path: Path) -> None:
    run_dir = tmp_path / "run_lidar_bad"
    run_dir.mkdir()
    world = {
        "environment": {"map_size_m": [10.0, 10.0]},
        "floor_zones": [{"id": "base", "type": "dry", "zone": [0, 0, 10, 10], "mu": 0.9}],
        "hazards": {"traction": []},
        "aabbs": [],
        "static_obstacles": [],
    }
    _write_world(run_dir / "world.json", world)
    _write_run_csv(run_dir / "run_one.csv", [_row(0.0, 1.0, 1.0), _row(0.1, 1.0, 1.1)])
    angles = np.linspace(-np.pi, np.pi, 3, endpoint=False)
    ranges = np.array([[1.0, np.nan, 2.0], [0.2, 12.0, 0.3]], dtype=np.float32)
    _write_lidar(run_dir / "lidar.npz", [0.0, 0.1], ranges, angles)

    res = dq.check_run(run_dir)
    assert res.counts.get("lidar_nan", 0) >= 1
    assert res.counts.get("lidar_range", 0) >= 1

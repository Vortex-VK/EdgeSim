from __future__ import annotations

import argparse
import csv
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import PatchCollection
from matplotlib.patches import Circle, Rectangle
from matplotlib.widgets import Slider

# Simple type aliases
Vec2 = Tuple[float, float]


# ---------- Helpers ----------

def _to_float(val: Any, default: float = math.nan) -> float:
    try:
        if val is None:
            return default
        return float(val)
    except Exception:
        return default


def _parse_bool(val: Any) -> bool:
    if val is None:
        return False
    if isinstance(val, (int, float)):
        return bool(val)
    s = str(val).strip().lower()
    return s in {"1", "true", "yes", "y", "t"}


def _load_json(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _rot2d(xy: np.ndarray, yaw: float) -> np.ndarray:
    """Rotate Nx2 points by yaw radians."""
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s], [s, c]], dtype=np.float32)
    return xy @ rot.T


# ---------- Data containers ----------

@dataclass
class FramesInfo:
    base_frame: str = "base"
    laser_xy: Vec2 = (0.0, 0.0)
    laser_yaw: float = 0.0
    raw: Dict[str, Any] | None = None

    @classmethod
    def from_file(cls, path: Path) -> "FramesInfo":
        if not path.exists():
            return cls()
        try:
            obj = _load_json(path)
        except Exception:
            return cls()
        base = str(obj.get("base_frame") or obj.get("world_frame") or "base")
        laser_xy: Vec2 = (0.0, 0.0)
        laser_yaw = 0.0
        for frame in obj.get("frames", []):
            try:
                child = frame.get("child")
                if child != "laser":
                    continue
                xyz = frame.get("xyz") or [0.0, 0.0, 0.0]
                rpy = frame.get("rpy") or [0.0, 0.0, 0.0]
                laser_xy = (float(xyz[0]), float(xyz[1]))
                laser_yaw = float(rpy[2]) if len(rpy) >= 3 else 0.0
            except Exception:
                continue
        return cls(base_frame=base, laser_xy=laser_xy, laser_yaw=laser_yaw, raw=obj)


@dataclass
class Trajectory:
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    columns: Dict[str, np.ndarray]
    text: Dict[str, List[str]]
    in_wet: np.ndarray

    @property
    def n(self) -> int:
        return len(self.t)


@dataclass
class ActorTrack:
    actor_id: str
    actor_type: str
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    phase: List[str]

    def pose_at(self, t: float) -> Tuple[float, float, float] | None:
        """Return the pose at or before time t."""
        if len(self.t) == 0:
            return None
        idx = np.searchsorted(self.t, t, side="right") - 1
        if idx < 0:
            idx = 0
        idx = min(idx, len(self.t) - 1)
        return float(self.x[idx]), float(self.y[idx]), float(self.yaw[idx])


@dataclass
class LidarData:
    t: np.ndarray
    pose: Optional[np.ndarray]
    ranges: Optional[np.ndarray]
    angles: Optional[np.ndarray]
    xyz: Optional[np.ndarray]
    points_world: Optional[np.ndarray]
    laser_xy: Vec2
    laser_yaw: float

    def __len__(self) -> int:
        if self.ranges is not None:
            return self.ranges.shape[0]
        if self.xyz is not None:
            return self.xyz.shape[0]
        if self.points_world is not None:
            return self.points_world.shape[0]
        return 0

    def points_for_index(self, idx: int, default_pose: Tuple[float, float, float]) -> np.ndarray:
        """Return lidar points in world XY for a frame index."""
        if len(self) == 0:
            return np.empty((0, 2), dtype=np.float32)
        idx = max(0, min(idx, len(self) - 1))
        if self.points_world is not None:
            pts = self.points_world[idx]
            return np.asarray(pts[:, :2], dtype=np.float32)

        base_pose = default_pose
        if self.pose is not None and len(self.pose) > idx:
            try:
                px, py, pyaw = self.pose[idx]
                base_pose = (float(px), float(py), float(pyaw))
            except Exception:
                pass

        if self.xyz is not None:
            local = np.asarray(self.xyz[idx], dtype=np.float32)
            pts = local[:, :2]
        elif self.ranges is not None and self.angles is not None:
            rng = np.asarray(self.ranges[idx], dtype=np.float32)
            ang = np.asarray(self.angles, dtype=np.float32)
            pts = np.stack([rng * np.cos(ang), rng * np.sin(ang)], axis=1)
        else:
            return np.empty((0, 2), dtype=np.float32)

        # Apply laser extrinsics then base pose
        pts = _rot2d(pts, self.laser_yaw)
        pts = pts + np.array(self.laser_xy, dtype=np.float32)
        pts = _rot2d(pts, base_pose[2])
        pts = pts + np.array([base_pose[0], base_pose[1]], dtype=np.float32)
        return pts


@dataclass
class RunData:
    run_dir: Path
    world: Dict[str, Any]
    frames: FramesInfo
    traj: Trajectory
    actors: Dict[str, ActorTrack]
    lidar: Optional[LidarData]


# ---------- Loaders ----------

class RunLoader:
    def __init__(self, run_dir: Path):
        self.run_dir = run_dir

    def load(self, include_lidar: bool = True) -> RunData:
        world = self._load_world()
        frames = FramesInfo.from_file(self.run_dir / "frames.json")
        traj = self._load_trajectory()
        actors = self._load_actors()
        lidar = self._load_lidar(frames) if include_lidar else None
        return RunData(run_dir=self.run_dir, world=world, frames=frames, traj=traj, actors=actors, lidar=lidar)

    def _load_world(self) -> Dict[str, Any]:
        path = self.run_dir / "world.json"
        if not path.exists():
            raise FileNotFoundError(f"world.json not found in {self.run_dir}")
        return _load_json(path)

    def _load_trajectory(self) -> Trajectory:
        path = self.run_dir / "run_one.csv"
        if not path.exists():
            raise FileNotFoundError(f"run_one.csv not found in {self.run_dir}")

        with path.open("r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            rows = list(reader)
        if not rows:
            raise ValueError(f"run_one.csv empty in {self.run_dir}")

        raw_cols: Dict[str, List[str]] = {name: [] for name in reader.fieldnames or []}
        for row in rows:
            for k, v in row.items():
                raw_cols.setdefault(k, []).append(v)

        def _col(name: str) -> List[str]:
            return raw_cols.get(name, [])

        t = np.array([_to_float(v, 0.0) for v in _col("t")], dtype=np.float32)
        x = np.array([_to_float(v, 0.0) for v in _col("x")], dtype=np.float32)
        y = np.array([_to_float(v, 0.0) for v in _col("y")], dtype=np.float32)
        yaw = np.array([_to_float(v, 0.0) for v in _col("yaw")], dtype=np.float32)

        text_cols = {"event", "event_detail", "human_phase", "interaction", "sensor_faults"}
        text: Dict[str, List[str]] = {}
        columns: Dict[str, np.ndarray] = {}
        in_wet_raw = _col("in_wet")
        in_wet = np.array([_parse_bool(v) for v in in_wet_raw], dtype=bool) if in_wet_raw else np.zeros_like(t, dtype=bool)

        for name, vals in raw_cols.items():
            if name in text_cols:
                text[name] = [str(v or "") for v in vals]
                continue
            if name == "in_wet":
                continue
            columns[name] = np.array([_to_float(v) for v in vals], dtype=np.float32)

        required_missing = [c for c in ["t", "x", "y", "yaw"] if c not in raw_cols]
        if required_missing:
            raise ValueError(f"run_one.csv missing required columns: {required_missing}")

        return Trajectory(t=t, x=x, y=y, yaw=yaw, columns=columns, text=text, in_wet=in_wet)

    def _load_actors(self) -> Dict[str, ActorTrack]:
        path = self.run_dir / "actors.csv"
        if not path.exists():
            return {}
        with path.open("r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            rows = list(reader)
        tracks: Dict[str, Dict[str, Any]] = {}
        for row in rows:
            aid = str(row.get("actor_id") or row.get("id") or "").strip()
            if not aid:
                continue
            atype = str(row.get("type") or "actor")
            track = tracks.setdefault(aid, {"actor_id": aid, "actor_type": atype, "t": [], "x": [], "y": [], "yaw": [], "phase": []})
            track["t"].append(_to_float(row.get("t"), 0.0))
            track["x"].append(_to_float(row.get("x"), 0.0))
            track["y"].append(_to_float(row.get("y"), 0.0))
            track["yaw"].append(_to_float(row.get("yaw"), 0.0))
            track["phase"].append(str(row.get("phase") or ""))
            if "type" not in track:
                track["actor_type"] = atype
        out: Dict[str, ActorTrack] = {}
        for aid, data in tracks.items():
            out[aid] = ActorTrack(
                actor_id=aid,
                actor_type=str(data.get("actor_type") or "actor"),
                t=np.array(data.get("t") or [], dtype=np.float32),
                x=np.array(data.get("x") or [], dtype=np.float32),
                y=np.array(data.get("y") or [], dtype=np.float32),
                yaw=np.array(data.get("yaw") or [], dtype=np.float32),
                phase=data.get("phase") or [],
            )
        return out

    def _load_lidar(self, frames: FramesInfo) -> Optional[LidarData]:
        path = self.run_dir / "lidar.npz"
        if not path.exists():
            return None
        try:
            npz = np.load(path)
        except Exception as e:
            print(f"[edgesim viewer] Failed to load lidar.npz: {e}")
            return None
        t = np.asarray(npz["t"], dtype=np.float32) if "t" in npz else np.array([], dtype=np.float32)
        pose = np.asarray(npz["pose"], dtype=np.float32) if "pose" in npz else None
        ranges = np.asarray(npz["ranges"], dtype=np.float32) if "ranges" in npz else None
        angles = np.asarray(npz["angles"], dtype=np.float32) if "angles" in npz else None
        xyz = np.asarray(npz["xyz"], dtype=np.float32) if "xyz" in npz else None
        points_world = np.asarray(npz["points_world"], dtype=np.float32) if "points_world" in npz else None
        return LidarData(
            t=t,
            pose=pose,
            ranges=ranges,
            angles=angles,
            xyz=xyz,
            points_world=points_world,
            laser_xy=frames.laser_xy,
            laser_yaw=frames.laser_yaw,
        )


# ---------- Timeline ----------

class TimelineIndex:
    def __init__(self, run_t: np.ndarray, lidar_t: Optional[np.ndarray] = None,
                 start: int = 0, end: int = -1, step: int = 1):
        n = len(run_t)
        if end < 0 or end >= n:
            end = n - 1
        start = max(0, min(start, n - 1))
        if end < start:
            end = start
        step = max(1, step)
        self.frame_indices = np.arange(start, end + 1, step, dtype=int)
        self.t = run_t[self.frame_indices]
        self.lidar_map = self._map_lidar(self.t, lidar_t) if lidar_t is not None and len(lidar_t) else None

    @staticmethod
    def _map_lidar(run_t: np.ndarray, lidar_t: np.ndarray) -> np.ndarray:
        if len(lidar_t) == 0:
            return np.array([], dtype=int)
        lidar_t = np.asarray(lidar_t, dtype=np.float32)
        idx = np.searchsorted(lidar_t, run_t, side="left")
        out = np.zeros_like(run_t, dtype=int)
        for i, j in enumerate(idx):
            if j <= 0:
                out[i] = 0
            elif j >= len(lidar_t):
                out[i] = len(lidar_t) - 1
            else:
                prev = j - 1
                pick = j if abs(lidar_t[j] - run_t[i]) < abs(lidar_t[prev] - run_t[i]) else prev
                out[i] = pick
        return out

    def clamp_index(self, i: int) -> int:
        if i < 0:
            return 0
        if i >= len(self.frame_indices):
            return len(self.frame_indices) - 1
        return i


# ---------- Rendering ----------

class WorldRenderer2D:
    def __init__(self, world: Dict[str, Any]):
        self.world = world
        self.patches: List[Rectangle] = []
        self.wet_patches: List[Rectangle] = []
        self.aabb = self._compute_bounds()

    def _compute_bounds(self) -> Tuple[float, float, float, float]:
        xs: List[float] = []
        ys: List[float] = []
        env = self.world.get("environment") or {}
        map_size = env.get("map_size_m")
        if isinstance(map_size, (list, tuple)) and len(map_size) >= 2:
            try:
                xs.extend([0.0, float(map_size[0])])
                ys.extend([0.0, float(map_size[1])])
            except Exception:
                pass
        for aabb in self.world.get("aabbs", []):
            try:
                x0, y0, x1, y1 = aabb
                xs.extend([float(x0), float(x1)])
                ys.extend([float(y0), float(y1)])
            except Exception:
                continue
        for pt in (self.world.get("start"), self.world.get("goal")):
            if isinstance(pt, (list, tuple)) and len(pt) == 2:
                try:
                    xs.append(float(pt[0])); ys.append(float(pt[1]))
                except Exception:
                    pass
        if not xs or not ys:
            return (0.0, 0.0, 10.0, 10.0)
        return (min(xs), min(ys), max(xs), max(ys))

    def add_to_axes(self, ax: plt.Axes) -> None:
        patches: List[Rectangle] = []
        wet: List[Rectangle] = []
        # Floor zones (wet/traction)
        for zone in self.world.get("floor_zones", []):
            rect = self._rect_from_zone(zone, default_color=(0.85, 0.85, 0.85, 0.2))
            if rect:
                if str(zone.get("type", "")).lower() in {"wet", "oil", "cleaning_liquid"}:
                    rect.set_facecolor((0.2, 0.6, 1.0, 0.25))
                    wet.append(rect)
                patches.append(rect)
        # Aisles
        for aisle in self.world.get("aisles", []):
            rect = self._rect_from_zone(aisle, default_color=(0.9, 0.9, 0.9, 0.2))
            if rect:
                patches.append(rect)
        # Static obstacles
        for obj in self.world.get("static_obstacles", []):
            rect = self._rect_from_aabb(obj, default_color=(0.2, 0.2, 0.2, 0.6))
            if rect:
                patches.append(rect)

        if patches:
            pc = PatchCollection(patches, match_original=True)
            ax.add_collection(pc)
        if wet:
            self.wet_patches = wet
        self.patches = patches

        # Start/goal markers
        start = self.world.get("start")
        goal = self.world.get("goal")
        if isinstance(start, (list, tuple)) and len(start) == 2:
            ax.plot(start[0], start[1], marker="o", color="green", markersize=6, label="start")
        if isinstance(goal, (list, tuple)) and len(goal) == 2:
            ax.plot(goal[0], goal[1], marker="*", color="red", markersize=7, label="goal")
        handles, labels = ax.get_legend_handles_labels()
        if handles:
            ax.legend(loc="upper right", fontsize=8, framealpha=0.4)

    def _rect_from_zone(self, zone: Dict[str, Any], default_color: Tuple[float, float, float, float]) -> Optional[Rectangle]:
        if not isinstance(zone, dict):
            return None
        pts = zone.get("zone") or zone.get("aabb")
        if not pts or len(pts) != 4:
            return None
        x0, y0, x1, y1 = map(float, pts)
        center = zone.get("center")
        half = zone.get("half_extents")
        yaw = float(zone.get("yaw", 0.0) or 0.0)
        if center and half:
            cx, cy = center
            hx, hy = half[0], half[1]
            w, h = 2.0 * hx, 2.0 * hy
            rect = Rectangle((cx - hx, cy - hy), w, h, angle=math.degrees(yaw), facecolor=default_color, edgecolor=(0.1, 0.1, 0.1, 0.2))
            return rect
        rect = Rectangle((x0, y0), x1 - x0, y1 - y0, angle=math.degrees(yaw), facecolor=default_color, edgecolor=(0.1, 0.1, 0.1, 0.25))
        return rect

    def _rect_from_aabb(self, obj: Dict[str, Any], default_color: Tuple[float, float, float, float]) -> Optional[Rectangle]:
        if not isinstance(obj, dict):
            return None
        aabb = obj.get("aabb")
        if not aabb or len(aabb) != 4:
            return None
        x0, y0, x1, y1 = map(float, aabb)
        center = obj.get("center")
        half = obj.get("half_extents")
        yaw = float(obj.get("yaw", 0.0) or 0.0)
        fc = default_color
        t = str(obj.get("type") or "")
        if t == "rack":
            fc = (0.6, 0.4, 0.2, 0.7)
        elif t == "wall":
            fc = (0.1, 0.1, 0.1, 0.7)
        elif t == "endcap":
            fc = (0.8, 0.2, 0.2, 0.7)
        if center and half:
            cx, cy = center
            hx, hy = half[0], half[1]
            return Rectangle((cx - hx, cy - hy), 2.0 * hx, 2.0 * hy, angle=math.degrees(yaw), facecolor=fc, edgecolor=(0.1, 0.1, 0.1, 0.6))
        return Rectangle((x0, y0), x1 - x0, y1 - y0, angle=math.degrees(yaw), facecolor=fc, edgecolor=(0.1, 0.1, 0.1, 0.6))


# ---------- Viewer ----------

class Viewer2DApp:
    def __init__(self, run: RunData, follow_robot: bool = False, show_lidar: bool = True,
                 color_mode: str = "type", fps: float = 30.0, step: int = 1,
                 start: int = 0, end: int = -1):
        self.run = run
        self.follow_robot = follow_robot
        self.show_lidar = show_lidar and (run.lidar is not None)
        self.color_mode = color_mode
        self.fps = max(1.0, fps)
        self.timeline = TimelineIndex(run.traj.t, run.lidar.t if run.lidar is not None else None,
                                      start=start, end=end, step=step)
        if len(self.timeline.frame_indices) == 0:
            raise ValueError("No frames to replay (run_one.csv empty?)")
        self.frame_idx = 0
        self.playing = True
        self.playback_clock = 0.0
        self.last_wall_time = time.monotonic()
        self.play_anchor_t = float(self.timeline.t[0]) if len(self.timeline.t) else 0.0
        self._suppress_slider = False

        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        plt.subplots_adjust(bottom=0.14)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_title(f"EdgeSim Replay @ {run.run_dir.name}")

        # No world/actors in LiDAR-only mode; camera fits trajectory
        self.world_bounds = self._traj_bounds()
        self.camera_center = self._initial_center()
        self.camera_span = self._initial_span()

        # Artists
        self.robot_patch = Circle((0, 0), radius=self._robot_radius(), color="orange", alpha=0.8, zorder=5)
        self.trail, = self.ax.plot([], [], color="orange", linewidth=1.5, alpha=0.6, zorder=4)
        self.lidar_scatter = self.ax.scatter([], [], s=4, color="crimson", alpha=0.7, zorder=3)
        self.ax.add_patch(self.robot_patch)

        # HUD
        self.hud = self.ax.text(0.02, 0.98, "", transform=self.ax.transAxes, va="top", ha="left",
                                fontsize=9, color="black", bbox=dict(facecolor="white", alpha=0.6, edgecolor="none"))

        # Slider
        slider_ax = self.fig.add_axes([0.1, 0.05, 0.8, 0.03])
        self.slider = Slider(slider_ax, "frame", 0, len(self.timeline.frame_indices) - 1,
                             valinit=0, valstep=1, color="gray")
        self.slider.on_changed(self._on_slider)

        # Events
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.fig.canvas.mpl_connect("scroll_event", self._on_scroll)
        self.fig.canvas.mpl_connect("button_press_event", self._on_press)
        self.fig.canvas.mpl_connect("button_release_event", self._on_release)
        self.fig.canvas.mpl_connect("motion_notify_event", self._on_motion)
        self._drag_start: Optional[Tuple[float, float]] = None
        self._drag_center: Optional[Tuple[float, float]] = None

        self._update_view(0, force_camera=True)
        self._start_timer()

    # ---- Camera helpers ----
    def _initial_center(self) -> Vec2:
        x0, y0, x1, y1 = self.world_bounds
        return ((float(x0 + x1) * 0.5), (float(y0 + y1) * 0.5))

    def _initial_span(self) -> float:
        x0, y0, x1, y1 = self.world_bounds
        span = max(5.0, max(x1 - x0, y1 - y0))
        return span * 0.6

    def _traj_bounds(self) -> Tuple[float, float, float, float]:
        xs = list(map(float, self.run.traj.x)) if len(self.run.traj.x) else []
        ys = list(map(float, self.run.traj.y)) if len(self.run.traj.y) else []
        if not xs or not ys:
            return (0.0, 0.0, 10.0, 10.0)
        margin = 1.5
        return (min(xs) - margin, min(ys) - margin, max(xs) + margin, max(ys) + margin)

    def _apply_camera(self) -> None:
        span = self.camera_span
        cx, cy = self.camera_center
        self.ax.set_xlim(cx - span, cx + span)
        self.ax.set_ylim(cy - span, cy + span)
        self.ax.figure.canvas.draw_idle()

    def _robot_radius(self) -> float:
        meta = (self.run.frames.raw or {}).get("meta") if self.run.frames and self.run.frames.raw else None
        if isinstance(meta, dict):
            try:
                return float(meta.get("radius_m", 0.4))
            except Exception:
                pass
        return 0.4

    # ---- Input handling ----
    def _start_timer(self) -> None:
        interval_ms = 1000.0 / self.fps
        timer = self.fig.canvas.new_timer(interval=interval_ms)
        timer.add_callback(self._on_timer)
        timer.start()
        self._timer = timer

    def _on_timer(self) -> None:
        if not self.playing:
            return
        now = time.monotonic()
        elapsed = now - self.last_wall_time
        self.last_wall_time = now
        self.playback_clock += elapsed
        target_t = self.play_anchor_t + self.playback_clock
        idx = int(np.searchsorted(self.timeline.t, target_t, side="left"))
        idx = self.timeline.clamp_index(idx)
        self._set_frame(idx, reset_clock=False)

    def _on_slider(self, val: float) -> None:
        if self._suppress_slider:
            return
        self._set_frame(int(val), update_slider=False, reset_clock=True)

    def _on_key(self, event) -> None:
        if event.key == " ":
            self.playing = not self.playing
            if self.playing:
                self.last_wall_time = time.monotonic()
        elif event.key in {"right", "d"}:
            self.playing = False
            self._step(1)
        elif event.key in {"left", "a"}:
            self.playing = False
            self._step(-1)
        elif event.key in {"shift+right", "shift+up"}:
            self.playing = False
            self._step(10)
        elif event.key in {"shift+left", "shift+down"}:
            self.playing = False
            self._step(-10)
        elif event.key == "home":
            self.playing = False
            self._set_frame(0)
        elif event.key == "end":
            self.playing = False
            self._set_frame(len(self.timeline.frame_indices) - 1)
        elif event.key == "f":
            self.follow_robot = not self.follow_robot

    def _on_scroll(self, event) -> None:
        factor = 0.9 if event.button == "up" else 1.1
        self.camera_span *= factor
        self.camera_span = max(0.5, min(self.camera_span, 1e4))
        self._apply_camera()

    def _on_press(self, event) -> None:
        if event.inaxes != self.ax:
            return
        if event.button not in {1, 2, 3}:
            return
        self._drag_start = (event.xdata, event.ydata)
        self._drag_center = self.camera_center

    def _on_motion(self, event) -> None:
        if self._drag_start is None or self._drag_center is None:
            return
        if event.inaxes != self.ax or event.xdata is None or event.ydata is None:
            return
        dx = event.xdata - self._drag_start[0]
        dy = event.ydata - self._drag_start[1]
        self.camera_center = (self._drag_center[0] - dx, self._drag_center[1] - dy)
        self._apply_camera()

    def _on_release(self, event) -> None:
        self._drag_start = None
        self._drag_center = None

    def _step(self, delta: int) -> None:
        idx = self.timeline.clamp_index(self.frame_idx + delta)
        self._set_frame(idx)

    def _set_frame(self, idx: int, update_slider: bool = True, reset_clock: bool = True) -> None:
        idx = self.timeline.clamp_index(idx)
        if idx == self.frame_idx:
            return
        self.frame_idx = idx
        if update_slider:
            self._suppress_slider = True
            try:
                if int(self.slider.val) != idx:
                    self.slider.set_val(idx)
            finally:
                self._suppress_slider = False
        if reset_clock:
            self.play_anchor_t = float(self.timeline.t[idx])
            self.playback_clock = 0.0
            self.last_wall_time = time.monotonic()
        self._update_view(idx)

    # ---- Drawing ----
    def _update_view(self, idx: int, force_camera: bool = False) -> None:
        run_idx = self.timeline.frame_indices[idx]
        t = float(self.run.traj.t[run_idx])
        x = float(self.run.traj.x[run_idx])
        y = float(self.run.traj.y[run_idx])
        yaw = float(self.run.traj.yaw[run_idx])

        # Robot pose
        self.robot_patch.center = (x, y)

        # Breadcrumb trail (last 3 s)
        window_t = t - 3.0
        recent_mask = self.run.traj.t <= t
        if np.any(recent_mask):
            recent_idx = np.where(self.run.traj.t >= window_t)[0]
            if len(recent_idx):
                lo = int(recent_idx[0])
                hi = run_idx + 1
                self.trail.set_data(self.run.traj.x[lo:hi], self.run.traj.y[lo:hi])
        else:
            self.trail.set_data([], [])

        # Actors
        # Lidar
        if self.show_lidar and self.run.lidar is not None and self.timeline.lidar_map is not None:
            lidar_idx = int(self.timeline.lidar_map[idx])
            pts = self.run.lidar.points_for_index(lidar_idx, (x, y, yaw))
            if len(pts):
                color = "crimson"
                if self.color_mode == "wet":
                    wet = bool(self.run.traj.in_wet[run_idx]) if len(self.run.traj.in_wet) > run_idx else False
                    color = "dodgerblue" if wet else "crimson"
                pts_ds = pts[::max(1, int(len(pts) / 4000) or 1)]
                self.lidar_scatter.set_offsets(pts_ds[:, :2])
                self.lidar_scatter.set_color(color)
            else:
                self.lidar_scatter.set_offsets(np.empty((0, 2)))

        # HUD
        event_list = self.run.traj.text.get("event") or []
        detail_list = self.run.traj.text.get("event_detail") or []
        event = event_list[run_idx] if run_idx < len(event_list) else ""
        detail = detail_list[run_idx] if run_idx < len(detail_list) else ""
        wet = bool(self.run.traj.in_wet[run_idx]) if len(self.run.traj.in_wet) > run_idx else False
        v_cmd = self.run.traj.columns.get("v_cmd")
        speed = ""
        if v_cmd is not None and len(v_cmd) > run_idx:
            speed = f" v_cmd={v_cmd[run_idx]:.2f} m/s"
        self.hud.set_text(f"t={t:.2f}s  frame={run_idx}  yaw={yaw:.2f}{speed}\n"
                          f"event={event or '-'}  detail={detail or '-'}  in_wet={wet}")

        # Camera follow
        if self.follow_robot or force_camera:
            self.camera_center = (x, y)
        self._apply_camera()
        self.fig.canvas.draw_idle()

    def export_mp4(self, out_path: Path, fps: Optional[float] = None) -> None:
        """Render the current timeline to an MP4 (best-effort)."""
        try:
            import imageio.v2 as imageio
        except Exception:
            print("[edgesim replay] imageio not available; skipping export")
            return
        out_path = out_path.resolve()
        out_path.parent.mkdir(parents=True, exist_ok=True)
        playing_before = self.playing
        self.playing = False
        writer = imageio.get_writer(out_path, fps=fps or self.fps)
        try:
            for idx in range(len(self.timeline.frame_indices)):
                self.frame_idx = idx
                self._update_view(idx, force_camera=True)
                self.fig.canvas.draw()
                buf = np.frombuffer(self.fig.canvas.tostring_rgb(), dtype=np.uint8)
                w, h = self.fig.canvas.get_width_height()
                frame = buf.reshape((h, w, 3))
                writer.append_data(frame)
        finally:
            writer.close()
            self.playing = playing_before
        print(f"[edgesim replay] wrote {out_path}")


# ---------- CLI entry ----------

def resolve_run_dir(path: Path, run_index: Optional[int] = None) -> Path:
    path = path.resolve()
    if (path / "run_one.csv").exists():
        return path
    if path.name.startswith("run_") and (path / "run_one.csv").exists():
        return path
    per_run = path / "per_run"
    if per_run.exists():
        idx = 0 if run_index is None else max(0, int(run_index))
        candidate = per_run / f"run_{idx:04d}"
        if not candidate.exists():
            # pick first available run dir
            candidates = sorted([p for p in per_run.glob("run_*") if p.is_dir()])
            if not candidates:
                raise FileNotFoundError(f"No per-run folders found in {per_run}")
            candidate = candidates[0]
            print(f"[edgesim replay] run_one.csv not found at index {idx}; defaulting to {candidate.name}")
        if not (candidate / "run_one.csv").exists():
            raise FileNotFoundError(f"run_one.csv not found in {candidate}")
        return candidate
    raise FileNotFoundError(f"Could not find run_one.csv under {path}")


def run_viewer_cli(args: argparse.Namespace) -> int:
    export_path = getattr(args, "export_mp4", None)
    if not getattr(args, "viewer", False) and not export_path:
        raise SystemExit("Use --viewer to open the interactive replay (or --export-mp4 to save a video).")
    run_dir = resolve_run_dir(Path(args.run_dir), run_index=getattr(args, "run_index", None))
    loader = RunLoader(run_dir)
    run = loader.load(include_lidar=not getattr(args, "no_lidar", False))
    app = Viewer2DApp(
        run,
        follow_robot=bool(getattr(args, "follow_robot", False)),
        show_lidar=not getattr(args, "no_lidar", False),
        color_mode=str(getattr(args, "color_mode", "type")),
        fps=float(getattr(args, "fps", 30.0)),
        step=int(getattr(args, "step", 1)),
        start=int(getattr(args, "start", 0)),
        end=int(getattr(args, "end", -1)),
    )
    print(f"[edgesim replay] Loaded {run_dir}")
    if export_path:
        try:
            app.export_mp4(Path(export_path), fps=float(getattr(args, "fps", 30.0)))
        except Exception as e:
            print(f"[edgesim replay] Failed to export MP4: {e}")
    if getattr(args, "viewer", False):
        plt.show()
    return 0


def build_replay_parser(sub) -> None:
    sp = sub.add_parser("replay", help="Replay a finished run (opens viewer)")
    sp.add_argument("run_dir", type=str, help="Path to run folder or per_run/run_x")
    sp.add_argument("--run-index", type=int, default=None, help="When run_dir has per_run/, pick run_<idx>")
    sp.add_argument("--viewer", action="store_true", help="Open 2D viewer window (required for now)")
    sp.add_argument("--follow-robot", action="store_true", help="Keep camera centered on robot")
    sp.add_argument("--no-lidar", action="store_true", help="Disable LiDAR rendering even if lidar.npz exists")
    sp.add_argument("--color-mode", type=str, default="type", choices=["type", "wet"], help="Coloring for LiDAR points")
    sp.add_argument("--fps", type=float, default=30.0, help="Target render FPS for playback timer")
    sp.add_argument("--step", type=int, default=1, help="Subsample trajectory by this many steps")
    sp.add_argument("--start", type=int, default=0, help="Start frame index")
    sp.add_argument("--end", type=int, default=-1, help="End frame index (inclusive, -1 = last)")
    sp.add_argument("--export-mp4", type=str, default=None, help="Optional path to save an MP4 of the replay")
    sp.set_defaults(func=run_viewer_cli)

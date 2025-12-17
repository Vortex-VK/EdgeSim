from __future__ import annotations
from dataclasses import dataclass, field
from typing import Dict, Any, Tuple, List, Optional, Callable
import math, random, time

import numpy as np
import pybullet as p


# ------------------------------
# Common types / light utilities
# ------------------------------

Vec2 = Tuple[float, float]

def _clip(a: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, a))

def _wrap_angle(a: float) -> float:
    # Wrap to [-pi, pi)
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ------------------------------
# Base injector interface
# ------------------------------

@dataclass
class InjectorState:
    """A lightweight snapshot passed to injectors each step."""
    t: float                        # sim time [s]
    dt: float                       # step [s]
    robot_xy: Vec2                  # robot position (x, y)
    robot_yaw: float                # robot yaw [rad]
    humans: List[int]               # active human body ids
    fallen_human: Optional[int]     # fallen human body id (if any)
    bounds: Vec2                    # (Lx, Ly)
    border: float                   # safety border of map (for clamping)


class Injector:
    """Base class for event injectors."""
    name: str = "base"

    def on_step(self, s: InjectorState) -> None:
        """Advance internal state; may create bodies or modify dynamics."""
        return

    def apply_lidar(self, s: InjectorState, ranges: Optional[np.ndarray], max_range: float) -> Optional[np.ndarray]:
        """Optionally modify a LiDAR scan in-place; return modified array or None."""
        return None

    def pop_event(self) -> Optional[Tuple[str, str]]:
        """Return (event, detail) once when notable things happen."""
        return None


# ---------------------------------------------------------
# LiDAR sector blackout (e.g., glare/reflection/puddle fog)
# ---------------------------------------------------------

@dataclass
class LidarSectorBlackoutInjector(Injector):
    """
    When robot enters a trigger disc around (cx, cy), for `duration_s` seconds
    zero out a sector of LiDAR (set to max range).
    """
    name: str = "lidar_blackout"
    cx: float = 10.0
    cy: float = 10.0
    trigger_radius: float = 2.0
    duration_s: float = 4.0
    sector_deg: float = 60.0   # total angular width
    sector_offset_deg: float = 0.0  # center relative to robot forward (+x in robot frame)
    active_until: float = -1.0
    fired: bool = False

    def on_step(self, s: InjectorState) -> None:
        if self.active_until >= 0.0 and s.t <= self.active_until:
            return
        dx, dy = s.robot_xy[0] - self.cx, s.robot_xy[1] - self.cy
        if dx*dx + dy*dy <= self.trigger_radius * self.trigger_radius:
            self.active_until = s.t + self.duration_s
            self.fired = True

    def apply_lidar(self, s: InjectorState, ranges: Optional[np.ndarray], max_range: float) -> Optional[np.ndarray]:
        if ranges is None:
            return None
        if self.active_until < 0.0 or s.t > self.active_until:
            return None
        n = len(ranges)
        # compute sector index window
        half = math.radians(self.sector_deg) / 2.0
        center = _wrap_angle(math.radians(self.sector_offset_deg))
        angles = np.linspace(-math.pi, math.pi, n, endpoint=False)
        mask = np.abs((_wrap_angle(angles - center))) <= half
        # set sector to max range (blackout)
        ranges = ranges.copy()
        ranges[mask] = max_range
        return ranges

    def pop_event(self) -> Optional[Tuple[str, str]]:
        if self.fired:
            self.fired = False
            return ("sensor_glare_blackout", f"{self.cx:.2f},{self.cy:.2f},{self.sector_deg:.0f}deg,{self.duration_s:.1f}s")
        return None


# ---------------------------------------------------------
# Ghost / phantom obstacles in LiDAR
# ---------------------------------------------------------

@dataclass
class GhostObstacleInjector(Injector):
    """
    Inserts short-lived phantom obstacle clusters into LiDAR scans to emulate
    sensor ghosts / false positives. Clusters are defined in world-frame angle
    and distance and decay over time.
    """
    name: str = "ghost_obstacle"
    spawn_rate_hz: float = 0.25           # expected spawns per second
    dist_range: Tuple[float, float] = (1.5, 6.0)
    ttl_range: Tuple[float, float] = (0.6, 2.0)
    width_deg: Tuple[float, float] = (8.0, 18.0)
    range_jitter: float = 0.4
    max_clusters: int = 4
    rng_seed: Optional[int] = None
    _rng: np.random.Generator = field(default_factory=np.random.default_rng, repr=False)
    _clusters: List[Dict[str, float]] = field(default_factory=list)
    _event_queue: List[Tuple[str, str]] = field(default_factory=list)

    def __post_init__(self) -> None:
        if self.rng_seed is not None:
            try:
                self._rng = np.random.default_rng(int(self.rng_seed))
            except Exception:
                self._rng = np.random.default_rng()

    def on_step(self, s: InjectorState) -> None:
        # Drop expired clusters
        if self._clusters:
            for c in self._clusters:
                c["ttl"] -= s.dt
            self._clusters = [c for c in self._clusters if c["ttl"] > 0.0]
        # Probabilistic spawn
        if len(self._clusters) >= self.max_clusters:
            return
        if self._rng.random() < max(0.0, self.spawn_rate_hz) * s.dt:
            ang = float(self._rng.uniform(-math.pi, math.pi))
            dist = float(self._rng.uniform(*self.dist_range))
            ttl = float(self._rng.uniform(*self.ttl_range))
            width = float(self._rng.uniform(*self.width_deg))
            self._clusters.append({"ang": ang, "dist": dist, "ttl": ttl, "width_deg": width})
            self._event_queue.append(("sensor_fault_ghost", f"{dist:.1f}m@{math.degrees(ang):.0f}deg"))

    def apply_lidar(self, s: InjectorState, ranges: Optional[np.ndarray], max_range: float) -> Optional[np.ndarray]:
        if ranges is None or not self._clusters:
            return None
        n = len(ranges)
        angles = np.linspace(-math.pi, math.pi, n, endpoint=False)
        mod = ranges.copy()
        changed = False
        for c in self._clusters:
            rel = _wrap_angle(c["ang"] - s.robot_yaw)
            half = math.radians(c.get("width_deg", 10.0)) * 0.5
            mask = np.abs(_wrap_angle(angles - rel)) <= half
            if not np.any(mask):
                continue
            noise = self._rng.normal(0.0, self.range_jitter, size=np.count_nonzero(mask))
            phantom = np.clip(c["dist"] + noise, 0.3, max_range * 0.9)
            mod[mask] = phantom
            changed = True
        if changed:
            return mod
        return None

    def pop_event(self) -> Optional[Tuple[str, str]]:
        if self._event_queue:
            return self._event_queue.pop(0)
        return None


# ---------------------------------------
# Falling object (box) / shatter to puddle
# ---------------------------------------

@dataclass
class FallingObjectInjector(Injector):
    """
    Spawns a small box above an aisle which falls under gravity. On first impact,
    optionally 'shatters' into a wet patch (low μ).
    """
    name: str = "falling_object"
    drop_x: float = 8.0
    drop_y: float = 10.0
    drop_z: float = 3.5
    half_extents: Tuple[float, float, float] = (0.25, 0.25, 0.25)
    mass: float = 2.0
    restitution: float = 0.1
    lateral_mu: float = 0.6
    shatter_on_impact: bool = True
    puddle_mu: float = 0.32
    puddle_half: Tuple[float, float] = (1.2, 0.6)  # x,y half-extends
    body_id: Optional[int] = None
    made_puddle: bool = False
    fired_spawn: bool = False
    _will_shatter: Optional[bool] = None
    _event_queue: List[Tuple[str, str]] = field(default_factory=list)

    # callback to create wet patch in sim: (x0,y0,x1,y1,mu)-> int body id
    spawn_wet_patch_cb: Optional[Callable[[float, float, float, float, float], int]] = None

    def on_step(self, s: InjectorState) -> None:
        # Spawn when robot is near but not underneath
        if self.body_id is None:
            dx, dy = s.robot_xy[0] - self.drop_x, s.robot_xy[1] - self.drop_y
            if dx*dx + dy*dy < 9.0:  # within 3m
                vis = p.createVisualShape(p.GEOM_BOX, halfExtents=list(self.half_extents), rgbaColor=[0.8, 0.6, 0.2, 1.0])
                col = p.createCollisionShape(p.GEOM_BOX, halfExtents=list(self.half_extents))
                self.body_id = p.createMultiBody(baseMass=self.mass, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                                                 basePosition=[self.drop_x, self.drop_y, self.drop_z])
                p.changeDynamics(self.body_id, -1, restitution=self.restitution, lateralFriction=self.lateral_mu)
                self.fired_spawn = True
                self._event_queue.append(("falling_object_spawn", f"{self.drop_x:.2f},{self.drop_y:.2f},{self.drop_z:.2f}"))
                return

        # If spawned, check for ground contact
        if self.body_id is not None and not self.made_puddle:
            cps = p.getContactPoints(bodyA=self.body_id)
            for cp in cps:
                contact_dist = float(cp[8]) if len(cp) > 8 else 1.0
                normal_force = float(cp[9]) if len(cp) > 9 else 0.0
                # Require an actual contact/overlap or non-zero force
                if contact_dist > 0.02 and normal_force <= 0.0:
                    continue
                x, y, z = p.getBasePositionAndOrientation(self.body_id)[0]
                # Ignore spurious early contacts while object is still high
                if z > max(0.6, 0.15 * self.drop_z):
                    continue
                if self._will_shatter is None:
                    self._will_shatter = bool(self.shatter_on_impact and random.random() < 0.5)
                if self._will_shatter and self.spawn_wet_patch_cb is not None:
                    hx, hy = self.puddle_half
                    x0, y0 = _clip(x - hx, 0.0, s.bounds[0]), _clip(y - hy, 0.0, s.bounds[1])
                    x1, y1 = _clip(x + hx, 0.0, s.bounds[0]), _clip(y + hy, 0.0, s.bounds[1])
                    pid = self.spawn_wet_patch_cb(x0, y0, x1, y1, self.puddle_mu)
                    self._event_queue.append(("shatter_to_puddle", f"{x0:.2f},{y0:.2f},{x1:.2f},{y1:.2f},mu={self.puddle_mu:.2f}"))
                    self.made_puddle = True
                    return
                # Non-shatter: settle on ground at box height and zero velocities
                h = float(self.half_extents[2]) if len(self.half_extents) >= 3 else 0.25
                _, orn = p.getBasePositionAndOrientation(self.body_id)
                z_set = max(h, 0.02)
                p.resetBasePositionAndOrientation(self.body_id, [x, y, z_set], orn)
                p.resetBaseVelocity(self.body_id, linearVelocity=[0.0, 0.0, 0.0], angularVelocity=[0.0, 0.0, 0.0])
                p.changeDynamics(self.body_id, -1, restitution=0.0, linearDamping=0.8, angularDamping=0.8)
                self.made_puddle = True
                return

    def pop_event(self) -> Optional[Tuple[str, str]]:
        if self._event_queue:
            return self._event_queue.pop(0)
        return None

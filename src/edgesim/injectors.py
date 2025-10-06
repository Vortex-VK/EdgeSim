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
                # Any contact with plane or floor is enough
                if cp[3] < 0.05:  # contact distance
                    if self.shatter_on_impact and self.spawn_wet_patch_cb is not None:
                        x, y, z = p.getBasePositionAndOrientation(self.body_id)[0]
                        hx, hy = self.puddle_half
                        x0, y0 = _clip(x - hx, 0.0, s.bounds[0]), _clip(y - hy, 0.0, s.bounds[1])
                        x1, y1 = _clip(x + hx, 0.0, s.bounds[0]), _clip(y + hy, 0.0, s.bounds[1])
                        pid = self.spawn_wet_patch_cb(x0, y0, x1, y1, self.puddle_mu)
                        self._event_queue.append(("shatter_to_puddle", f"{x0:.2f},{y0:.2f},{x1:.2f},{y1:.2f},mu={self.puddle_mu:.2f}"))
                    self.made_puddle = True
                    return

    def pop_event(self) -> Optional[Tuple[str, str]]:
        if self._event_queue:
            return self._event_queue.pop(0)
        return None


# ------------------------------------------
# Thrown/flying object across robot trajectory
# ------------------------------------------

@dataclass
class ThrownObjectInjector(Injector):
    """
    Launches a light object across the aisle, creating a fast cross-traffic hazard.
    """
    name: str = "thrown_object"
    origin: Vec2 = (6.0, 3.0)
    target: Vec2 = (6.0, 17.0)
    z0: float = 1.2
    speed_mps: float = 6.0
    mass: float = 0.5
    radius: float = 0.12
    restitution: float = 0.3
    launch_within_r: float = 4.0
    _body: Optional[int] = None
    _launched: bool = False
    _event_done: bool = False

    def on_step(self, s: InjectorState) -> None:
        if self._launched:
            return
        dx = s.robot_xy[0] - self.origin[0]
        dy = s.robot_xy[1] - self.origin[1]
        if dx*dx + dy*dy <= self.launch_within_r * self.launch_within_r:
            # Build small sphere and give it initial velocity toward target
            vis = p.createVisualShape(p.GEOM_SPHERE, radius=self.radius, rgbaColor=[0.3, 0.3, 0.9, 1.0])
            col = p.createCollisionShape(p.GEOM_SPHERE, radius=self.radius)
            self._body = p.createMultiBody(baseMass=self.mass, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                                           basePosition=[self.origin[0], self.origin[1], self.z0])
            p.changeDynamics(self._body, -1, restitution=self.restitution, lateralFriction=0.7)
            # Compute velocity
            vx = self.target[0] - self.origin[0]
            vy = self.target[1] - self.origin[1]
            L = math.hypot(vx, vy) + 1e-9
            vx, vy = (vx / L) * self.speed_mps, (vy / L) * self.speed_mps
            p.resetBaseVelocity(self._body, linearVelocity=[vx, vy, 0.0])
            self._launched = True

    def pop_event(self) -> Optional[Tuple[str, str]]:
        if self._launched and not self._event_done:
            self._event_done = True
            return ("thrown_object", f"{self.origin[0]:.2f},{self.origin[1]:.2f}->{self.target[0]:.2f},{self.target[1]:.2f}")
        return None

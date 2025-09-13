from __future__ import annotations
from typing import Dict, Any, List, Tuple
from pathlib import Path
import csv
import math
import time

import numpy as np
import pybullet as p

from .world import build_world, spawn_human

# ---------- utils ----------

def _write_csv(path: Path, header: List[str], rows: List[List[float | str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)

def _spawn_disc(radius: float = 0.4, height: float = 0.2, mass: float = 20.0, color=(0.9, 0.3, 0.3, 1.0)) -> int:
    """Simple cylindrical AMR body."""
    vis = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height, rgbaColor=color)
    col = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)
    body = p.createMultiBody(
        baseMass=mass,
        baseCollisionShapeIndex=col,
        baseVisualShapeIndex=vis,
        basePosition=[1.0, 1.0, height / 2.0],
    )
    # Wheel-ish friction; sliding still possible on wet patches
    p.changeDynamics(body, -1, lateralFriction=0.8)
    return body

def _spawn_wet_patch(x0: float, y0: float, x1: float, y1: float, mu: float = 0.35, rgba=(0.2, 0.6, 1.0, 0.35)) -> int:
    """Create a thin low-friction slab over the plane."""
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    hx, hy = max(0.01, (x1 - x0) / 2.0), max(0.01, (y1 - y0) / 2.0)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[hx, hy, 0.001], rgbaColor=rgba)
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[hx, hy, 0.001])
    bid = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
                            basePosition=[cx, cy, 0.001])
    p.changeDynamics(bid, -1, lateralFriction=float(mu))
    return bid

def _spawn_fallen_human(x: float, y: float, length: float = 1.6, radius: float = 0.25,
                        mass: float = 70.0, rgba=(0.2, 0.8, 0.2, 1.0)) -> int:
    """
    Spawn a dynamic, horizontal capsule to represent a fallen person.
    Oriented so its long axis lies along the X-axis (pitch = 90°).
    """
    col = p.createCollisionShape(p.GEOM_CAPSULE, radius=radius, height=length)
    vis = p.createVisualShape(p.GEOM_CAPSULE, radius=radius, length=length, rgbaColor=rgba)
    quat = p.getQuaternionFromEuler([0.0, math.pi / 2.0, 0.0])  # lay on side
    z = radius  # resting on the ground
    bid = p.createMultiBody(
        baseMass=mass,
        baseCollisionShapeIndex=col,
        baseVisualShapeIndex=vis,
        basePosition=[x, y, z],
        baseOrientation=quat,
    )
    # Slightly lower friction so it can slide a bit on the wet area
    p.changeDynamics(bid, -1, lateralFriction=0.5)
    # Small nudge
    p.resetBaseVelocity(bid, linearVelocity=[0.2, 0.0, 0.0], angularVelocity=[0.0, 0.0, 1.0])
    return bid

def _rasterized_perimeter_aabbs(map_w: float, map_h: float, band: float = 0.05) -> List[Tuple[float, float, float, float]]:
    """Four thin perimeter rectangles to keep the robot in-bounds; used for clearance."""
    return [
        (0.0, 0.0, band, map_h),                 # left
        (map_w - band, 0.0, map_w, map_h),       # right
        (0.0, 0.0, map_w, band),                 # bottom
        (0.0, map_h - band, map_w, map_h),       # top
    ]

def _clearance_to_aabbs(x: float, y: float, aabbs: List[Tuple[float, float, float, float]]) -> float:
    """Point-to-rectangle Euclidean distance; aabbs: list of (x0,y0,x1,y1)."""
    d = 1e9
    for x0, y0, x1, y1 in aabbs:
        dx = max(x0 - x, 0.0, x - x1)
        dy = max(y0 - y, 0.0, y - y1)
        dd = math.hypot(dx, dy)
        if dd < d:
            d = dd
    return float(d)

def _pid_follow(current_xy: Tuple[float, float], target_xy: Tuple[float, float], prev_err: np.ndarray,
                Kp: float = 1.2, Ki: float = 0.0, Kd: float = 0.2, dt: float = 0.1) -> Tuple[np.ndarray, np.ndarray]:
    """Very small PID on XY error; returns control vector and updated integral/prev terms (here we just track prev)."""
    err = np.array(target_xy) - np.array(current_xy)
    der = (err - prev_err) / max(dt, 1e-6)
    ctrl = Kp * err + Kd * der  # Ki omitted for V0
    return ctrl, err

def _pt_in_aabb(x: float, y: float, aabb: Tuple[float, float, float, float] | None) -> bool:
    if aabb is None:
        return False
    x0, y0, x1, y1 = aabb
    return (x0 <= x <= x1) and (y0 <= y <= y1)

# ---------- main single-run ----------

def run_one(
    prompt: str,
    scn: Dict[str, Any],
    out_dir: Path,
    dt: float = 0.05,
    realtime: bool = False,
    gui: bool = False,
) -> Dict[str, Any]:
    """
    Single rollout:
      - world (plane + perimeter, wet patches on plane)
      - AMR (disc), kinematic + PID follower
      - human starts late, runs; may slip on wet band (probabilistic); may finish crossing
      - proximity brake for AMR to avoid some collisions
      - coverage columns: in_wet, human_phase, near_stop
      - stop condition: robot–human collision; otherwise run until success/timeout
    """
    # allow scenario dt override
    if "runtime" in scn and "dt" in scn["runtime"]:
        try:
            dt = float(scn["runtime"]["dt"])
        except Exception:
            pass

    # world/scene
    env = build_world(scn, use_gui=gui)
    Lx, Ly = env["bounds"]
    plane_id = env["plane_id"]

    ignored_for_collision = {plane_id, *env["patches"]}  # ignore base plane + configured wet patches

    if gui:
        p.resetDebugVisualizerCamera(
            cameraDistance=10.0,
            cameraYaw=45.0,
            cameraPitch=-35.0,
            cameraTargetPosition=[0.5 * Lx, 0.5 * Ly, 0.0],
        )

    # domain randomization: per-run wet μ (seeded)
    _seed = int(scn.get("seed", 0))
    rng = np.random.default_rng(_seed)
    wet_mu = float(rng.uniform(0.35, 0.60))
    for bid in env["patches"]:
        p.changeDynamics(bid, -1, lateralFriction=wet_mu)

    # robot
    radius = float(scn["agents"][0].get("radius_m", 0.4))
    amr = _spawn_disc(radius=radius)

    # start/goal
    start = list(map(float, scn["layout"]["start"]))
    goal = list(map(float, scn["layout"]["goal"]))

    # initialize pose at start
    p.resetBasePositionAndOrientation(amr, [start[0], start[1], 0.1], [0, 0, 0, 1])

    # straight-line waypoints toward goal
    waypoints: List[Tuple[float, float]] = []
    steps = max(2, int(max(abs(goal[0] - start[0]), abs(goal[1] - start[1])) / 0.5))
    for k in range(1, steps + 1):
        a = k / steps
        waypoints.append((start[0] + a * (goal[0] - start[0]), start[1] + a * (goal[1] - start[1])))
    if tuple(waypoints[-1]) != (goal[0], goal[1]):
        waypoints.append((goal[0], goal[1]))

    # clearance: ONLY hard geometry (perimeter walls). Do NOT include wet patches.
    static_aabbs = _rasterized_perimeter_aabbs(Lx, Ly, band=0.05)

    # ---------- EDGE CASE: late runner over wet band, may slip ----------
    use_human = bool(scn["hazards"].get("human"))
    human_id = None
    fallen_human_id = None
    wet_band_aabb: Tuple[float, float, float, float] | None = None

    # For visual debug & braking
    xh, yh = None, None

    if use_human:
        human_id = spawn_human()
        _human_z = p.getBasePositionAndOrientation(human_id)[0][2]

        border = max(0.6, radius + 0.2)
        cross_x = float(np.clip(0.5 * Lx, border, Lx - border))

        # Crossing (bottom -> top) fully in-bounds
        start_xy = (cross_x, border)
        end_xy   = (cross_x, Ly - border)

        cfg = scn["hazards"]["human"][0]
        crossing_duration = float(cfg.get("duration_s_running", 2.5))  # fast (runner)
        p_slip = float(cfg.get("p_slip", 0.7))                        # 70% slip by default
        trigger_mu = float(cfg.get("trigger_mu", 1.2))                # mean late trigger
        trigger_sigma = float(cfg.get("trigger_sigma", 0.4))          # spread of lateness
        trigger_distance = float(max(0.3, rng.normal(trigger_mu, trigger_sigma)))

        # Wet band centered at crossing line (separate from scene patches)
        wet_band_h = 1.2
        wet_y0 = max(border, 0.5 * Ly - 0.5 * wet_band_h)
        wet_y1 = min(Ly - border, 0.5 * wet_band_h + 0.5 * Ly)
        wet_id = _spawn_wet_patch(cross_x - 1.5, wet_y0, cross_x + 1.5, wet_y1, mu=0.32)
        wet_band_aabb = (cross_x - 1.5, wet_y0, cross_x + 1.5, wet_y1)
        ignored_for_collision.add(wet_id)  # never collide with wet band

        crossing_active = False
        crossing_started_at = 0.0
        slipped = False

    # logging
    out_dir.mkdir(parents=True, exist_ok=True)
    rows: List[List[float | str]] = []
    # coverage columns added at the end:
    header = ["t", "x", "y", "yaw", "v_cmd", "w_cmd", "min_clearance", "event", "event_detail",
              "in_wet", "human_phase", "near_stop"]

    # loop state
    t = 0.0
    timeout = float(scn["runtime"]["duration_s"])
    min_clear = 10.0
    wp_idx = 0
    success = False
    prev_err = np.array([0.0, 0.0])

    def _compute_in_wet(xr: float, yr: float) -> bool:
        # scene-configured traction patches (visual slabs on plane)
        for patch in scn["hazards"].get("traction", []):
            x0, y0, x1, y1 = patch["zone"]
            if _pt_in_aabb(xr, yr, (float(x0), float(y0), float(x1), float(y1))):
                return True
        # ad-hoc crossing band
        if _pt_in_aabb(xr, yr, wet_band_aabb):
            return True
        return False

    def _human_phase_str() -> str:
        if not use_human:
            return "none"
        if fallen_human_id is not None:
            return "fallen"
        if 'crossing_active' in locals() and crossing_active:
            return "running"
        return "none"

    # main loop
    while t < timeout:
        # read pose
        pos, orn = p.getBasePositionAndOrientation(amr)
        x, y, _ = pos
        yaw = p.getEulerFromQuaternion(orn)[2]

        # target waypoint
        target = waypoints[min(wp_idx, len(waypoints) - 1)]

        # PID (XY) -> control vector
        ctrl, prev_err = _pid_follow((x, y), target, prev_err, dt=dt)
        desired_yaw = math.atan2(target[1] - y, target[0] - x)
        yaw_err = (desired_yaw - yaw + math.pi) % (2 * math.pi) - math.pi
        w_cmd = max(-1.5, min(1.5, 1.2 * yaw_err))
        v_cmd = max(0.0, min(1.8, float(np.linalg.norm(ctrl))))

        # --- proximity brake vs. human (front-arc) ---
        STOP_DIST = 1.2     # full stop if a human is closer than this
        SLOW_DIST = 3.0     # slow down if within this range
        STOP_HOLD = 0.8     # keep stopped this many seconds after last "too close"
        if 'stop_until' not in locals():
            stop_until = 0.0

        h_pos = None
        if use_human:
            if fallen_human_id is not None:
                h_pos = p.getBasePositionAndOrientation(fallen_human_id)[0]
            elif 'crossing_active' in locals() and crossing_active and xh is not None and yh is not None:
                h_pos = (xh, yh, _human_z)

        if h_pos is not None:
            dx, dy = h_pos[0] - x, h_pos[1] - y
            ahead = (math.cos(yaw) * dx + math.sin(yaw) * dy) > 0.0
            dist = math.hypot(dx, dy)

            if ahead and dist < STOP_DIST:
                stop_until = max(stop_until, t + STOP_HOLD)
            elif ahead and dist < SLOW_DIST:
                v_cmd = min(v_cmd, 0.2)

        # apply “hard stop” if we’re in hold window
        if t < stop_until:
            v_cmd = 0.0


        # kinematic integrator
        yaw += w_cmd * dt
        x += v_cmd * math.cos(yaw) * dt
        y += v_cmd * math.sin(yaw) * dt
        quat = p.getQuaternionFromEuler([0.0, 0.0, yaw])
        p.resetBasePositionAndOrientation(amr, [x, y, 0.1], quat)

        # waypoint advance
        if math.hypot(target[0] - x, target[1] - y) < max(0.35, radius):
            wp_idx += 1
            if wp_idx >= len(waypoints):
                success = True
                in_wet = int(_compute_in_wet(x, y))
                rows.append([t, x, y, yaw, v_cmd, w_cmd, min_clear, "success", "",
                             in_wet, _human_phase_str(), int(v_cmd < 0.2)])
                break

        # clearance to hard geometry only
        clearance = _clearance_to_aabbs(x, y, static_aabbs)
        min_clear = min(min_clear, clearance)

        # collision filtering: stop only on human/fallen human
        raw_contacts = p.getContactPoints(bodyA=amr)
        human_contact = False
        nonhuman_contact_count = 0
        for cp in raw_contacts:
            bodyA = cp[1]; bodyB = cp[2]
            if bodyA in ignored_for_collision or bodyB in ignored_for_collision:
                continue
            if amr not in (bodyA, bodyB):
                continue
            other = bodyB if bodyA == amr else bodyA
            if other == human_id or other == fallen_human_id:
                human_contact = True
                break
            else:
                nonhuman_contact_count += 1

        if human_contact:
            in_wet = int(_compute_in_wet(x, y))
            rows.append([t, x, y, yaw, v_cmd, w_cmd, 0.0, "collision_human", "1",
                         in_wet, _human_phase_str(), int(v_cmd < 0.2)])
            success = False
            break
        else:
            if nonhuman_contact_count > 0:
                in_wet = int(_compute_in_wet(x, y))
                rows.append([t, x, y, yaw, v_cmd, w_cmd, min_clear, "contact_nonhuman", str(nonhuman_contact_count),
                             in_wet, _human_phase_str(), int(v_cmd < 0.2)])

        # human crossing state machine (late start → run → maybe slip → fall and stay)
        if use_human and human_id is not None:
            if fallen_human_id is None:
                if not crossing_active:
                    # Trigger when near the crossing line, with randomized lateness
                    if abs(x - start_xy[0]) <= trigger_distance:
                        crossing_active = True
                        crossing_started_at = t
                        p.resetBasePositionAndOrientation(human_id, [start_xy[0], start_xy[1], _human_z], [0, 0, 0, 1])
                        xh, yh = start_xy
                else:
                    # Running human position
                    alpha = min(1.0, (t - crossing_started_at) / crossing_duration)
                    xh = start_xy[0] + alpha * (end_xy[0] - start_xy[0])
                    yh = start_xy[1] + alpha * (end_xy[1] - start_xy[1])
                    p.resetBasePositionAndOrientation(human_id, [xh, yh, _human_z], [0, 0, 0, 1])

                    # Slip when entering wet band (probabilistic)
                    if (wet_band_aabb is not None) and (not slipped):
                        x0, y0, x1, y1 = wet_band_aabb
                        if (x0 <= xh <= x1) and (y0 <= yh <= y1):
                            if rng.random() < p_slip:
                                slipped = True
                                in_wet = int(_compute_in_wet(x, y))
                                rows.append([t, x, y, yaw, v_cmd, w_cmd, min_clear, "slip", f"{xh:.2f},{yh:.2f}",
                                             in_wet, _human_phase_str(), int(v_cmd < 0.2)])
                                # Hide kinematic runner offstage
                                p.resetBasePositionAndOrientation(human_id, [-10.0, -10.0, _human_z], [0,0,0,1])
                                # Spawn fallen dynamic body at slip point
                                fallen_human_id = _spawn_fallen_human(xh, yh)
                                rows.append([t, x, y, yaw, v_cmd, w_cmd, min_clear, "fall_spawned", str(fallen_human_id),
                                             in_wet, _human_phase_str(), int(v_cmd < 0.2)])
                                crossing_active = False
                    # If they finish crossing without slipping, remove them
                    if slipped is False and alpha >= 1.0:
                        crossing_active = False
                        p.resetBasePositionAndOrientation(human_id, [-10.0, -10.0, _human_z], [0, 0, 0, 1])
                        xh, yh = None, None

        # regular log row
        in_wet = int(_compute_in_wet(x, y))
        rows.append([t, x, y, yaw, v_cmd, w_cmd, min_clear, "", "", in_wet, _human_phase_str(), int(v_cmd < 0.2)])

        # step
        p.stepSimulation()
        if realtime:
            time.sleep(dt)
        t += dt

    # write CSV
    _write_csv(out_dir / "run_one.csv", header, rows)

    return {"success": success, "time": t, "steps": len(rows)}

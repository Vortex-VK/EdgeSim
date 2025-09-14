from __future__ import annotations
from typing import Dict, Any, List, Tuple
from pathlib import Path
import csv
import math
import time

import numpy as np
import pybullet as p

from .world import build_world, spawn_human
from .sampling import should_save_frames  # outcome-aware shared budget


# ---------- small I/O helpers ----------

def _write_csv(path: Path, header: List[str], rows: List[List[float | str]]) -> None:
	path.parent.mkdir(parents=True, exist_ok=True)
	with path.open("w", newline="", encoding="utf-8") as f:
		w = csv.writer(f)
		w.writerow(header)
		w.writerows(rows)


# ---------- geometry & controls ----------

def _spawn_disc(radius: float = 0.4, height: float = 0.2, mass: float = 20.0, color=(0.9, 0.3, 0.3, 1.0)) -> int:
	"""Simple cylindrical AMR body (disc)."""
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
	bid = p.createMultiBody(
		baseMass=0,
		baseCollisionShapeIndex=col,
		baseVisualShapeIndex=vis,
		basePosition=[cx, cy, 0.001],
	)
	p.changeDynamics(bid, -1, lateralFriction=float(mu))
	return bid


def _aabb_distance_point(aabb: Tuple[float, float, float, float], x: float, y: float) -> float:
	"""Distance from 2D point to AABB (0 if inside)."""
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
	ctrl = kp * err + kd * derr  # velocity-like vector in XY
	return ctrl, err


def _pt_in_aabb(x: float, y: float, aabb: Tuple[float, float, float, float] | None) -> bool:
	if aabb is None:
		return False
	x0, y0, x1, y1 = aabb
	return (x0 <= x <= x1) and (y0 <= y <= y1)


# ---------- simple frame capture (TinyRenderer works headless) ----------

def _capture_frame(batch_root: Path, run_dir_name: str, cam_conf: Dict[str, Any], width: int = 640, height: int = 480, idx: int = 0) -> None:
	view_m = p.computeViewMatrix(
		cameraEyePosition=[cam_conf["cx"], cam_conf["cy"], cam_conf["cz"]],
		cameraTargetPosition=[cam_conf["tx"], cam_conf["ty"], 0.0],
		cameraUpVector=[0.0, 1.0, 0.0],
	)
	proj_m = p.computeProjectionMatrixFOV(fov=cam_conf["fov_deg"], aspect=width/height, nearVal=0.01, farVal=100.0)
	_, _, px, _, _ = p.getCameraImage(width, height, view_m, proj_m, renderer=p.ER_TINY_RENDERER)

	img = np.reshape(np.array(px, dtype=np.uint8), (height, width, 4))[:, :, :3]  # drop alpha
	out_dir = batch_root / "frames_sample" / run_dir_name
	out_dir.mkdir(parents=True, exist_ok=True)
	out_path = out_dir / f"{idx:03d}.png"

	# Try pybullet writer; fallback to imageio if available
	try:
		p.writeImageFile(str(out_path), img)  # type: ignore[attr-defined]
	except Exception:
		try:
			import imageio.v2 as imageio
			imageio.imwrite(out_path, img)
		except Exception:
			# Last resort: silently skip
			return


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
	Single rollout (V0) with outcome-aware frame sampling:
	- Build world (plane, walls, traction/wet patches)
	- AMR: kinematic disc with PID XY follower + yaw PD
	- Human: late-start crosser; may slip on wet band and become a fallen obstacle
	- Proximity brake attempts to avoid human (slow/stop)
	- CSV: min_clearance, in_wet, human_phase, events (success/collision/contacts)
	- Frames: capture ONE terminal frame iff shared batch budget allows (sampling.py)
	"""
	# Allow scenario dt override
	if "runtime" in scn and "dt" in scn["runtime"]:
		try:
			dt = float(scn["runtime"]["dt"])
		except Exception:
			pass

	# Build world
	env = build_world(scn, use_gui=gui)
	client_id = env["client"]  # for safe teardown later
	try:
		Lx, Ly = env["bounds"]
		plane_id = env["plane_id"]
		wall_ids = list(env.get("walls", []))
		patch_ids = list(env.get("patches", []))
		ignored_for_collision = {plane_id, *patch_ids}

		if gui:
			p.resetDebugVisualizerCamera(
				cameraDistance=10.0,
				cameraYaw=45.0,
				cameraPitch=-35.0,
				cameraTargetPosition=[0.5 * Lx, 0.5 * Ly, 0.0],
			)

		# Randomize traction μ for configured patches (seeded)
		seed_val = int(scn.get("seed", 0))
		rng = np.random.default_rng(seed_val)
		wet_mu = float(rng.uniform(0.35, 0.60))
		for bid in patch_ids:
			p.changeDynamics(bid, -1, lateralFriction=wet_mu)

		# Robot (disc)
		radius = float(scn.get("agents", [{}])[0].get("radius_m", 0.4))
		amr = _spawn_disc(radius=radius)

		# Start & goal
		border = 0.6
		start = list(map(float, scn.get("layout", {}).get("start", [border, border])))
		goal = list(map(float, scn.get("layout", {}).get("goal", [Lx - border, Ly - border])))
		start[0] = max(border, min(Lx - border, start[0]))
		start[1] = max(border, min(Ly - border, start[1]))
		goal[0] = max(border, min(Lx - border, goal[0]))
		goal[1] = max(border, min(Ly - border, goal[1]))
		p.resetBasePositionAndOrientation(amr, [start[0], start[1], radius * 0.5], p.getQuaternionFromEuler([0, 0, 0]))

		# Simple two-waypoint path (start -> goal)
		waypoints: List[Tuple[float, float]] = [(goal[0], goal[1])]

		# Human hazard
		use_human = bool(scn.get("hazards", {}).get("human"))
		human_id: int | None = spawn_human() if use_human else None
		fallen_human_id: int | None = None

		if human_id is not None:
			# Defaults, overridable by scenario.hazards.human[0]
			cfg = (scn.get("hazards", {}).get("human") or [{}])[0]
			cross_x = float(cfg.get("cross_x", 0.5 * Lx))
			crossing_duration = float(cfg.get("duration_s_running", 2.5))  # fast runner
			p_slip = float(cfg.get("p_slip", 0.7))
			trigger_mu = float(cfg.get("trigger_mu", 1.2))
			trigger_sigma = float(cfg.get("trigger_sigma", 0.4))
			trigger_distance = float(max(0.3, rng.normal(trigger_mu, trigger_sigma)))

			# Wet band for crossing area
			wet_band_h = 1.2
			wet_y0 = max(border, 0.5 * Ly - 0.5 * wet_band_h)
			wet_y1 = min(Ly - border, 0.5 * wet_band_h + 0.5 * Ly)
			wet_id = _spawn_wet_patch(cross_x - 1.5, wet_y0, cross_x + 1.5, wet_y1, mu=0.32)
			patch_ids.append(wet_id)
			ignored_for_collision.add(wet_id)
			wet_band_aabb: Tuple[float, float, float, float] = (cross_x - 1.5, wet_y0, cross_x + 1.5, wet_y1)
		else:
			cross_x = 0.5 * Lx
			crossing_duration = 0.0
			p_slip = 0.0
			trigger_distance = 9999.0
			wet_band_aabb = None  # type: ignore

		# Human state
		crossing_active = False
		crossing_started_at = 0.0
		slipped = False

		# Logging setup
		out_dir.mkdir(parents=True, exist_ok=True)
		rows: List[List[float | str]] = []
		header = [
			"t", "x", "y", "yaw", "v_cmd", "w_cmd",
			"min_clearance", "event", "event_detail",
			"in_wet", "human_phase", "near_stop"
		]

		# Loop state
		t = 0.0
		timeout = float(scn.get("runtime", {}).get("duration_s", 90.0))
		min_clear = 10.0
		prev_err = np.array([0.0, 0.0])
		wp_idx = 0
		success = False

		# Precompute static AABBs for walls (for simple clearance metric)
		static_aabbs: List[Tuple[float, float, float, float]] = []
		for wid in wall_ids:
			(a0, a1) = p.getAABB(wid)
			static_aabbs.append((a0[0], a0[1], a1[0], a1[1]))

		def _human_phase_str() -> str:
			if fallen_human_id is not None:
				return "fallen"
			if crossing_active:
				return "running"
			return "none"

		def _compute_in_wet(xr: float, yr: float) -> bool:
			# scene-configured traction patches
			for patch in scn.get("hazards", {}).get("traction", []):
				x0, y0, x1, y1 = patch["zone"]
				if _pt_in_aabb(xr, yr, (float(x0), float(y0), float(x1), float(y1))):
					return True
			# ad-hoc crossing band
			if _pt_in_aabb(xr, yr, wet_band_aabb):
				return True
			return False

		# Camera config for terminal snapshots
		run_dir_name = out_dir.name  # 'run_0007'
		batch_root = out_dir.parent.parent  # .../runs/<batch>/  -> has 'per_run'
		cam_conf = {
			"cx": 0.5 * Lx,
			"cy": 0.5 * Ly,
			"cz": max(Lx, Ly) * 0.9 + 5.0,  # high enough for top-down
			"tx": 0.5 * Lx,
			"ty": 0.5 * Ly,
			"fov_deg": 30.0,
		}

		stop_until = 0.0  # proximity brake hold timer

		# Main loop
		while t < timeout:
			# Robot pose
			pos, orn = p.getBasePositionAndOrientation(amr)
			x, y, _ = pos
			yaw = p.getEulerFromQuaternion(orn)[2]

			# Goal waypoint
			target = waypoints[min(wp_idx, len(waypoints) - 1)]

			# PID XY -> desired linear speed; PD on yaw
			ctrl, prev_err = _pid_follow((x, y), target, prev_err, dt=dt)
			desired_yaw = math.atan2(target[1] - y, target[0] - x)
			yaw_err = (desired_yaw - yaw + math.pi) % (2 * math.pi) - math.pi
			w_cmd = max(-1.5, min(1.5, 1.2 * yaw_err))
			v_cmd = float(np.linalg.norm(ctrl))
			v_cmd = max(0.0, min(1.8, v_cmd))

			# --- Proximity brake vs human (front arc only) ---
			STOP_DIST = 1.2
			SLOW_DIST = 3.0
			STOP_HOLD = 0.8
			h_pos = None
			if use_human:
				if fallen_human_id is not None:
					h_pos = p.getBasePositionAndOrientation(fallen_human_id)[0]
				elif human_id is not None:
					h_pos = p.getBasePositionAndOrientation(human_id)[0]

			if h_pos is not None:
				hx, hy, _hz = h_pos
				d = math.hypot(hx - x, hy - y)
				# front arc check
				ang_to_h = math.atan2(hy - y, hx - x)
				rel = (ang_to_h - yaw + math.pi) % (2 * math.pi) - math.pi
				in_front = abs(rel) < (math.pi / 2.0)

				if in_front and d < STOP_DIST:
					stop_until = max(stop_until, t + STOP_HOLD)

				if t < stop_until:
					v_cmd = 0.0
				elif in_front and d < SLOW_DIST:
					v_cmd = min(v_cmd, 0.6)

			# ---- Human crossing logic ----
			if use_human and human_id is not None and fallen_human_id is None:
				# Trigger crossing when robot is close to the crossing x-line
				if not crossing_active and abs(x - cross_x) < trigger_distance:
					crossing_active = True
					crossing_started_at = t
					# place human at bottom edge facing +Y
					start_y = border
					p.resetBasePositionAndOrientation(
						human_id, [cross_x, start_y, 1.05], p.getQuaternionFromEuler([0, 0, 0])
					)

				if crossing_active:
					progress = min(1.0, (t - crossing_started_at) / max(1e-6, crossing_duration))
					yh = (1.0 - progress) * border + progress * (Ly - border)
					p.resetBasePositionAndOrientation(human_id, [cross_x, yh, 1.05], p.getQuaternionFromEuler([0, 0, 0]))

					# Slip when entering wet band (once)
					if not slipped and _pt_in_aabb(cross_x, yh, wet_band_aabb) and rng.random() < p_slip:
						slipped = True
						# convert to fallen obstacle
						p.resetBasePositionAndOrientation(
							human_id, [cross_x, yh, 0.25], p.getQuaternionFromEuler([math.pi / 2.0, 0.0, 0.0])
						)
						fallen_human_id = human_id
						human_id = None  # no longer running
						rows.append([t, x, y, yaw, v_cmd, w_cmd, min_clear,
						             "slip", f"{cross_x:.2f},{yh:.2f}", int(_compute_in_wet(x, y)),
						             "fallen", int(v_cmd < 0.2)])

					# Finished crossing without falling
					if human_id is not None and yh >= (Ly - border - 1e-3):
						crossing_active = False  # exited scene

			# Kinematic integrate robot motion
			new_yaw = yaw + w_cmd * dt
			new_x = x + (v_cmd * math.cos(new_yaw)) * dt
			new_y = y + (v_cmd * math.sin(new_yaw)) * dt
			# Clamp to bounds (keep within warehouse)
			new_x = max(border, min(Lx - border, new_x))
			new_y = max(border, min(Ly - border, new_y))
			p.resetBasePositionAndOrientation(amr, [new_x, new_y, radius * 0.5], p.getQuaternionFromEuler([0, 0, new_yaw]))

			# Waypoint arrival
			if math.hypot(target[0] - new_x, target[1] - new_y) < 0.5:
				wp_idx += 1
				if wp_idx >= len(waypoints):
					success = True
					in_wet = int(_compute_in_wet(new_x, new_y))
					rows.append([t, new_x, new_y, new_yaw, v_cmd, w_cmd, min_clear,
					             "success", "", in_wet, _human_phase_str(), int(v_cmd < 0.2)])
					# Terminal snapshot (subject to shared budget)
					save, _frames_root = should_save_frames(batch_root, "success")
					if save:
						_capture_frame(batch_root, run_dir_name, cam_conf, idx=0)
					break

			# Clearance vs. static geometry (walls)
			clearance = _clearance_to_aabbs(new_x, new_y, static_aabbs)
			min_clear = min(min_clear, clearance)

			# Contacts: only consider collisions with human/fallen human for outcome
			raw_contacts = p.getContactPoints(bodyA=amr)
			human_contact = False
			nonhuman_contact_count = 0
			for cp in raw_contacts:
				bodyA = cp[1]; bodyB = cp[2]
				if amr not in (bodyA, bodyB):
					continue
				other = bodyB if bodyA == amr else bodyA
				if other in ignored_for_collision:
					continue
				if other == fallen_human_id or other == human_id:
					human_contact = True
					break
				nonhuman_contact_count += 1

			if human_contact:
				in_wet = int(_compute_in_wet(new_x, new_y))
				rows.append([t, new_x, new_y, new_yaw, v_cmd, w_cmd, 0.0,
				             "collision_human", "1", in_wet, _human_phase_str(), int(v_cmd < 0.2)])
				# Terminal snapshot (subject to shared budget)
				save, _frames_root = should_save_frames(batch_root, "collision_human")
				if save:
					_capture_frame(batch_root, run_dir_name, cam_conf, idx=0)
				break
			elif nonhuman_contact_count > 0:
				# Non-human contacts are informative but do not terminate the run
				in_wet = int(_compute_in_wet(new_x, new_y))
				rows.append([t, new_x, new_y, new_yaw, v_cmd, w_cmd, min_clear,
				             "contact_nonhuman", str(nonhuman_contact_count), in_wet, _human_phase_str(), int(v_cmd < 0.2)])

			# Regular log row
			in_wet = int(_compute_in_wet(new_x, new_y))
			rows.append([t, new_x, new_y, new_yaw, v_cmd, w_cmd, min_clear,
			             "", "", in_wet, _human_phase_str(), int(v_cmd < 0.2)])

			# Step physics & time
			p.stepSimulation()
			if realtime:
				time.sleep(dt)
			t += dt

		# Write CSV
		_write_csv(out_dir / "run_one.csv", header, rows)

		return {"success": success, "time": t, "steps": len(rows)}

	finally:
		# Always disconnect this Bullet client so batches don't leak handles
		try:
			p.disconnect(client_id)
		except Exception:
			pass

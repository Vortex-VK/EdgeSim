# EdgeSim V0 — sim_one.py (full)
# Changes vs your last version:
# - Waypoints use CELL CENTERS to avoid illegal targets along borders.
# - Stall detector: if no goal-distance improvement for STALL_GRACE sec -> _replan("stall", ...).
# - Human fallback: if no crossing by 5s, start 1 crossing so taxonomy isn't empty (then same slip/fall logic).
# - Safe inits for v_cmd, w_cmd, min_clear_geom to avoid unbound variables in early logs.
# - Portable LiDAR: rayTestBatch without collisionFilterMask (Windows overflow fix) + per-ray fallback.
# - Realistic sensors (Step 3): LiDAR 360@10 Hz with 1–2% dropout, Gaussian noise; latency-jitter queue (0–50ms);
#   odometry with drift (bias + noise). We log min_clearance_lidar and min_clearance_geom.
# - CSV columns match spec: t,x,y,yaw,v_cmd,w_cmd,min_clearance_lidar,min_clearance_geom,event,event_detail,in_wet,human_phase,near_stop

from __future__ import annotations
from typing import Dict, Any, List, Tuple
from pathlib import Path
import csv, math, time, random

import numpy as np
import pybullet as p

from .planner import rasterize_occupancy, astar, inflate_grid
from .world import build_world, spawn_human
from .sampling import should_save_frames  # outcome-aware shared budget
from .world_digest import build_world_digest, write_world_digest

# ---------- small I/O helpers ----------

def _write_csv(path: Path, header: List[str], rows: List[List[float | str]]) -> None:
	path.parent.mkdir(parents=True, exist_ok=True)
	with path.open("w", newline="", encoding="utf-8") as f:
		w = csv.writer(f)
		w.writerow(header)
		w.writerows(rows)

# ---------- geometry & controls ----------

def _spawn_disc(radius: float = 0.4, height: float = 0.2, mass: float = 20.0, color=(0.9, 0.3, 0.3, 1.0)) -> int:
	vis = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height, rgbaColor=color)
	col = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)
	body = p.createMultiBody(
		baseMass=mass,
		baseCollisionShapeIndex=col,
		baseVisualShapeIndex=vis,
		basePosition=[1.0, 1.0, height / 2.0],
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

# ---------- simple frame capture ----------

def _capture_frame(batch_root: Path, run_dir_name: str, cam_conf: Dict[str, Any], width: int = 640, height: int = 480, idx: int = 0) -> None:
	view_m = p.computeViewMatrix(
		cameraEyePosition=[cam_conf["cx"], cam_conf["cy"], cam_conf["cz"]],
		cameraTargetPosition=[cam_conf["tx"], cam_conf["ty"], 0.0],
		cameraUpVector=[0.0, 1.0, 0.0],
	)
	proj_m = p.computeProjectionMatrixFOV(fov=cam_conf["fov_deg"], aspect=width/height, nearVal=0.01, farVal=100.0)
	_, _, px, _, _ = p.getCameraImage(width, height, view_m, proj_m, renderer=p.ER_TINY_RENDERER)

	img = np.reshape(np.array(px, dtype=np.uint8), (height, width, 4))[:, :, :3]
	out_dir = batch_root / "frames_sample" / run_dir_name
	out_dir.mkdir(parents=True, exist_ok=True)
	out_path = out_dir / f"{idx:03d}.png"
	try:
		p.writeImageFile(str(out_path), img)  # type: ignore[attr-defined]
	except Exception:
		try:
			import imageio.v2 as imageio
			imageio.imwrite(out_path, img)
		except Exception:
			return

# ---------- helpers for grid/waypoints & replanning ----------

def _to_cell(px: float, py: float, res: float, grid: List[List[int]]) -> tuple[int, int]:
	ix = max(0, min(int(px / res), len(grid[0]) - 1))
	iy = max(0, min(int(py / res), len(grid) - 1))
	return ix, iy

def _to_world(ix: int, iy: int, res: float) -> tuple[float, float]:
	# Center of cell (prevents waypoints at border clamp)
	return ((ix + 0.5) * res, (iy + 0.5) * res)

def _cells_to_waypoints(path_cells: List[Tuple[int,int]], res: float, min_gap: float = 0.5) -> List[Tuple[float,float]]:
	waypoints: List[Tuple[float,float]] = []
	last = (-999.0, -999.0)
	for (ix, iy) in path_cells:
		wx, wy = _to_world(ix, iy, res)
		if not waypoints or math.hypot(wx - last[0], wy - last[1]) > min_gap:
			waypoints.append((wx, wy))
			last = (wx, wy)
	return waypoints

def _inflate(aabb: Tuple[float,float,float,float], r: float) -> Tuple[float,float,float,float]:
	x0, y0, x1, y1 = aabb
	return (x0 - r, y0 - r, x1 + r, y1 + r)

def _fallen_human_aabb(hid: int | None) -> Tuple[float,float,float,float] | None:
	if hid is None: return None
	try:
		a0, a1 = p.getAABB(hid)
		return (a0[0], a0[1], a1[0], a1[1])
	except Exception:
		return None

# ---------- Sensors (LiDAR, Odom, latency jitter) ----------

class _LidarSim:
	def __init__(self, beams: int = 360, hz: float = 10.0, max_range: float = 8.0,
	             dropout_pct_range=(0.01, 0.02), noise_sigma: float = 0.02,
	             jitter_ms_max: int = 50):
		self.beams = int(beams)
		self.hz = float(hz)
		self.dt = 1.0 / max(1e-6, self.hz)
		self.max_range = float(max_range)
		self.dropout_low, self.dropout_high = float(dropout_pct_range[0]), float(dropout_pct_range[1])
		self.noise_sigma = float(noise_sigma)
		self.next_update_t = 0.0
		self.angles = np.linspace(-math.pi, math.pi, self.beams, endpoint=False)
		self._rng = np.random.default_rng()
		# latency jitter queue (timestamps when readings become available)
		self._queue: List[Tuple[float, np.ndarray]] = []
		self._jitter_s = float(jitter_ms_max) / 1000.0

	def maybe_update(self, t: float, x: float, y: float, yaw: float, sensor_z: float, ignore_ids: set[int]) -> None:
		if t < self.next_update_t:
			return
		self.next_update_t = t + self.dt

		# Ray origins/ends for all beams
		cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
		dir_x = np.cos(self.angles); dir_y = np.sin(self.angles)
		rot_x = cos_yaw * dir_x - sin_yaw * dir_y
		rot_y = sin_yaw * dir_x + cos_yaw * dir_y

		origins = [[x, y, sensor_z]] * self.beams
		ends = [[x + float(rx) * self.max_range, y + float(ry) * self.max_range, sensor_z] for rx, ry in zip(rot_x, rot_y)]

		# Portable batch rays
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
			if hit_uid in ignore_ids or hit_uid == -1:
				d = self.max_range
			else:
				d = float(hit_frac) * self.max_range
			dists[i] = d

		# Apply dropout & Gaussian noise
		dropout_pct = float(self._rng.uniform(self.dropout_low, self.dropout_high))
		if dropout_pct > 0.0:
			k = max(1, int(round(dropout_pct * self.beams)))
			idx = self._rng.choice(self.beams, size=k, replace=False)
			dists[idx] = self.max_range
		if self.noise_sigma > 0:
			dists = np.clip(dists + self._rng.normal(0.0, self.noise_sigma, size=self.beams), 0.0, self.max_range)

		# Push to latency queue with jitter
		avail_t = t + float(self._rng.uniform(0.0, self._jitter_s))
		self._queue.append((avail_t, dists))

	def try_pop_ready(self, t: float) -> np.ndarray | None:
		if not self._queue:
			return None
		# pop left-most ready sample
		self._queue.sort(key=lambda x: x[0])
		if self._queue[0][0] <= t:
			return self._queue.pop(0)[1]
		return None

class _OdomSim:
	def __init__(self, bias_v: float = 0.02, bias_w: float = 0.01, noise_v: float = 0.02, noise_w: float = 0.01):
		self.bias_v = float(bias_v); self.bias_w = float(bias_w)
		self.noise_v = float(noise_v); self.noise_w = float(noise_w)
		self.rng = np.random.default_rng()

	def apply(self, v_cmd: float, w_cmd: float) -> Tuple[float, float]:
		v = v_cmd * (1.0 + self.bias_v) + float(self.rng.normal(0.0, self.noise_v))
		w = w_cmd * (1.0 + self.bias_w) + float(self.rng.normal(0.0, self.noise_w))
		return v, w

# ---------- main single-run ----------

def run_one(
	prompt: str,
	scn: Dict[str, Any],
	out_dir: Path,
	dt: float = 0.05,
	realtime: bool = False,
	gui: bool = False,
) -> Dict[str, Any]:

	# Allow scenario dt override
	if "runtime" in scn and "dt" in scn["runtime"]:
		try:
			dt = float(scn["runtime"]["dt"])
		except Exception:
			pass

	# Build world
	env = build_world(scn, use_gui=gui)
	client_id = env["client"]
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

		# Human hazard
		use_human = bool(scn.get("hazards", {}).get("human"))
		human_id: int | None = spawn_human() if use_human else None
		fallen_human_id: int | None = None

		if human_id is not None:
			# V0 defaults (override via scenario.hazards.human[0])
			cfg = (scn.get("hazards", {}).get("human") or [{}])[0]
			cross_x = float(cfg.get("cross_x", 0.5 * Lx))
			crossing_duration = float(cfg.get("duration_s_running", 2.5))
			p_slip = float(cfg.get("p_slip", 0.45))
			trigger_mu = float(cfg.get("trigger_mu", 1.2))
			trigger_sigma = float(cfg.get("trigger_sigma", 0.4))
			trigger_distance = float(max(0.3, rng.normal(trigger_mu, trigger_sigma)))

			# Narrower wet band along crossing
			wet_band_h = 0.9
			wet_y0 = max(border, 0.5 * Ly - 0.5 * wet_band_h)
			wet_y1 = min(Ly - border, 0.5 * wet_band_h + 0.5 * Ly)
			wet_id = _spawn_wet_patch(cross_x - 1.3, wet_y0, cross_x + 1.3, wet_y1, mu=0.32)
			patch_ids.append(wet_id)
			ignored_for_collision.add(wet_id)
			wet_band_aabb: Tuple[float, float, float, float] = (cross_x - 1.3, wet_y0, cross_x + 1.3, wet_y1)
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
			"min_clearance_lidar", "min_clearance_geom",
			"event", "event_detail", "in_wet", "human_phase", "near_stop"
		]

		# Precompute static AABBs for walls
		static_aabbs: List[Tuple[float, float, float, float]] = []
		for wid in wall_ids:
			(a0, a1) = p.getAABB(wid)
			static_aabbs.append((a0[0], a0[1], a1[0], a1[1]))

		# === NEW: world digest (deterministic layout snapshot) ===
		try:
			hazards_digest = {
				"traction": [dict(zone=list(z["zone"]), mu=float(z.get("mu", 0.45)))
				             for z in scn.get("hazards", {}).get("traction", []) if isinstance(z, dict)],
				"human": (scn.get("hazards", {}).get("human") or [{}]),
			}
			digest = build_world_digest(static_aabbs, hazards_digest, (start[0], start[1]), (goal[0], goal[1]))
			write_world_digest(out_dir, digest)
		except Exception:
			pass  # non-fatal in V0

		try:
			batch_root = out_dir.parent  # if out_dir is runs/<batch>/per_run/run_xxxx, then parent is runs/<batch>/per_run
			if batch_root.name == "per_run":
				batch_root = batch_root.parent  # now runs/<batch>
			root_world = batch_root / "world.json"
			if not root_world.exists():
				root_world.write_text((out_dir / "world.json").read_text(encoding="utf-8"), encoding="utf-8")
				# propagate hash into the batch manifest
				import json, hashlib
				h = hashlib.sha256(root_world.read_bytes()).hexdigest()
				man_path = batch_root / "manifest.json"
				try:
					man = json.loads(man_path.read_text(encoding="utf-8"))
				except Exception:
					man = {}
				man["world_sha256"] = h
				man_path.write_text(json.dumps(man, indent=2), encoding="utf-8")
		except Exception:
			pass

		# Initial path (A* with inflation)
		grid, res = rasterize_occupancy((Lx, Ly), [(_inflate(a, radius)) for a in static_aabbs], res=0.25)
		inflate_cells = max(0, int(round(radius / max(1e-6, res))))
		grid_infl = inflate_grid(grid, inflate_cells)  # double-safe inflation (AABB expand + grid dilate)
		s_ix, s_iy = _to_cell(start[0], start[1], res, grid_infl)
		g_ix, g_iy = _to_cell(goal[0], goal[1], res, grid_infl)
		path_cells = astar(grid_infl, (s_ix, s_iy), (g_ix, g_iy))
		waypoints: List[Tuple[float, float]] = _cells_to_waypoints(path_cells, res) or [(goal[0], goal[1])]

		# Camera config for terminal snapshots
		run_dir_name = out_dir.name
		batch_root = out_dir.parent.parent
		cam_conf = {"cx": 0.5 * Lx, "cy": 0.5 * Ly, "cz": max(Lx, Ly) * 0.9 + 5.0, "tx": 0.5 * Lx, "ty": 0.5 * Ly, "fov_deg": 30.0}

		# Control & stopping
		STOP_DIST = 1.4
		SLOW_DIST = 3.6
		STOP_HOLD = 1.0
		stop_until = 0.0
		prev_err = np.array([0.0, 0.0])
		wp_idx = 0
		success = False
		t = 0.0
		timeout = float(scn.get("runtime", {}).get("duration_s", 90.0))
		min_clear_legacy = 10.0
		min_clear_geom = 10.0
		v_cmd = 0.0
		w_cmd = 0.0

		# Stall detection toward goal
		best_goal_dist = float("inf")
		last_progress_at = 0.0
		STALL_GRACE = float(scn.get("planner", {}).get("stall_grace_s", 3.0))

		def _human_phase_str() -> str:
			if fallen_human_id is not None:
				return "fallen"
			if crossing_active:
				return "running"
			return "none"

		def _compute_in_wet(xr: float, yr: float) -> bool:
			for patch in scn.get("hazards", {}).get("traction", []):
				x0, y0, x1, y1 = patch["zone"]
				if _pt_in_aabb(xr, yr, (float(x0), float(y0), float(x1), float(y1))):
					return True
			if _pt_in_aabb(xr, yr, wet_band_aabb):
				return True
			return False

		def _min_clear_geom(px: float, py: float, ry: float) -> float:
			"""Clearance to walls + human (fallen footprint or running slice)."""
			dyn = list(static_aabbs)
			if fallen_human_id is not None:
				a0, a1 = p.getAABB(fallen_human_id)
				fh = (a0[0], a0[1], a1[0], a1[1])
				inflate = 0.3 * radius
				fh = (fh[0] - inflate, fh[1] - inflate, fh[2] + inflate, fh[3] + inflate)
				dyn.append(fh)
			if human_id is not None and crossing_active:
				slice_w = 0.25
				aabb = (cross_x - slice_w, max(0.0, ry - 0.6), cross_x + slice_w, min(Ly, ry + 0.6))
				dyn.append(aabb)
			return _clearance_to_aabbs(px, py, dyn)

		def _replan(reason: str, cur_x: float, cur_y: float) -> None:
			nonlocal waypoints, wp_idx, grid, res
			# Compose obstacles: static (inflated) + fallen human (inflated)
			dyn_obs: List[Tuple[float, float, float, float]] = [(_inflate(a, radius)) for a in static_aabbs]
			fh_aabb = _fallen_human_aabb(fallen_human_id)
			if fh_aabb is not None:
				dyn_obs.append(_inflate(fh_aabb, radius * 1.1))
			grid, res = rasterize_occupancy((Lx, Ly), dyn_obs, res=0.25)
			inflate_cells = max(0, int(round(radius / max(1e-6, res))))
			grid_infl = inflate_grid(grid, inflate_cells)
			ci, cj = _to_cell(cur_x, cur_y, res, grid_infl)
			gi, gj = _to_cell(goal[0], goal[1], res, grid_infl)
			new_cells = astar(grid_infl, (ci, cj), (gi, gj))
			new_wps = _cells_to_waypoints(new_cells, res)
			if new_wps:
				waypoints = new_wps
				wp_idx = 0
				rows.append([t, cur_x, cur_y, yaw, v_cmd, w_cmd,
							 min_clear_lidar if 'min_clear_lidar' in locals() else 10.0, min_clear_geom,
							 "replan", reason, int(_compute_in_wet(cur_x, cur_y)),
							 _human_phase_str(), int(v_cmd < 0.2)])

		# Sensors
		lidar_hz = float(((scn.get("sensors", {}) or {}).get("lidar", {}) or {}).get("hz", 10.0))
		lidar = _LidarSim(beams=360, hz=lidar_hz, max_range=8.0, dropout_pct_range=(0.01, 0.02), noise_sigma=0.02, jitter_ms_max=50)
		odom = _OdomSim(bias_v=0.02, bias_w=0.01, noise_v=0.02, noise_w=0.01)
		sensor_z = radius * 0.9
		min_clear_lidar = 10.0

		# Main loop
		while t < timeout:
			# Pose
			pos, orn = p.getBasePositionAndOrientation(amr)
			x, y, _ = pos
			yaw = p.getEulerFromQuaternion(orn)[2]

			# Target waypoint
			target = waypoints[min(wp_idx, len(waypoints) - 1)]

			# PID & yaw PD
			ctrl, prev_err = _pid_follow((x, y), target, prev_err, dt=dt)
			desired_yaw = math.atan2(target[1] - y, target[0] - x)
			yaw_err = (desired_yaw - yaw + math.pi) % (2 * math.pi) - math.pi
			w_cmd = max(-1.5, min(1.5, 1.2 * yaw_err))
			v_cmd = float(np.linalg.norm(ctrl))
			v_cmd = max(0.0, min(1.8, v_cmd))

			# Human proximity brake
			h_pos = None
			if use_human:
				if fallen_human_id is not None:
					h_pos = p.getBasePositionAndOrientation(fallen_human_id)[0]
				elif human_id is not None:
					h_pos = p.getBasePositionAndOrientation(human_id)[0]

			if h_pos is not None:
				hx, hy, _hz = h_pos
				d = math.hypot(hx - x, hy - y)
				ang_to_h = math.atan2(hy - y, hx - x)
				rel = (ang_to_h - yaw + math.pi) % (2 * math.pi) - math.pi
				in_front = abs(rel) < (math.pi / 2.0)

				if in_front and d < STOP_DIST:
					stop_until = max(stop_until, t + STOP_HOLD)

				if t < stop_until:
					v_cmd = 0.0
				elif in_front and d < SLOW_DIST:
					v_cmd = min(v_cmd, 0.6)

			# Human crossing logic (fallback + distance trigger)
			if use_human and human_id is not None and fallen_human_id is None:
				# Fallback trigger at 5s
				if (not crossing_active) and (t >= 5.0):
					crossing_active = True
					crossing_started_at = t
					start_y = border
					p.resetBasePositionAndOrientation(human_id, [cross_x, start_y, 1.05], p.getQuaternionFromEuler([0, 0, 0]))

				if not crossing_active and abs(x - cross_x) < trigger_distance:
					crossing_active = True
					crossing_started_at = t
					start_y = border
					p.resetBasePositionAndOrientation(human_id, [cross_x, start_y, 1.05], p.getQuaternionFromEuler([0, 0, 0]))

				if crossing_active:
					progress = min(1.0, (t - crossing_started_at) / max(1e-6, crossing_duration))
					yh = (1.0 - progress) * border + progress * (Ly - border)
					p.resetBasePositionAndOrientation(human_id, [cross_x, yh, 1.05], p.getQuaternionFromEuler([0, 0, 0]))

					if not slipped and _pt_in_aabb(cross_x, yh, wet_band_aabb) and rng.random() < p_slip:
						slipped = True
						p.resetBasePositionAndOrientation(human_id, [cross_x, yh, 0.25], p.getQuaternionFromEuler([math.pi / 2.0, 0.0, 0.0]))
						fallen_human_id = human_id
						human_id = None
						rows.append([t, x, y, yaw, v_cmd, w_cmd,
									min_clear_lidar, min_clear_geom,
									"slip", f"{cross_x:.2f},{yh:.2f}", int(_compute_in_wet(x, y)),
									"fallen", int(v_cmd < 0.2)])
						# immediate replan on fallen hazard
						_replan("fallen_block", x, y)

					if human_id is not None and yh >= (Ly - border - 1e-3):
						crossing_active = False

			# Sensors update
			lidar_ignore = ignored_for_collision | {amr}
			if human_id is not None:
				lidar_ignore.add(human_id)
			if fallen_human_id is not None:
				lidar_ignore.add(fallen_human_id)

			lidar.maybe_update(t, x, y, yaw, sensor_z, lidar_ignore)
			lr = lidar.try_pop_ready(t)
			if lr is not None:
				min_clear_lidar = float(np.min(lr))

			# Use odometry drifted cmd for integration (simple)
			ov, ow = odom.apply(v_cmd, w_cmd)

			# Integrate motion
			new_yaw = yaw + ow * dt
			new_x = x + (ov * math.cos(new_yaw)) * dt
			new_y = y + (ov * math.sin(new_yaw)) * dt
			new_x = max(border, min(Lx - border, new_x))
			new_y = max(border, min(Ly - border, new_y))
			p.resetBasePositionAndOrientation(amr, [new_x, new_y, radius * 0.5], p.getQuaternionFromEuler([0, 0, new_yaw]))

			# Waypoint arrival
			if math.hypot(target[0] - new_x, target[1] - new_y) < 0.5:
				wp_idx += 1
				if wp_idx >= len(waypoints):
					success = True
					in_wet = int(_compute_in_wet(new_x, new_y))
					rows.append([t, new_x, new_y, new_yaw, v_cmd, w_cmd,
								min_clear_lidar, min_clear_geom,
								"success", "", in_wet, _human_phase_str(), int(v_cmd < 0.2)])
					save, _ = should_save_frames(batch_root, "success")
					if save: _capture_frame(batch_root, run_dir_name, cam_conf, idx=0)
					break

			# Clearance metrics (geom)
			cur_geom = _min_clear_geom(new_x, new_y, new_y)
			min_clear_geom = min(min_clear_geom, cur_geom)

			# Stall detection & replan
			goal_dist = math.hypot(goal[0] - new_x, goal[1] - new_y)
			if goal_dist < (best_goal_dist - 0.1):
				best_goal_dist = goal_dist
				last_progress_at = t
			elif (t - last_progress_at) >= STALL_GRACE:
				_replan("stall", new_x, new_y)
				last_progress_at = t

			# Low-clearance legacy (kept; optional minclr triggers if configured)
			MINCLR_THRESH = float(scn.get("planner", {}).get("minclr_replan_m", 0.35))
			MINCLR_GRACE = float(scn.get("planner", {}).get("minclr_grace_s", 0.6))
			if cur_geom < MINCLR_THRESH:
				if 'minclr_below_since' not in locals() or minclr_below_since is None:
					minclr_below_since = t
				elif (t - minclr_below_since) >= MINCLR_GRACE:
					_replan("low_clearance", new_x, new_y)
					minclr_below_since = None
			else:
				minclr_below_since = None  # type: ignore

			# Contacts & terminal outcomes
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
				rows.append([t, new_x, new_y, new_yaw, v_cmd, w_cmd,
							min_clear_lidar, min_clear_geom,
							"collision_human", "1", in_wet, _human_phase_str(), int(v_cmd < 0.2)])
				save, _ = should_save_frames(batch_root, "collision_human")
				if save: _capture_frame(batch_root, run_dir_name, cam_conf, idx=0)
				break
			elif nonhuman_contact_count > 0:
				in_wet = int(_compute_in_wet(new_x, new_y))
				rows.append([t, new_x, new_y, new_yaw, v_cmd, w_cmd,
							min_clear_lidar, min_clear_geom,
							"contact_nonhuman", str(nonhuman_contact_count), in_wet, _human_phase_str(), int(v_cmd < 0.2)])

			# Regular log row
			in_wet = int(_compute_in_wet(new_x, new_y))
			rows.append([t, new_x, new_y, new_yaw, v_cmd, w_cmd,
						min_clear_lidar, min_clear_geom,
						"", "", in_wet, _human_phase_str(), int(v_cmd < 0.2)])

			# Step sim
			p.stepSimulation()
			if realtime: time.sleep(dt)
			t += dt

		# If we exited by timeout, write a terminal row if not already written
		if not success and (not rows or (rows and not rows[-1][8])):
			in_wet = int(_compute_in_wet(new_x, new_y))
			rows.append([t, new_x, new_y, new_yaw, v_cmd, w_cmd,
						min_clear_lidar, min_clear_geom,
						"other", "timeout", in_wet, _human_phase_str(), int(v_cmd < 0.2)])

		# Write CSV
		_write_csv(out_dir / "run_one.csv", header, rows)
		return {"success": success, "time": t, "steps": len(rows)}

	finally:
		try: p.disconnect(client_id)
		except Exception: pass

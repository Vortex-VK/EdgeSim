# EdgeSim V0 — sim_one.py (balanced)

from __future__ import annotations
from typing import Dict, Any, List, Tuple, Optional
from pathlib import Path
import csv, math, time, os, json

import numpy as np
import pybullet as p

from .planner import rasterize_occupancy, astar, inflate_grid
from .world import build_world, spawn_human
from .sampling import should_save_frames
from .world_digest import build_world_digest, write_world_digest
from .injectors import (
	LidarSectorBlackoutInjector, FallingObjectInjector, ThrownObjectInjector,
	InjectorState
)

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

# ---------- Site profile loader ----------

def _load_site_tuned() -> Dict[str, Any]:
	site = os.environ.get("EDGESIM_SITE", "").strip()
	if not site:
		return {}
	path = os.path.join("site_profiles", f"{site}.json")
	if not os.path.exists(path):
		return {}
	try:
		with open(path, "r", encoding="utf-8") as f:
			obj = json.load(f)
		return obj.get("tuned", {})
	except Exception:
		return {}

# ---------- Sensors ----------

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
		self._queue: List[Tuple[float, np.ndarray]] = []
		self._jitter_s = float(jitter_ms_max) / 1000.0

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

		dropout_pct = float(self._rng.uniform(self.dropout_low, self.dropout_high))
		if dropout_pct > 0.0:
			k = max(1, int(round(dropout_pct * self.beams)))
			idx = self._rng.choice(self.beams, size=k, replace=False)
			dists[idx] = self.max_range
		if self.noise_sigma > 0:
			dists = np.clip(dists + self._rng.normal(0.0, self.noise_sigma, size=self.beams), 0.0, self.max_range)

		avail_t = t + float(self._rng.uniform(0.0, self._jitter_s))
		self._queue.append((avail_t, dists))

	def try_pop_ready(self, t: float) -> np.ndarray | None:
		if not self._queue:
			return None
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

	if "runtime" in scn and "dt" in scn["runtime"]:
		try:
			dt = float(scn["runtime"]["dt"])
		except Exception:
			pass

	env = build_world(scn, use_gui=gui)
	client_id = env["client"]
	try:
		Lx, Ly = env["bounds"]
		plane_id = env["plane_id"]
		wall_ids = list(env.get("walls", []))
		patch_ids = list(env.get("patches", []))
		ignored_for_collision = {plane_id, *patch_ids}

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

		# ---- tuned knobs / defaults ----
		tr_cfg = (_tuned.get("traction") or {})
		mu_wet_min = float(tr_cfg.get("mu_wet_min", 0.35))
		mu_wet_max = float(tr_cfg.get("mu_wet_max", 0.60))

		h_cfg = (_tuned.get("human") or {})
		tuned_slip_prob = float(h_cfg.get("slip_prob", 0.85))               # moderate default
		slip_min_exposure_s = float(h_cfg.get("slip_min_exposure_s", 0.06))  # brief exposure
		replan_on_fall_p = float(h_cfg.get("replan_on_fall_p", 0.5))        # only sometimes replan
		tuned_fall_duration = float(h_cfg.get("fall_duration_s", 6.0))

		l_cfg = (_tuned.get("lidar") or {})
		tuned_lidar_noise = float(l_cfg.get("noise_sigma", 0.02))
		tuned_lidar_dropout = float(l_cfg.get("dropout", 0.01))
		tuned_lidar_jitter = int(l_cfg.get("latency_ms_max", 50))

		o_cfg = (_tuned.get("odom") or {})
		tuned_bias_v = float(o_cfg.get("bias_v", 0.02))
		tuned_bias_w = float(o_cfg.get("bias_w", 0.01))

		brake_cfg = (_tuned.get("brake") or {})
		STOP_DIST = float(brake_cfg.get("stop_dist_m", 1.0))   # was 1.4
		SLOW_DIST = float(brake_cfg.get("slow_dist_m", 2.4))   # was 3.6
		STOP_HOLD = float(brake_cfg.get("stop_hold_s", 0.8))   # was 1.0

		# per-run rng (salted by run id)
		seed_val = int(scn.get("seed", 0))
		try:
			run_salt = int(''.join(ch for ch in out_dir.name if ch.isdigit()) or "0")
		except Exception:
			run_salt = 0
		seed_val = (seed_val * 1664525 + 1013904223 + run_salt) & 0xFFFFFFFF
		rng = np.random.default_rng(seed_val)

		dynamic_wet_aabbs: List[Tuple[float, float, float, float]] = []

		if gui:
			p.resetDebugVisualizerCamera(
				cameraDistance=10.0,
				cameraYaw=45.0,
				cameraPitch=-35.0,
				cameraTargetPosition=[0.5 * Lx, 0.5 * Ly, 0.0],
			)

		# sample a single μ_wet for this run and apply to existing patches
		wet_mu = float(rng.uniform(mu_wet_min, mu_wet_max))
		for bid in patch_ids:
			p.changeDynamics(bid, -1, lateralFriction=wet_mu)

		radius = float(scn.get("agents", [{}])[0].get("radius_m", 0.4))
		amr = _spawn_disc(radius=radius)

		border = 0.6
		start = list(map(float, scn.get("layout", {}).get("start", [border, border])))
		goal = list(map(float, scn.get("layout", {}).get("goal", [Lx - border, Ly - border])))
		start[0] = max(border, min(Lx - border, start[0]))
		start[1] = max(border, min(Ly - border, start[1]))
		goal[0] = max(border, min(Lx - border, goal[0]))
		goal[1] = max(border, min(Ly - border, goal[1]))
		p.resetBasePositionAndOrientation(amr, [start[0], start[1], radius * 0.5], p.getQuaternionFromEuler([0, 0, 0]))

		# Optional wet corridor (not every run)
		ensure_corr_p = float(_tuned.get("ensure_wet_corridor_pct", 0.0))
		corr_w = float(_tuned.get("ensure_wet_corridor_width_m", 1.2))
		if ensure_corr_p > 0.0 and rng.random() < ensure_corr_p:
			midx = 0.5 * Lx
			x0, x1 = max(0.0, midx - 0.5 * corr_w), min(Lx, midx + 0.5 * corr_w)
			y0, y1 = 0.0 + 0.6, Ly - 0.6
			corr_id = _spawn_wet_patch(x0, y0, x1, y1, mu=wet_mu)
			patch_ids.append(corr_id)
			ignored_for_collision.add(corr_id)
			dynamic_wet_aabbs.append((x0, y0, x1, y1))

		# Human hazard
		use_human = bool(scn.get("hazards", {}).get("human"))
		human_id: int | None = spawn_human() if use_human else None
		fallen_human_id: int | None = None

		if human_id is not None:
			cfg = (scn.get("hazards", {}).get("human") or [{}])[0]
			cross_x = float(cfg.get("cross_x", 0.5 * Lx))
			crossing_duration = float(cfg.get("duration_s_running", 4.5))
			p_slip = float(cfg.get("p_slip", tuned_slip_prob))
			trigger_mu = float(cfg.get("trigger_mu", 1.0))       # slightly earlier triggers
			trigger_sigma = float(cfg.get("trigger_sigma", 0.3))
			trigger_distance = float(max(0.3, rng.normal(trigger_mu, trigger_sigma)))

			wet_band_h = 2.2                                     # narrower band → more variability
			wet_y0 = max(border, 0.5 * Ly - 0.5 * wet_band_h)
			wet_y1 = min(Ly - border, 0.5 * wet_band_h + 0.5 * Ly)
			wet_id = _spawn_wet_patch(cross_x - 1.0, wet_y0, cross_x + 1.0, wet_y1, mu=wet_mu)
			patch_ids.append(wet_id)
			ignored_for_collision.add(wet_id)
			wet_band_aabb: Tuple[float, float, float, float] = (cross_x - 1.0, wet_y0, cross_x + 1.0, wet_y1)
		else:
			cross_x = 0.5 * Lx
			crossing_duration = 0.0
			p_slip = 0.0
			trigger_distance = 9999.0
			wet_band_aabb = None  # type: ignore

		# Injectors (each gated by probability p if provided)
		injectors = []  # type: ignore[list-item-type]
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

		if isinstance(inj_cfg.get("falling_object"), dict):
			c = inj_cfg["falling_object"]
			if rng.random() < float(c.get("p", 0.40)):
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
					dynamic_wet_aabbs.append((float(x0), float(y0), float(x1), float(y1)))
					return bid
				inj.spawn_wet_patch_cb = _spawn_patch_cb
				injectors.append(inj)

		if isinstance(inj_cfg.get("thrown_object"), dict):
			c = inj_cfg["thrown_object"]
			if rng.random() < float(c.get("p", 0.35)):
				injectors.append(ThrownObjectInjector(
					origin=tuple(map(float, c.get("origin", [6.0, 3.0]))),
					target=tuple(map(float, c.get("target", [6.0, 17.0]))),
					z0=float(c.get("z0", 1.2)),
					speed_mps=float(c.get("speed_mps", 6.0)),
					mass=float(c.get("mass", 0.5)),
					radius=float(c.get("radius", 0.12)),
					restitution=float(c.get("restitution", 0.3)),
					launch_within_r=float(c.get("launch_within_r", 4.0)),
				))

		# Human state
		crossing_active = False
		crossing_started_at = 0.0
		slipped = False
		human_wet_since: float | None = None

		# Logging
		out_dir.mkdir(parents=True, exist_ok=True)
		rows: List[List[float | str]] = []
		header = [
			"t", "x", "y", "yaw", "v_cmd", "w_cmd",
			"min_clearance_lidar", "min_clearance_geom",
			"event", "event_detail", "in_wet", "human_phase", "near_stop",
			"min_ttc"
		]

		# Static AABBs
		static_aabbs: List[Tuple[float, float, float, float]] = []
		for wid in wall_ids:
			(a0, a1) = p.getAABB(wid)
			static_aabbs.append((a0[0], a0[1], a1[0], a1[1]))

		# World digest (best-effort)
		try:
			hazards_digest = {
				"traction": [dict(zone=list(z["zone"]), mu=float(z.get("mu", 0.45)))
				             for z in scn.get("hazards", {}).get("traction", []) if isinstance(z, dict)],
				"human": (scn.get("hazards", {}).get("human") or [{}]),
			}
			digest = build_world_digest(static_aabbs, hazards_digest, (start[0], start[1]), (goal[0], goal[1]))
			write_world_digest(out_dir, digest)
		except Exception:
			pass

		try:
			batch_root = out_dir.parent
			if batch_root.name == "per_run":
				batch_root = batch_root.parent
			root_world = batch_root / "world.json"
			if not root_world.exists():
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

		# Plan initial path
		grid, res = rasterize_occupancy((Lx, Ly), [(_inflate(a, radius)) for a in static_aabbs], res=0.25)
		inflate_cells = max(0, int(round(radius / max(1e-6, res))))
		grid_infl = inflate_grid(grid, inflate_cells)
		s_ix, s_iy = _to_cell(start[0], start[1], res, grid_infl)
		g_ix, g_iy = _to_cell(goal[0], goal[1], res, grid_infl)
		path_cells = astar(grid_infl, (s_ix, s_iy), (g_ix, g_iy))
		waypoints: List[Tuple[float, float]] = _cells_to_waypoints(path_cells, res) or [(goal[0], goal[1])]

		# Camera
		run_dir_name = out_dir.name
		batch_root = out_dir.parent.parent
		cam_conf = {"cx": 0.5 * Lx, "cy": 0.5 * Ly, "cz": max(Lx, Ly) * 0.9 + 5.0, "tx": 0.5 * Lx, "ty": 0.5 * Ly, "fov_deg": 30.0}

		# Control state
		stop_until = 0.0
		prev_err = np.array([0.0, 0.0])
		wp_idx = 0
		success = False
		t = 0.0
		timeout = float(scn.get("runtime", {}).get("duration_s", 90.0))
		min_clear_geom = 10.0
		min_ttc: float = 1e9
		v_cmd = 0.0
		w_cmd = 0.0
		minclr_below_since: Optional[float] = None

		# Replan policy knobs: a bit less aggressive than before
		MINCLR_THRESH = float(scn.get("planner", {}).get("minclr_replan_m", 0.35))  # was 0.35
		MINCLR_GRACE = float(scn.get("planner", {}).get("minclr_grace_s", 1.0))     # was 0.6
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
			for (x0, y0, x1, y1) in dynamic_wet_aabbs:
				if _pt_in_aabb(xr, yr, (x0, y0, x1, y1)):
					return True
			return False

		# crude TTC helper
		last_xy: Optional[Tuple[float, float]] = None
		last_hxy: Optional[Tuple[float, float]] = None

		def _update_min_ttc(x: float, y: float) -> None:
			nonlocal min_ttc, last_xy, last_hxy
			hx, hy = None, None
			if fallen_human_id is None and human_id is not None and crossing_active:
				hx, hy, _ = p.getBasePositionAndOrientation(human_id)[0]
			if last_xy is not None:
				vx_r = (x - last_xy[0]) / max(1e-6, dt)
				vy_r = (y - last_xy[1]) / max(1e-6, dt)
			else:
				vx_r = vy_r = 0.0
			if hx is not None and hy is not None:
				if last_hxy is not None:
					vx_h = (hx - last_hxy[0]) / max(1e-6, dt)
					vy_h = (hy - last_hxy[1]) / max(1e-6, dt)
				else:
					vx_h = vy_h = 0.0
				dx, dy = (hx - x), (hy - y)
				d2 = dx*dx + dy*dy
				if d2 > 1e-6:
					L = math.sqrt(d2)
					ux, uy = dx / L, dy / L
					rel_v = (vx_h - vx_r) * ux + (vy_h - vy_r) * uy
					if rel_v < -1e-3:
						ttc = L / (-rel_v)
						min_ttc = min(min_ttc, ttc)
			last_xy = (x, y)
			if hx is not None and hy is not None:
				last_hxy = (hx, hy)

		# Sensors
		lidar_hz = float(((scn.get("sensors", {}) or {}).get("lidar", {}) or {}).get("hz", 10.0))
		lidar = _LidarSim(
			beams=360,
			hz=lidar_hz,
			max_range=8.0,
			dropout_pct_range=(tuned_lidar_dropout, tuned_lidar_dropout + 1e-9),
			noise_sigma=tuned_lidar_noise,
			jitter_ms_max=tuned_lidar_jitter
		)
		odom = _OdomSim(bias_v=tuned_bias_v, bias_w=tuned_bias_w, noise_v=0.02, noise_w=0.01)
		sensor_z = radius * 0.9
		min_clear_lidar = 10.0

		# -------- main loop --------
		best_goal_dist = float("inf")
		last_progress_at = 0.0

		while t < timeout:
			pos, orn = p.getBasePositionAndOrientation(amr)
			x, y, _ = pos
			yaw = p.getEulerFromQuaternion(orn)[2]

			target = waypoints[min(wp_idx, len(waypoints) - 1)]
			ctrl, prev_err = _pid_follow((x, y), target, prev_err, dt=dt)
			desired_yaw = math.atan2(target[1] - y, target[0] - x)
			yaw_err = (desired_yaw - yaw + math.pi) % (2 * math.pi) - math.pi
			w_cmd = max(-1.5, min(1.5, 1.2 * yaw_err))
			v_cmd = max(0.0, min(1.8, float(np.linalg.norm(ctrl))))

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

			# Human crossing + slip
			if use_human and human_id is not None and fallen_human_id is None:
				# randomize fallback trigger a bit to avoid perfect sync
				fallback_t = 3.0 + 1.0 * rng.random()  # 4.5–6.0s
				if (not crossing_active) and (t >= fallback_t):
					crossing_active = True
					crossing_started_at = t
					p.resetBasePositionAndOrientation(human_id, [cross_x, border, 1.05], p.getQuaternionFromEuler([0, 0, 0]))
				if not crossing_active and abs(x - cross_x) < trigger_distance:
					crossing_active = True
					crossing_started_at = t
					p.resetBasePositionAndOrientation(human_id, [cross_x, border, 1.05], p.getQuaternionFromEuler([0, 0, 0]))
				if crossing_active:
					progress = min(1.0, (t - crossing_started_at) / max(1e-6, crossing_duration))
					yh = (1.0 - progress) * border + progress * (Ly - border)
					p.resetBasePositionAndOrientation(human_id, [cross_x, yh, 1.05], p.getQuaternionFromEuler([0, 0, 0]))
					if _pt_in_aabb(cross_x, yh, wet_band_aabb):
						if human_wet_since is None:
							human_wet_since = t
					else:
						human_wet_since = None
					if (not slipped) and (human_wet_since is not None) and ((t - human_wet_since) >= slip_min_exposure_s):
						if rng.random() < p_slip:
							slipped = True
							p.resetBasePositionAndOrientation(human_id, [cross_x, yh, 0.25],
							                                  p.getQuaternionFromEuler([math.pi / 2.0, 0.0, 0.0]))
							fallen_human_id = human_id
							human_id = None
							rows.append([t, x, y, yaw, v_cmd, w_cmd,
							             min_clear_lidar, min_clear_geom,
							             "slip", f"{cross_x:.2f},{yh:.2f}", int(_compute_in_wet(x, y)),
							             "fallen", int(v_cmd < 0.2), min_ttc if min_ttc < 1e8 else ""])
							# replan with probability; otherwise keep current plan (hard day)
							if rng.random() < replan_on_fall_p:
								# build obstacles with fallen human inflated
								dyn_obs: List[Tuple[float, float, float, float]] = [(_inflate(a, radius)) for a in static_aabbs]
								fh_aabb = _fallen_human_aabb(fallen_human_id)
								if fh_aabb is not None:
									dyn_obs.append(_inflate(fh_aabb, radius * 1.1))
								nonlocal_grid = rasterize_occupancy((Lx, Ly), dyn_obs, res=0.25)
								grid2, res2 = nonlocal_grid
								inflate_cells2 = max(0, int(round(radius / max(1e-6, res2))))
								grid_infl2 = inflate_grid(grid2, inflate_cells2)
								ci, cj = _to_cell(x, y, res2, grid_infl2)
								gi, gj = _to_cell(goal[0], goal[1], res2, grid_infl2)
								new_cells = astar(grid_infl2, (ci, cj), (gi, gj))
								new_wps = _cells_to_waypoints(new_cells, res2)
								if new_wps:
									waypoints = new_wps
									wp_idx = 0
					if human_id is not None and yh >= (Ly - border - 1e-3):
						crossing_active = False

			# Injectors
			state = InjectorState(
				t=t, dt=dt, robot_xy=(x, y), robot_yaw=yaw,
				humans=[h for h in ([human_id] if human_id is not None else [])],
				fallen_human=fallen_human_id,
				bounds=(Lx, Ly), border=border
			)
			for inj in injectors:
				inj.on_step(state)

			# Sensors
			lidar_ignore = ignored_for_collision | {amr}
			if human_id is not None:
				lidar_ignore.add(human_id)
			if fallen_human_id is not None:
				lidar_ignore.add(fallen_human_id)
			lidar.maybe_update(t, x, y, yaw, sensor_z, lidar_ignore)
			lr = lidar.try_pop_ready(t)
			if lr is not None:
				for inj in injectors:
					mod = inj.apply_lidar(state, lr, lidar.max_range)
					if mod is not None:
						lr = mod
				min_clear_lidar = float(np.min(lr))

			# Integrate motion
			ov, ow = _OdomSim(bias_v=tuned_bias_v, bias_w=tuned_bias_w).apply(v_cmd, w_cmd)
			new_yaw = yaw + ow * dt
			new_x = max(border, min(Lx - border, x + (ov * math.cos(new_yaw)) * dt))
			new_y = max(border, min(Ly - border, y + (ov * math.sin(new_yaw)) * dt))
			p.resetBasePositionAndOrientation(amr, [new_x, new_y, radius * 0.5], p.getQuaternionFromEuler([0, 0, new_yaw]))

			_update_min_ttc(new_x, new_y)

			# Clearance (geom) vs static + dynamic human footprint
			def _min_clear_geom(px: float, py: float, ry: float) -> float:
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

			cur_geom = _min_clear_geom(new_x, new_y, new_y)
			min_clear_geom = min(min_clear_geom, cur_geom)

			# waypoint / success
			target = waypoints[min(wp_idx, len(waypoints) - 1)]
			if math.hypot(target[0] - new_x, target[1] - new_y) < 0.5:
				wp_idx += 1
				if wp_idx >= len(waypoints):
					success = True
					in_wet = int(_compute_in_wet(new_x, new_y))
					rows.append([t, new_x, new_y, new_yaw, v_cmd, w_cmd,
					             min_clear_lidar, min_clear_geom,
					             "success", "", in_wet, _human_phase_str(), int(v_cmd < 0.2),
					             min_ttc if min_ttc < 1e8 else ""])
					save, _ = should_save_frames(batch_root, "success")
					if save: _capture_frame(batch_root, run_dir_name, cam_conf, idx=0)
					break

			# progress / stall replan
			goal_dist = math.hypot(goal[0] - new_x, goal[1] - new_y)
			if goal_dist < (best_goal_dist - 0.1):
				best_goal_dist = goal_dist
				last_progress_at = t
			elif (t - last_progress_at) >= STALL_GRACE:
				# replan on stall
				dyn_obs: List[Tuple[float, float, float, float]] = [(_inflate(a, radius)) for a in static_aabbs]
				fh_aabb = _fallen_human_aabb(fallen_human_id)
				if fh_aabb is not None:
					dyn_obs.append(_inflate(fh_aabb, radius * 1.1))
				grid2, res2 = rasterize_occupancy((Lx, Ly), dyn_obs, res=0.25)
				inflate_cells2 = max(0, int(round(radius / max(1e-6, res2))))
				grid_infl2 = inflate_grid(grid2, inflate_cells2)
				ci, cj = _to_cell(new_x, new_y, res2, grid_infl2)
				gi, gj = _to_cell(goal[0], goal[1], res2, grid_infl2)
				new_cells = astar(grid_infl2, (ci, cj), (gi, gj))
				new_wps = _cells_to_waypoints(new_cells, res2)
				if new_wps:
					waypoints = new_wps
					wp_idx = 0
					rows.append([t, new_x, new_y, new_yaw, v_cmd, w_cmd,
					             min_clear_lidar, min_clear_geom,
					             "replan", "stall", int(_compute_in_wet(new_x, new_y)),
					             _human_phase_str(), int(v_cmd < 0.2),
					             min_ttc if min_ttc < 1e8 else ""])
				last_progress_at = t

			# min-clearance replan (less aggressive)
			if cur_geom < MINCLR_THRESH:
				if minclr_below_since is None:
					minclr_below_since = t
				elif (t - minclr_below_since) >= MINCLR_GRACE:
					dyn_obs: List[Tuple[float, float, float, float]] = [(_inflate(a, radius)) for a in static_aabbs]
					fh_aabb = _fallen_human_aabb(fallen_human_id)
					if fh_aabb is not None:
						dyn_obs.append(_inflate(fh_aabb, radius * 1.1))
					grid2, res2 = rasterize_occupancy((Lx, Ly), dyn_obs, res=0.25)
					inflate_cells2 = max(0, int(round(radius / max(1e-6, res2))))
					grid_infl2 = inflate_grid(grid2, inflate_cells2)
					ci, cj = _to_cell(new_x, new_y, res2, grid_infl2)
					gi, gj = _to_cell(goal[0], goal[1], res2, grid_infl2)
					new_cells = astar(grid_infl2, (ci, cj), (gi, gj))
					new_wps = _cells_to_waypoints(new_cells, res2)
					if new_wps:
						waypoints = new_wps
						wp_idx = 0
						rows.append([t, new_x, new_y, new_yaw, v_cmd, w_cmd,
						             min_clear_lidar, min_clear_geom,
						             "replan", "low_clearance", int(_compute_in_wet(new_x, new_y)),
						             _human_phase_str(), int(v_cmd < 0.2),
						             min_ttc if min_ttc < 1e8 else ""])
					minclr_below_since = None
			else:
				minclr_below_since = None

			# contacts & outcomes
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

			# injector events
			for inj in injectors:
				ev = inj.pop_event()
				if ev:
					rows.append([t, new_x, new_y, new_yaw, v_cmd, w_cmd,
					             min_clear_lidar, min_clear_geom,
					             ev[0], ev[1], int(_compute_in_wet(new_x, new_y)),
					             _human_phase_str(), int(v_cmd < 0.2),
					             min_ttc if min_ttc < 1e8 else ""])

			if human_contact:
				in_wet = int(_compute_in_wet(new_x, new_y))
				rows.append([t, new_x, new_y, new_yaw, v_cmd, w_cmd,
				             min_clear_lidar, min_clear_geom,
				             "collision_human", "1", in_wet, _human_phase_str(), int(v_cmd < 0.2),
				             min_ttc if min_ttc < 1e8 else ""])
				save, _ = should_save_frames(batch_root, "collision_human")
				if save: _capture_frame(batch_root, run_dir_name, cam_conf, idx=0)
				break
			elif nonhuman_contact_count > 0:
				in_wet = int(_compute_in_wet(new_x, new_y))
				rows.append([t, new_x, new_y, new_yaw, v_cmd, w_cmd,
				             min_clear_lidar, min_clear_geom,
				             "contact_nonhuman", str(nonhuman_contact_count), in_wet, _human_phase_str(), int(v_cmd < 0.2),
				             min_ttc if min_ttc < 1e8 else ""])

			# regular log row
			in_wet = int(_compute_in_wet(new_x, new_y))
			rows.append([t, new_x, new_y, new_yaw, v_cmd, w_cmd,
			             min_clear_lidar, min_clear_geom,
			             "", "", in_wet, _human_phase_str(), int(v_cmd < 0.2),
			             min_ttc if min_ttc < 1e8 else ""])

			p.stepSimulation()
			if realtime: time.sleep(dt)
			t += dt

		if not success and (not rows or (rows and not rows[-1][8])):
			in_wet = int(_compute_in_wet(new_x, new_y))
			rows.append([t, new_x, new_y, new_yaw, v_cmd, w_cmd,
			             min_clear_lidar, min_clear_geom,
			             "other", "timeout", in_wet, _human_phase_str(), int(v_cmd < 0.2),
			             min_ttc if min_ttc < 1e8 else ""])

		_write_csv(out_dir / "run_one.csv", header, rows)
		return {"success": success, "time": t, "steps": len(rows)}

	finally:
		try:
			p.disconnect(client_id)
		except Exception:
			pass

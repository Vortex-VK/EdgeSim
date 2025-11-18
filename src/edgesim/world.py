from __future__ import annotations
from typing import Dict, Any, Tuple, List
import math
import pybullet as p
import pybullet_data

def _mk_box(half_extents, rgba=(0.8,0.8,0.8,1.0)):
	vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=rgba)
	col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
	return vis, col


_SURFACE_COLORS = {
	"dry": (0.9, 0.9, 0.9, 0.05),
	"painted_epoxy": (0.9, 0.5, 0.2, 0.25),
	"wet": (0.2, 0.6, 1.0, 0.35),
	"oil": (0.15, 0.15, 0.15, 0.5),
	"smooth_plastic": (0.9, 0.9, 0.4, 0.3),
	"rough": (0.4, 0.3, 0.2, 0.4),
	"metal_ramp": (0.6, 0.6, 0.7, 0.6),
	"anti_slip": (0.1, 0.6, 0.2, 0.4),
	"cleaning_liquid": (0.4, 0.7, 1.0, 0.4),
}

_OBSTACLE_COLORS = {
	"fire_extinguisher": (0.8, 0.1, 0.1, 1.0),
	"bollard": (0.9, 0.8, 0.1, 1.0),
	"power_pole": (0.4, 0.4, 0.4, 1.0),
	"it_rack": (0.15, 0.15, 0.2, 1.0),
	"trash_bin": (0.2, 0.4, 0.2, 1.0),
	"standing_pallet": (0.6, 0.4, 0.2, 1.0),
	"emergency_station": (0.9, 0.1, 0.1, 1.0),
	"cart_block": (0.2, 0.2, 0.2, 1.0),
}

_VEHICLE_COLORS = {
	"forklift": (0.9, 0.7, 0.2, 1.0),
	"pallet_jack": (0.9, 0.3, 0.2, 1.0),
	"tugger_train": (0.2, 0.5, 0.9, 1.0),
}


def _color_for_surface(surface_type: str) -> Tuple[float, float, float, float]:
	return _SURFACE_COLORS.get(surface_type, (0.2, 0.6, 1.0, 0.35))


def _spawn_floor_patch(zone, mu: float, rgba) -> int:
	x0, y0, x1, y1 = zone
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


def _spawn_box_from_aabb(aabb, height: float, rgba, mass: float = 0.0) -> int:
	x0, y0, x1, y1 = aabb
	hx = max(0.01, (x1 - x0) / 2.0)
	hy = max(0.01, (y1 - y0) / 2.0)
	hz = max(0.05, height / 2.0)
	cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
	vis, col = _mk_box([hx, hy, hz], rgba=rgba)
	bid = p.createMultiBody(
		baseMass=mass,
		baseCollisionShapeIndex=col,
		baseVisualShapeIndex=vis,
		basePosition=[cx, cy, hz],
	)
	return bid


def _spawn_cylinder_obstacle(pos, radius: float, height: float, rgba) -> int:
	vis = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height, rgbaColor=rgba)
	col = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)
	bid = p.createMultiBody(
		baseMass=0,
		baseCollisionShapeIndex=col,
		baseVisualShapeIndex=vis,
		basePosition=[pos[0], pos[1], height / 2.0],
	)
	return bid


def _spawn_vehicle_body(cfg: Dict[str, Any]) -> Tuple[int, Dict[str, Any]]:
	vtype = cfg.get("type", "forklift")
	half_extents = {
		"forklift": [0.65, 0.45, 0.9],
		"pallet_jack": [0.6, 0.35, 0.35],
		"tugger_train": [1.2, 0.5, 0.6],
		"default": [0.5, 0.4, 0.4],
	}.get(vtype, [0.5, 0.4, 0.4])
	color = _VEHICLE_COLORS.get(vtype, (0.6, 0.6, 0.6, 1.0))
	vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=color)
	col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
	path = [list(pt) for pt in cfg.get("path", []) if isinstance(pt, (list, tuple))]
	spawn_xy = list(cfg.get("start", path[0] if path else [1.0, 1.0]))
	if len(path) >= 2:
		dx = path[1][0] - path[0][0]
		dy = path[1][1] - path[0][1]
		yaw = math.atan2(dy, dx)
	else:
		yaw = float(cfg.get("yaw", 0.0))
	quat = p.getQuaternionFromEuler([0.0, 0.0, yaw])
	body = p.createMultiBody(
		baseMass=0.0,
		baseCollisionShapeIndex=col,
		baseVisualShapeIndex=vis,
		basePosition=[spawn_xy[0], spawn_xy[1], half_extents[2]],
		baseOrientation=quat,
	)
	p.changeDynamics(body, -1, lateralFriction=0.9, rollingFriction=0.02)
	meta = {
		"id": cfg.get("id"),
		"type": vtype,
		"body_id": body,
		"path": path,
		"speed_mps": cfg.get("speed_mps", 1.0),
		"fork_height_m": cfg.get("fork_height_m", 0.0),
		"carrying_pallet": cfg.get("carrying_pallet", False),
		"reversing_bias": cfg.get("reversing_bias", False),
		"warning_lights": cfg.get("warning_lights", False),
		"half_extents": list(half_extents),
	}
	return body, meta

def build_world(scn: Dict[str, Any], use_gui: bool = False) -> Dict[str, Any]:
	"""Create a simple 2.5D warehouse scene.
	- Base floor (dry friction)
	- Walls as thin boxes
	- Aisles (visual only)
	- Optional 'wet patch' slab with low friction in each traction zone
	Returns IDs and helpful info.
	"""
	client = p.connect(p.GUI if use_gui else p.DIRECT)
	p.resetSimulation()
	p.setAdditionalSearchPath(pybullet_data.getDataPath())
	p.setGravity(0, 0, -9.81)

	# Ground plane (visual + collision)
	plane_id = p.loadURDF("plane.urdf")
	# Dry friction on plane
	p.changeDynamics(plane_id, -1, lateralFriction=0.9, rollingFriction=0.0, spinningFriction=0.0)

	# World bounds & walls (very thin tall boxes)
	layout = scn["layout"]
	walls = []
	for wall in layout.get("walls", []):
		# (Placeholder if you later want arbitrary walls from 'wall')
		pass

	# A couple of perimeter walls to keep the robot in-bounds
	Lx, Ly = layout["map_size_m"]
	H = 0.5
	# left
	vis,col = _mk_box([0.05, Ly/2, H], rgba=(0.2,0.2,0.2,1))
	w0 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
	                       basePosition=[0.0, Ly/2, H])
	# right
	vis,col = _mk_box([0.05, Ly/2, H], rgba=(0.2,0.2,0.2,1))
	w1 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
	                       basePosition=[Lx, Ly/2, H])
	# bottom
	vis,col = _mk_box([Lx/2, 0.05, H], rgba=(0.2,0.2,0.2,1))
	w2 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
	                       basePosition=[Lx/2, 0.0, H])
	# top
	vis,col = _mk_box([Lx/2, 0.05, H], rgba=(0.2,0.2,0.2,1))
	w3 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
	                       basePosition=[Lx/2, Ly, H])
	walls = [w0,w1,w2,w3]

	hazards = scn.get("hazards", {})
	patch_bodies: List[int] = []
	floor_meta: List[Dict[str, Any]] = []
	transition_meta: List[Dict[str, Any]] = []
	static_meta: List[Dict[str, Any]] = []
	vehicle_meta: List[Dict[str, Any]] = []

	# Layout floor surfaces (dry concrete, epoxy, etc.)
	for surface in layout.get("floor_surfaces", []):
		zone = surface.get("zone")
		if not zone or len(zone) != 4:
			continue
		mu = float(surface.get("mu", 0.85))
		s_type = surface.get("type", "dry")
		rgba = _color_for_surface(s_type)
		body_id = None
		if s_type != "dry":
			body_id = _spawn_floor_patch(zone, mu, rgba)
			patch_bodies.append(body_id)
		floor_meta.append({
			"id": surface.get("id"),
			"type": s_type,
			"zone": zone,
			"mu": mu,
			"body_id": body_id,
			"effects": {
				"brake_scale": surface.get("brake_scale", 1.0),
				"slip_boost": surface.get("slip_boost", 0.0),
				"imu_vibe_g": surface.get("imu_vibe_g", 0.0),
			},
		})

	# Traction hazards supplied via scenario (wet/oil patches)
	for patch in hazards.get("traction", []):
		zone = patch.get("zone")
		if not zone or len(zone) != 4:
			continue
		mu = float(patch.get("mu", 0.45))
		bid = _spawn_floor_patch(zone, mu, _color_for_surface("wet"))
		patch_bodies.append(bid)
		floor_meta.append({
			"id": patch.get("id"),
			"type": patch.get("type", "wet"),
			"zone": zone,
			"mu": mu,
			"body_id": bid,
			"effects": {"brake_scale": 1.5, "slip_boost": 0.6},
		})

	# Transition zones (doorways, dock plates) -> thin highlighted slabs
	for tz in layout.get("transition_zones", []):
		zone = tz.get("zone")
		if not zone or len(zone) != 4:
			continue
		mu = float(tz.get("mu", 0.8))
		rgba = (0.95, 0.95, 0.2, 0.35)
		tid = _spawn_floor_patch(zone, mu, rgba)
		patch_bodies.append(tid)
		transition_meta.append({
			"id": tz.get("id"),
			"type": tz.get("type", "doorway"),
			"zone": zone,
			"attributes": {
				"slope_deg": tz.get("slope_deg"),
				"threshold_cm": tz.get("threshold_cm"),
				"sensor_shadow": tz.get("sensor_shadow", False),
				"delay_s": tz.get("delay_s"),
			},
			"body_id": tid,
		})

	# Static geometry (racking, open storage, endcaps, blind corners)
	geometry = layout.get("geometry", {})
	for rack in geometry.get("racking", []):
		aabb = rack.get("aabb")
		if not aabb:
			continue
		height = float(rack.get("height_m", 3.0))
		bid = _spawn_box_from_aabb(aabb, height, rgba=(0.7, 0.5, 0.3, 1.0))
		walls.append(bid)
		static_meta.append({"id": rack.get("id"), "type": rack.get("type", "rack"), "body_id": bid, "aabb": aabb, "height": height})

	for storage in geometry.get("open_storage", []):
		aabb = storage.get("aabb")
		if not aabb:
			continue
		height = float(storage.get("height_m", 1.5))
		bid = _spawn_box_from_aabb(aabb, height, rgba=(0.5, 0.35, 0.2, 0.8))
		walls.append(bid)
		static_meta.append({"id": storage.get("id"), "type": storage.get("type", "open_storage"), "body_id": bid, "aabb": aabb, "height": height})

	for endcap in geometry.get("endcaps", []):
		aabb = endcap.get("aabb")
		if not aabb:
			continue
		bid = _spawn_box_from_aabb(aabb, 1.6, rgba=(0.6, 0.6, 0.2, 1.0))
		walls.append(bid)
		static_meta.append({"id": endcap.get("id"), "type": "endcap", "body_id": bid, "aabb": aabb, "height": 1.6})

	for blind in geometry.get("blind_corners", []):
		center = blind.get("center")
		if not center:
			continue
		radius = float(blind.get("radius_m", 0.8))
		aabb = [center[0] - 0.1, center[1] - radius, center[0] + 0.1, center[1] + radius]
		bid = _spawn_box_from_aabb(aabb, 2.2, rgba=(0.3, 0.3, 0.3, 0.9))
		walls.append(bid)
		static_meta.append({"id": blind.get("id"), "type": "blind_corner", "body_id": bid, "aabb": aabb, "height": 2.2})

	# Static obstacles (fire extinguishers, bollards, etc.)
	for obs in layout.get("static_obstacles", []):
		otype = obs.get("type", "obstacle")
		rgba = _OBSTACLE_COLORS.get(otype, (0.5, 0.5, 0.5, 1.0))
		body_id = None
		aabb = obs.get("aabb")
		if obs.get("shape") == "cylinder":
			pos = obs.get("pos")
			if not pos:
				continue
			radius = float(obs.get("radius", 0.1))
			height = float(obs.get("height", 1.0))
			body_id = _spawn_cylinder_obstacle(pos, radius, height, rgba)
			aabb = [pos[0] - radius, pos[1] - radius, pos[0] + radius, pos[1] + radius]
		elif aabb:
			height = float(obs.get("height", 1.0))
			body_id = _spawn_box_from_aabb(aabb, height, rgba)
		if body_id is not None:
			walls.append(body_id)
			static_meta.append({"id": obs.get("id"), "type": otype, "body_id": body_id, "aabb": aabb, "height": obs.get("height", 1.0)})

	# Parked / dynamic industrial vehicles
	for veh in hazards.get("vehicles", []):
		body_id, vmeta = _spawn_vehicle_body(veh)
		vehicle_meta.append(vmeta)

	return {
		"client": client,
		"plane_id": plane_id,
		"walls": walls,
		"patches": patch_bodies,
		"bounds": (Lx, Ly),
		"floor_zones": floor_meta,
		"transition_zones": transition_meta,
		"static_obstacles": static_meta,
		"vehicles_meta": vehicle_meta,
	}

def spawn_human(radius: float = 0.25, length: float = 1.6, rgba=(0.2, 0.8, 0.2, 1.0)):
	"""Create a kinematic human as a vertical capsule. Returns body id."""
	col = p.createCollisionShape(p.GEOM_CAPSULE, radius=radius, height=length)
	vis = p.createVisualShape(p.GEOM_CAPSULE, radius=radius, length=length, rgbaColor=rgba)
	# Center of a Z-capsule is at z = radius + length/2 when touching the ground plane at z=0
	z_center = float(radius + 0.5 * length)  # 0.25 + 0.8 = 1.05 for defaults
	bid = p.createMultiBody(
		baseMass=0.0,  # kinematic
		baseCollisionShapeIndex=col,
		baseVisualShapeIndex=vis,
		basePosition=[-10.0, -10.0, z_center],  # start off-stage but at correct height
	)
	return bid

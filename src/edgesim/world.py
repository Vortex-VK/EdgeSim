from __future__ import annotations
from typing import Dict, Any, Tuple, List
import math
import pybullet as p
import pybullet_data

def _mk_box(half_extents, rgba=(0.8,0.8,0.8,1.0)):
	vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=rgba)
	col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
	return vis, col

def _normalize_aabb(rect, issues: List[str] | None = None, label: str = "aabb") -> List[float]:
	try:
		x0, y0, x1, y1 = map(float, rect)
	except Exception:
		return []
	if x1 <= x0:
		if issues is not None:
			issues.append(f"{label} has non-positive width: {rect}")
		x0, x1 = sorted([x0, x1])
		if x1 - x0 < 1e-3:
			x1 = x0 + 0.05
	if y1 <= y0:
		if issues is not None:
			issues.append(f"{label} has non-positive height: {rect}")
		y0, y1 = sorted([y0, y1])
		if y1 - y0 < 1e-3:
			y1 = y0 + 0.05
	return [x0, y0, x1, y1]

def _aabb_center(rect: List[float] | tuple[float, float, float, float]) -> tuple[float, float]:
	x0, y0, x1, y1 = rect
	return ((x0 + x1) * 0.5, (y0 + y1) * 0.5)


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

_AISLE_COLORS = {
	"straight": (0.85, 0.85, 0.85, 0.18),
	"t": (0.82, 0.88, 0.72, 0.2),
	"l": (0.86, 0.78, 0.65, 0.2),
	"cross": (0.86, 0.86, 0.95, 0.25),
	"dead_end": (0.9, 0.8, 0.8, 0.25),
}


def _color_for_surface(surface_type: str) -> Tuple[float, float, float, float]:
	return _SURFACE_COLORS.get(surface_type, (0.2, 0.6, 1.0, 0.35))

def _pt_in_rect(pt: tuple[float, float], rect: tuple[float, float, float, float]) -> bool:
	x0, y0, x1, y1 = rect
	x, y = pt
	return (x0 <= x <= x1) and (y0 <= y <= y1)

def _rect_overlap(a: Tuple[float, float, float, float],
                  b: Tuple[float, float, float, float]) -> Tuple[float, float, float, float] | None:
	x0 = max(a[0], b[0])
	y0 = max(a[1], b[1])
	x1 = min(a[2], b[2])
	y1 = min(a[3], b[3])
	if (x1 - x0) > 1e-4 and (y1 - y0) > 1e-4:
		return (x0, y0, x1, y1)
	return None

def _intervals_without(start: float, end: float,
                       cuts: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
	intervals = [(start, end)]
	for (c0, c1) in cuts:
		c0 = max(start, min(end, c0))
		c1 = max(start, min(end, c1))
		if c1 <= c0:
			continue
		next_intervals: List[Tuple[float, float]] = []
		for (i0, i1) in intervals:
			if c1 <= i0 or c0 >= i1:
				next_intervals.append((i0, i1))
				continue
			if i0 < c0:
				next_intervals.append((i0, c0))
			if c1 < i1:
				next_intervals.append((c1, i1))
		intervals = next_intervals
	return intervals

def _clamp_zone(zone: List[float], bounds: Tuple[float, float], issues: List[str] | None = None, label: str = "zone") -> List[float]:
	normed = _normalize_aabb(zone, issues, label=label)
	if not normed:
		return []
	Lx, Ly = bounds
	x0, y0, x1, y1 = normed
	clamped = [
		max(0.0, min(Lx, x0)),
		max(0.0, min(Ly, y0)),
		max(0.0, min(Lx, x1)),
		max(0.0, min(Ly, y1)),
	]
	if issues is not None and clamped != normed:
		issues.append(f"{label} clamped to {bounds}: {normed} -> {clamped}")
	return clamped

def _spawn_walkway(zone, lane_type: str, mu: float, rgba=None) -> Tuple[int, Dict[str, Any]]:
	color = rgba if rgba is not None else _AISLE_COLORS.get(lane_type, (0.85, 0.85, 0.85, 0.2))
	bid = _spawn_floor_patch(zone, mu, color)
	meta = {
		"type": f"aisle_{lane_type}",
		"zone": zone,
		"mu": mu,
		"body_id": bid,
		"effects": {"brake_scale": 1.0, "slip_boost": 0.0, "imu_vibe_g": 0.0},
	}
	return bid, meta


def _spawn_floor_patch(zone, mu: float, rgba, thickness: float = 0.002) -> int:
	x0, y0, x1, y1 = zone
	cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
	hx, hy = max(0.01, (x1 - x0) / 2.0), max(0.01, (y1 - y0) / 2.0)
	hz = max(0.0005, float(thickness) * 0.5)
	vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[hx, hy, hz], rgbaColor=rgba)
	col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[hx, hy, hz])
	bid = p.createMultiBody(
		baseMass=0,
		baseCollisionShapeIndex=col,
		baseVisualShapeIndex=vis,
		basePosition=[cx, cy, hz],
	)
	p.changeDynamics(bid, -1, lateralFriction=float(mu))
	return bid

def _spawn_aisle_racks(zone: Tuple[float, float, float, float],
                       bounds: Tuple[float, float],
                       rack_cfg: Dict[str, Any],
                       static_meta: List[Dict[str, Any]],
                       cutouts: List[Tuple[float, float, float, float]] | None = None) -> None:
	x0, y0, x1, y1 = zone
	Lx, Ly = bounds
	dx, dy = (x1 - x0), (y1 - y0)
	orientation = "horizontal" if abs(dx) >= abs(dy) else "vertical"
	gap = float(rack_cfg.get("gap_m", 0.05))
	depth = float(rack_cfg.get("depth_m", 0.45))
	height = float(rack_cfg.get("height_m", 3.0))
	rack_type = rack_cfg.get("type", "rack")
	reflective = bool(rack_cfg.get("reflective", rack_type == "high_bay"))
	color = (0.75, 0.55, 0.35, 1.0) if not reflective else (0.7, 0.7, 0.9, 1.0)

	def _strip_segments(rect: Tuple[float, float, float, float], axis_start: float, axis_end: float,
	                    is_horizontal: bool) -> None:
		if cutouts:
			cuts: List[Tuple[float, float]] = []
			for cut in cutouts:
				overlap = _rect_overlap(rect, cut)
				if overlap is None:
					continue
				if is_horizontal:
					cuts.append((overlap[0], overlap[2]))
				else:
					cuts.append((overlap[1], overlap[3]))
			intervals = _intervals_without(axis_start, axis_end, cuts)
		else:
			intervals = [(axis_start, axis_end)]
		for seg_start, seg_end in intervals:
			if (seg_end - seg_start) <= 0.05:
				continue
			if is_horizontal:
				seg_rect = [seg_start, rect[1], seg_end, rect[3]]
			else:
				seg_rect = [rect[0], seg_start, rect[2], seg_end]
			s_rect = _clamp_zone(seg_rect, bounds)
			if s_rect[2] - s_rect[0] <= 0.01 or s_rect[3] - s_rect[1] <= 0.01:
				continue
			bid = _spawn_box_from_aabb(s_rect, height, rgba=color)
			static_meta.append({
				"id": rack_cfg.get("id"),
				"type": rack_type,
				"body_id": bid,
				"aabb": s_rect,
				"height": height,
				"reflective": reflective,
				"reflectivity": 0.7 if reflective else 0.0,
			})

	if not cutouts:
		cutouts = []

	if orientation == "horizontal":
		south = (x0, max(0.0, y0 - gap - depth), x1, max(0.0, y0 - gap))
		north = (x0, min(Ly, y1 + gap), x1, min(Ly, y1 + gap + depth))
		_strip_segments(south, south[0], south[2], True)
		_strip_segments(north, north[0], north[2], True)
	else:
		west = (max(0.0, x0 - gap - depth), y0, max(0.0, x0 - gap), y1)
		east = (min(Lx, x1 + gap), y0, min(Lx, x1 + gap + depth), y1)
		_strip_segments(west, west[1], west[3], False)
		_strip_segments(east, east[1], east[3], False)

def _expand_aisle_zone(zone: List[float], bounds: Tuple[float, float], scale: float | Tuple[float, float, float, float] = 1.3) -> List[float]:
	x0, y0, x1, y1 = zone
	Lx, Ly = bounds
	if isinstance(scale, (int, float)):
		width = x1 - x0
		height = y1 - y0
		cx = 0.5 * (x0 + x1)
		cy = 0.5 * (y0 + y1)
		if width >= height:
			half_h = 0.5 * height * scale
			y0 = max(0.0, cy - half_h)
			y1 = min(Ly, cy + half_h)
		else:
			half_w = 0.5 * width * scale
			x0 = max(0.0, cx - half_w)
			x1 = min(Lx, cx + half_w)
	else:
		pad_x0, pad_y0, pad_x1, pad_y1 = map(float, scale)
		x0 = max(0.0, x0 - pad_x0)
		y0 = max(0.0, y0 - pad_y0)
		x1 = min(Lx, x1 + pad_x1)
		y1 = min(Ly, y1 + pad_y1)
	return [x0, y0, x1, y1]


def _spawn_box_from_aabb(aabb, height: float, rgba, mass: float = 0.0) -> int:
	aabb = _normalize_aabb(aabb)
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

def _spawn_box_centered(pos: Tuple[float, float], half_extents: Tuple[float, float, float],
                        yaw: float, rgba, mass: float = 0.0) -> int:
	vis, col = _mk_box(list(half_extents), rgba=rgba)
	quat = p.getQuaternionFromEuler([0.0, 0.0, yaw])
	bid = p.createMultiBody(
		baseMass=mass,
		baseCollisionShapeIndex=col,
		baseVisualShapeIndex=vis,
		basePosition=[pos[0], pos[1], half_extents[2]],
		baseOrientation=quat,
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

def _draw_rect_outline(zone: List[float] | Tuple[float, float, float, float], color: Tuple[float, float, float] = (0.1, 0.1, 0.1), z: float = 0.05) -> None:
	x0, y0, x1, y1 = zone
	pts = [
		[x0, y0, z],
		[x1, y0, z],
		[x1, y1, z],
		[x0, y1, z],
	]
	for i in range(4):
		p.addUserDebugLine(pts[i], pts[(i + 1) % 4], color, lineWidth=2.0, lifeTime=0)

def _draw_path(points: List[List[float]], color: Tuple[float, float, float] = (0.8, 0.2, 0.2), z: float = 0.08) -> None:
	for i in range(len(points) - 1):
		p0 = [points[i][0], points[i][1], z]
		p1 = [points[i + 1][0], points[i + 1][1], z]
		p.addUserDebugLine(p0, p1, color, lineWidth=2.0, lifeTime=0)

def _validate_geometry(layout: Dict[str, Any], hazards: Dict[str, Any], bounds: Tuple[float, float]) -> List[str]:
	Lx, Ly = bounds
	issues: List[str] = []
	aisles: List[Tuple[float, float, float, float]] = []
	junctions: List[Tuple[float, float, float, float]] = []
	geom = layout.get("geometry", {}) or {}

	for idx, aisle in enumerate(layout.get("aisles", []) or []):
		rect = aisle.get("rect")
		if rect and len(rect) == 4:
			zone = _clamp_zone(list(rect), bounds, issues, label=f"aisle:{aisle.get('id', idx)}")
			if zone:
				aisles.append(tuple(zone))
	for idx, junction in enumerate(layout.get("junctions", []) or []):
		rect = junction.get("rect") or junction.get("zone")
		if rect and len(rect) == 4:
			zone = _clamp_zone(list(rect), bounds, issues, label=f"junction:{junction.get('id', idx)}")
			if zone:
				junctions.append(tuple(zone))
	walkable = aisles + junctions

	open_zones: List[Tuple[float, float, float, float]] = []

	solids: List[Tuple[Tuple[float, float, float, float], str, Any]] = []

	def _add_solid(container: Dict[str, Any], typ: str, key: str = "aabb") -> None:
		rect = container.get(key)
		if rect and len(rect) == 4:
			zone = _clamp_zone(list(rect), bounds, issues, label=f"{typ}:{container.get('id')}")
			if zone:
				solids.append((tuple(zone), typ, container.get("id")))
				cx, cy = _aabb_center(zone)
				if cx < 0.0 or cy < 0.0 or cx > Lx or cy > Ly:
					issues.append(f"{typ} '{container.get('id')}' center ({cx:.2f},{cy:.2f}) outside map {bounds}")

	for rack in geom.get("racking", []) or []:
		_add_solid(rack, "rack")
	for endcap in geom.get("endcaps", []) or []:
		_add_solid(endcap, "endcap")
	for os_entry in geom.get("open_storage", []) or []:
		_add_solid(os_entry, "open_storage")
	for obs in layout.get("static_obstacles", []) or []:
		if obs.get("aabb"):
			_add_solid(obs, obs.get("type", "static"))
		elif obs.get("shape") == "cylinder" and obs.get("pos") is not None:
			r = float(obs.get("radius", 0.2))
			px, py = obs.get("pos")
			approx = [px - r, py - r, px + r, py + r]
			_add_solid({"aabb": approx, "id": obs.get("id")}, obs.get("type", "static"))

	# prevent solid geometry in aisles (cart blocks are allowed occluders)
	for rect, typ, sid in solids:
		if typ == "cart_block":
			continue
		for walk in walkable:
			if _rect_overlap(rect, walk):
				issues.append(f"{typ} '{sid}' overlaps aisle/junction {walk}")
				break

	def _check_point(pt: Tuple[float, float], ctx: str) -> None:
		x, y = pt
		if x < 0.0 or y < 0.0 or x > Lx or y > Ly:
			issues.append(f"{ctx} point {pt} outside map {bounds}")
			return
		for rect, typ, sid in solids:
			if _pt_in_rect(pt, rect):
				issues.append(f"{ctx} point {pt} inside solid {typ} '{sid}'")
				return
		target_zones = walkable + open_zones
		if target_zones and not any(_pt_in_rect(pt, z) for z in target_zones):
			issues.append(f"{ctx} point {pt} not inside any walkable/open zone")

	for h_idx, human in enumerate(hazards.get("human", []) or []):
		path = human.get("waypoints") or human.get("path")
		if isinstance(path, list):
			for pt in path:
				if isinstance(pt, (list, tuple)) and len(pt) == 2:
					_check_point((float(pt[0]), float(pt[1])), f"human_{h_idx}")
	for v_idx, veh in enumerate(hazards.get("vehicles", []) or []):
		path = veh.get("path") or []
		if isinstance(path, list):
			for pt in path:
				if isinstance(pt, (list, tuple)) and len(pt) == 2:
					_check_point((float(pt[0]), float(pt[1])), f"vehicle_{v_idx}")

	return issues

def _draw_debug_overlays(layout: Dict[str, Any], hazards: Dict[str, Any], bounds: Tuple[float, float]) -> None:
	Lx, Ly = bounds
	geom = layout.get("geometry", {}) or {}
	for aisle in layout.get("aisles", []) or []:
		rect = aisle.get("rect")
		if rect and len(rect) == 4:
			zone = _clamp_zone(list(rect), (Lx, Ly))
			if zone:
				_draw_rect_outline(zone, color=(0.1, 0.6, 0.85))
	for junc in layout.get("junctions", []) or []:
		rect = junc.get("rect") or junc.get("zone")
		if rect and len(rect) == 4:
			zone = _clamp_zone(list(rect), (Lx, Ly))
			if zone:
				_draw_rect_outline(zone, color=(0.25, 0.75, 0.55))
	for rack in geom.get("racking", []) or []:
		rect = rack.get("aabb")
		if rect and len(rect) == 4:
			zone = _clamp_zone(list(rect), (Lx, Ly))
			if zone:
				_draw_rect_outline(zone, color=(0.45, 0.25, 0.15))
	for endcap in geom.get("endcaps", []) or []:
		rect = endcap.get("aabb")
		if rect and len(rect) == 4:
			zone = _clamp_zone(list(rect), (Lx, Ly))
			if zone:
				_draw_rect_outline(zone, color=(0.6, 0.6, 0.25))
	for obs in layout.get("static_obstacles", []) or []:
		rect = obs.get("aabb")
		if rect and len(rect) == 4:
			zone = _clamp_zone(list(rect), (Lx, Ly))
			if zone:
				_draw_rect_outline(zone, color=(0.35, 0.35, 0.35))
	for human in hazards.get("human", []) or []:
		path = human.get("waypoints") or human.get("path")
		if isinstance(path, list) and len(path) >= 2:
			pts = [[float(pt[0]), float(pt[1])] for pt in path if isinstance(pt, (list, tuple)) and len(pt) == 2]
			if len(pts) >= 2:
				_draw_path(pts, color=(0.2, 0.8, 0.2), z=0.06)
	for veh in hazards.get("vehicles", []) or []:
		path = veh.get("path") or []
		if isinstance(path, list) and len(path) >= 2:
			pts = [[float(pt[0]), float(pt[1])] for pt in path if isinstance(pt, (list, tuple)) and len(pt) == 2]
			if len(pts) >= 2:
				_draw_path(pts, color=(0.8, 0.3, 0.1), z=0.09)
	for marker, color in (("start", (0.9, 0.9, 0.1)), ("goal", (0.9, 0.3, 0.3))):
		pt = layout.get(marker)
		if isinstance(pt, (list, tuple)) and len(pt) == 2:
			zone = _clamp_zone([pt[0]-0.1, pt[1]-0.1, pt[0]+0.1, pt[1]+0.1], (Lx, Ly))
			if zone:
				_draw_rect_outline(zone, color=color, z=0.12)


def _spawn_vehicle_body(cfg: Dict[str, Any]) -> Tuple[int, Dict[str, Any]]:
	vtype = cfg.get("type", "forklift")
	base_half = {
		"forklift": [0.6, 0.45, 0.85],
		"pallet_jack": [0.5, 0.32, 0.35],
		"tugger_train": [1.2, 0.5, 0.6],
		"default": [0.5, 0.4, 0.4],
	}.get(vtype, [0.5, 0.4, 0.4])
	base_half = [base_half[0], base_half[1], float(cfg.get("body_height_m", base_half[2]))]
	color = _VEHICLE_COLORS.get(vtype, (0.6, 0.6, 0.6, 1.0))
	shape_types: List[int] = [p.GEOM_BOX]
	half_extents_list: List[List[float]] = [list(base_half)]
	frame_positions: List[List[float]] = [[0.0, 0.0, 0.0]]
	frame_orientations: List[List[float]] = [[0.0, 0.0, 0.0, 1.0]]
	color_list: List[Tuple[float, float, float, float]] = [color]
	full_half = list(base_half)
	base_height = float(cfg.get("base_height_m", base_half[2]))

	if vtype == "forklift":
		fork_len = float(cfg.get("fork_length_m", 0.9))
		fork_height = max(0.02, float(cfg.get("fork_height_m", 0.1)))
		fork_half = [fork_len / 2.0, base_half[1] * 0.2, fork_height / 2.0]
		mast_half = [0.15, base_half[1] * 0.85, max(1.5, base_half[2] + 0.6)]
		fork_offset_x = base_half[0] + fork_half[0]
		fork_offset_z = -base_half[2] + fork_height / 2.0 + float(cfg.get("fork_height_m", 0.0))
		mast_offset_z = mast_half[2] - base_half[2]
		shape_types.extend([p.GEOM_BOX, p.GEOM_BOX])
		half_extents_list.extend([fork_half, mast_half])
		frame_positions.extend([
			[fork_offset_x, 0.0, fork_offset_z],
			[base_half[0] * 0.2, 0.0, mast_offset_z],
		])
		frame_orientations.extend([[0, 0, 0, 1]] * 2)
		color_list.extend([
			(0.7, 0.7, 0.7, 1.0),
			(0.6, 0.6, 0.6, 1.0),
		])
		full_half[0] = max(full_half[0], fork_offset_x + fork_half[0])
		full_half[2] = max(full_half[2], mast_offset_z + mast_half[2])
		if cfg.get("carrying_pallet"):
			pallet_thickness = 0.08
			pallet_half = [fork_len * 0.45, base_half[1] * 0.9, pallet_thickness / 2.0]
			shape_types.append(p.GEOM_BOX)
			half_extents_list.append(pallet_half)
			frame_positions.append([fork_offset_x, 0.0, fork_offset_z + pallet_thickness])
			frame_orientations.append([0, 0, 0, 1])
			color_list.append((0.7, 0.45, 0.2, 1.0))

	vis = p.createVisualShapeArray(
		shapeTypes=shape_types,
		halfExtents=half_extents_list,
		rgbaColors=color_list,
		visualFramePositions=frame_positions,
		visualFrameOrientations=frame_orientations,
	)
	col = p.createCollisionShapeArray(
		shapeTypes=shape_types,
		halfExtents=half_extents_list,
		collisionFramePositions=frame_positions,
		collisionFrameOrientations=frame_orientations,
	)
	path = [list(pt) for pt in cfg.get("path", []) if isinstance(pt, (list, tuple))]
	spawn_xy = list(cfg.get("start", path[0] if path else [1.0, 1.0]))
	if len(path) >= 2:
		dx = path[1][0] - path[0][0]
		dy = path[1][1] - path[0][1]
		yaw = math.atan2(dy, dx)
	else:
		yaw = float(cfg.get("yaw", 0.0))
	quat = p.getQuaternionFromEuler([0.0, 0.0, yaw])
	base_z = float(cfg.get("base_z", base_height))
	body = p.createMultiBody(
		baseMass=0.0,
		baseCollisionShapeIndex=col,
		baseVisualShapeIndex=vis,
		basePosition=[spawn_xy[0], spawn_xy[1], base_z],
		baseOrientation=quat,
	)
	p.changeDynamics(body, -1, lateralFriction=0.9, rollingFriction=0.02)
	meta = {
		"id": cfg.get("id"),
		"type": vtype,
		"body_id": body,
		"base_z": base_z,
		"path": path,
		"speed_mps": cfg.get("speed_mps", 1.0),
		"fork_height_m": cfg.get("fork_height_m", 0.0),
		"carrying_pallet": cfg.get("carrying_pallet", False),
		"reversing_bias": cfg.get("reversing_bias", False),
		"warning_lights": cfg.get("warning_lights", False),
		"half_extents": full_half,
		"reflective": bool(cfg.get("reflective", False)),
	}
	return body, meta

def _build_static_geometry_from_layout(client_id: int, scn: Dict[str, Any], rand=None, issues: List[str] | None = None) -> Dict[str, Any]:
	# Deterministic; rand is a hook for future jitter
	del client_id, rand
	layout = scn.get("layout", {}) or {}
	hazards = scn.get("hazards", {}) or {}
	Lx, Ly = map(float, layout.get("map_size_m", [20.0, 20.0]))

	patch_bodies: List[int] = []
	floor_meta: List[Dict[str, Any]] = []
	transition_meta: List[Dict[str, Any]] = []
	static_meta: List[Dict[str, Any]] = []
	aisle_meta: List[Dict[str, Any]] = []
	walls_extra: List[int] = []

	# Floor surfaces from layout (spawn explicit patches for non-dry)
	for surface in layout.get("floor_surfaces", []):
		zone = surface.get("zone")
		if not zone or len(zone) != 4:
			continue
		zone = _clamp_zone(list(zone), (Lx, Ly), issues, label=f"floor:{surface.get('id')}")
		mu = float(surface.get("mu", 0.85))
		s_type = surface.get("type", "dry")
		rgba = _color_for_surface(s_type)
		body_id = None
		if s_type != "dry" or abs(mu - 0.9) > 1e-6:
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

	# Explicit traction hazards
	for patch in hazards.get("traction", []):
		zone = patch.get("zone")
		if not zone or len(zone) != 4:
			continue
		zone = _clamp_zone(list(zone), (Lx, Ly), issues, label=f"traction:{patch.get('id')}")
		mu = float(patch.get("mu", 0.45))
		s_type = patch.get("type", "wet")
		bid = _spawn_floor_patch(zone, mu, _color_for_surface(s_type))
		patch_bodies.append(bid)
		brake_scale = patch.get("brake_scale", 1.5)
		slip_boost = patch.get("slip_boost", 0.6)
		imu_vibe = patch.get("imu_vibe_g", 0.0)
		floor_meta.append({
			"id": patch.get("id"),
			"type": s_type,
			"zone": zone,
			"mu": mu,
			"body_id": bid,
			"effects": {"brake_scale": brake_scale, "slip_boost": slip_boost, "imu_vibe_g": imu_vibe},
		})

	# Transition zones (doorways, dock plates)
	for tz in layout.get("transition_zones", []):
		zone = tz.get("zone")
		if not zone or len(zone) != 4:
			continue
		zone = _clamp_zone(list(zone), (Lx, Ly), issues, label=f"transition:{tz.get('id')}")
		mu = float(tz.get("mu", 0.8))
		thickness = max(0.002, float(tz.get("threshold_cm", 0.0)) * 0.005)
		rgba = (0.95, 0.95, 0.2, 0.35)
		tid = _spawn_floor_patch(zone, mu, rgba, thickness=thickness)
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

	aisle_entries: List[Dict[str, Any]] = []
	aisles_cfg = layout.get("aisles") or []
	if isinstance(aisles_cfg, list):
		for idx, aisle in enumerate(aisles_cfg):
			rect = aisle.get("rect")
			if not rect or len(rect) != 4:
				continue
			base_zone = _clamp_zone(list(rect), (Lx, Ly), issues, label=f"aisle:{aisle.get('id', idx)}")
			if not base_zone:
				continue
			if aisle.get("pad") and len(aisle["pad"]) == 4:
				zone = _expand_aisle_zone(base_zone, (Lx, Ly), scale=tuple(aisle["pad"]))
			else:
				zone = base_zone
			aisle_entries.append({"id": aisle.get("id") or f"Aisle_{idx:02d}", "zone": zone, "cfg": aisle})

	junction_entries: List[Dict[str, Any]] = []
	junction_cfg = layout.get("junctions") or []
	if isinstance(junction_cfg, list):
		for idx, junction in enumerate(junction_cfg):
			rect = junction.get("rect") or junction.get("zone")
			if not rect or len(rect) != 4:
				continue
			if junction.get("pad") and len(junction["pad"]) == 4:
				zone = _expand_aisle_zone(_clamp_zone(list(rect), (Lx, Ly), issues, label=f"junction:{junction.get('id', idx)}"), (Lx, Ly), scale=tuple(junction["pad"]))
			else:
				zone = _clamp_zone(list(rect), (Lx, Ly), issues, label=f"junction:{junction.get('id', idx)}")
			if not zone:
				continue
			junction_entries.append({"id": junction.get("id") or f"Junction_{idx:02d}", "zone": zone, "cfg": junction})

	all_walkways: List[Dict[str, Any]] = []
	for entry in aisle_entries:
		all_walkways.append({"id": entry["id"], "zone": tuple(entry["zone"]), "kind": "aisle"})
	for entry in junction_entries:
		all_walkways.append({"id": entry["id"], "zone": tuple(entry["zone"]), "kind": "junction"})

	aisle_cutouts: Dict[str, List[Tuple[float, float, float, float]]] = {entry["id"]: [] for entry in aisle_entries}
	for i in range(len(all_walkways)):
		for j in range(i + 1, len(all_walkways)):
			if all_walkways[i]["kind"] == "junction" and all_walkways[j]["kind"] == "junction":
				continue
			overlap = _rect_overlap(all_walkways[i]["zone"], all_walkways[j]["zone"])
			if overlap is None:
				continue
			pad = 0.05
			cut = (
				max(0.0, overlap[0] - pad),
				max(0.0, overlap[1] - pad),
				min(Lx, overlap[2] + pad),
				min(Ly, overlap[3] + pad),
			)
			if all_walkways[i]["kind"] == "aisle":
				aisle_cutouts.setdefault(all_walkways[i]["id"], []).append(cut)
				if all_walkways[j]["kind"] == "aisle":
					aisle_cutouts.setdefault(all_walkways[j]["id"], []).append(cut)

	for entry in aisle_entries:
		aisle = entry["cfg"]
		zone = entry["zone"]
		lane_type = (aisle.get("type") or "straight").lower()
		mu = float(aisle.get("mu", 0.88))
		bid, meta = _spawn_walkway(zone, lane_type, mu)
		cx = 0.5 * (zone[0] + zone[2])
		cy = 0.5 * (zone[1] + zone[3])
		meta.update({
			"center": (cx, cy),
			"half_extents": (0.5 * (zone[2] - zone[0]), 0.5 * (zone[3] - zone[1])),
			"yaw": float(aisle.get("yaw", 0.0)),
		})
		meta.update({
			"id": entry["id"],
			"name": aisle.get("name"),
		})
		patch_bodies.append(bid)
		floor_meta.append(meta)
		aisle_meta.append(meta)
		if aisle.get("racking", True):
			rack_cfg = aisle.get("racking") if isinstance(aisle.get("racking"), dict) else {}
			if "height_m" not in rack_cfg and aisle.get("rack_height_m"):
				rack_cfg["height_m"] = aisle.get("rack_height_m")
			if "type" not in rack_cfg:
				rack_cfg["type"] = "high_bay" if aisle.get("high_bay") else "rack"
			_spawn_aisle_racks(zone, (Lx, Ly), rack_cfg, static_meta, cutouts=aisle_cutouts.get(entry["id"], []))

	for entry in junction_entries:
		zone = entry["zone"]
		junction = entry["cfg"]
		j_type = (junction.get("type") or "cross").lower()
		mu = float(junction.get("mu", 0.9))
		bid, meta = _spawn_walkway(zone, f"junction_{j_type}", mu, rgba=_AISLE_COLORS.get("cross"))
		cx = 0.5 * (zone[0] + zone[2])
		cy = 0.5 * (zone[1] + zone[3])
		meta.update({
			"center": (cx, cy),
			"half_extents": (0.5 * (zone[2] - zone[0]), 0.5 * (zone[3] - zone[1])),
			"yaw": float(junction.get("yaw", 0.0)),
		})
		meta.update({
			"id": entry["id"],
			"type": f"junction_{j_type}",
		})
		patch_bodies.append(bid)
		floor_meta.append(meta)
		aisle_meta.append(meta)

	geometry = layout.get("geometry", {}) or {}
	for rack in geometry.get("racking", []):
		aabb = rack.get("aabb")
		if not aabb:
			continue
		aabb = _clamp_zone(list(aabb), (Lx, Ly), issues, label=f"rack:{rack.get('id')}")
		height = float(rack.get("height_m", 3.0))
		rgba = (0.7, 0.5, 0.3, 1.0)
		reflective = bool((rack.get("type") or "").lower() == "high_bay" or rack.get("reflective", False))
		if reflective:
			rgba = (0.7, 0.7, 0.9, 1.0)
		bid = _spawn_box_from_aabb(aabb, height, rgba=rgba)
		walls_extra.append(bid)
		static_meta.append({
			"id": rack.get("id"),
			"type": rack.get("type", "rack"),
			"body_id": bid,
			"aabb": aabb,
			"height": height,
			"occlusion": True,
			"reflective": reflective,
		})

	for storage in geometry.get("open_storage", []):
		aabb = storage.get("aabb")
		if not aabb:
			continue
		aabb = _clamp_zone(list(aabb), (Lx, Ly), issues, label=f"open_storage:{storage.get('id')}")
		height = float(storage.get("height_m", 1.5))
		bid = _spawn_box_from_aabb(aabb, height, rgba=(0.5, 0.35, 0.2, 0.8))
		walls_extra.append(bid)
		static_meta.append({
			"id": storage.get("id"),
			"type": storage.get("type", "open_storage"),
			"body_id": bid,
			"aabb": aabb,
			"height": height,
			"occlusion": True,
		})

	for endcap in geometry.get("endcaps", []):
		aabb = endcap.get("aabb")
		if not aabb:
			continue
		aabb = _clamp_zone(list(aabb), (Lx, Ly), issues, label=f"endcap:{endcap.get('id')}")
		height = float(endcap.get("height_m", 1.6))
		bid = _spawn_box_from_aabb(aabb, height, rgba=(0.6, 0.6, 0.2, 1.0))
		walls_extra.append(bid)
		static_meta.append({
			"id": endcap.get("id"),
			"type": "endcap",
			"body_id": bid,
			"aabb": aabb,
			"height": height,
			"occlusion": True,
		})

	for blind in geometry.get("blind_corners", []):
		center = blind.get("center")
		if not center:
			continue
		radius = float(blind.get("radius_m", 0.8))
		aabb = [center[0] - 0.1, center[1] - radius, center[0] + 0.1, center[1] + radius]
		aabb = _clamp_zone(aabb, (Lx, Ly), issues, label=f"blind_corner:{blind.get('id')}")
		bid = _spawn_box_from_aabb(aabb, 2.2, rgba=(0.3, 0.3, 0.3, 0.9))
		walls_extra.append(bid)
		static_meta.append({
			"id": blind.get("id"),
			"type": "blind_corner",
			"body_id": bid,
			"aabb": aabb,
			"height": 2.2,
			"occlusion": True,
		})

	for wall in layout.get("walls", []):
		aabb = wall.get("aabb")
		if not aabb or len(aabb) != 4:
			continue
		aabb = _clamp_zone(list(aabb), (Lx, Ly), issues, label=f"wall:{wall.get('id')}")
		height = float(wall.get("height_m", 2.0))
		rgba = tuple(wall.get("color", (0.35, 0.35, 0.35, 1.0)))
		reflective = bool(wall.get("reflective", False))
		occlusion = wall.get("occlusion", True)
		bid = _spawn_box_from_aabb(aabb, height, rgba=rgba)
		walls_extra.append(bid)
		static_meta.append({
			"id": wall.get("id"),
			"type": wall.get("type", "wall"),
			"body_id": bid,
			"aabb": aabb,
			"height": height,
			"occlusion": occlusion,
			"reflective": reflective,
		})

	for obs in layout.get("static_obstacles", []):
		aabb_final = None
		otype = obs.get("type", "obstacle")
		rgba = _OBSTACLE_COLORS.get(otype, (0.5, 0.5, 0.5, 1.0))
		body_id = None
		aabb = obs.get("aabb")
		occlusion = obs.get("occlusion", True)
		reflective = bool(obs.get("reflective", False))
		if obs.get("shape") == "cylinder":
			pos = obs.get("pos")
			if not pos:
				continue
			radius = float(obs.get("radius", 0.1))
			height = float(obs.get("height", 1.0))
			body_id = _spawn_cylinder_obstacle(pos, radius, height, rgba)
			aabb_corr = [pos[0] - radius, pos[1] - radius, pos[0] + radius, pos[1] + radius]
			aabb_final = _clamp_zone(list(aabb_corr), (Lx, Ly), issues, label=f"{otype}:{obs.get('id')}")
		elif obs.get("pos") and obs.get("size"):
			pos = obs.get("pos")
			size = obs.get("size")
			if isinstance(pos, (list, tuple)) and len(pos) == 2 and isinstance(size, (list, tuple)) and len(size) == 3:
				half = (float(size[0]) * 0.5, float(size[1]) * 0.5, float(size[2]) * 0.5)
				yaw = float(obs.get("yaw", 0.0))
				body_id = _spawn_box_centered((float(pos[0]), float(pos[1])), half, yaw, rgba)
				# approximate AABB without yaw for spawn checks
				aabb_corr = [pos[0] - half[0], pos[1] - half[1], pos[0] + half[0], pos[1] + half[1]]
				aabb_final = _clamp_zone(list(aabb_corr), (Lx, Ly), issues, label=f"{otype}:{obs.get('id')}")
		elif aabb:
			aabb_final = _clamp_zone(list(aabb), (Lx, Ly), issues, label=f"{otype}:{obs.get('id')}")
			height = float(obs.get("height", 1.0))
			body_id = _spawn_box_from_aabb(aabb_final, height, rgba)
		if body_id is not None:
			walls_extra.append(body_id)
			static_meta.append({
				"id": obs.get("id"),
				"type": otype,
				"body_id": body_id,
				"aabb": aabb_final if aabb_final is not None else aabb,
				"height": obs.get("height", 1.0),
				"occlusion": occlusion,
				"reflective": reflective,
			})

	return {
		"patches": patch_bodies,
		"floor_meta": floor_meta,
		"transition_meta": transition_meta,
		"static_meta": static_meta,
		"aisle_meta": aisle_meta,
		"walls_extra": walls_extra,
		"bounds": (Lx, Ly),
	}

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
	layout = scn.get("layout", {}) or {}
	hazards = scn.get("hazards", {}) or {}
	map_size = layout.get("map_size_m", [20.0, 20.0])
	if not isinstance(map_size, (list, tuple)) or len(map_size) < 2:
		map_size = [20.0, 20.0]
	Lx, Ly = map(float, map_size[:2])
	bounds = (Lx, Ly)
	geom_issues = _validate_geometry(layout, hazards, bounds)

	walls: List[int] = []
	perimeter_walls: List[int] = []
	static_bodies: List[int] = []
	patch_bodies: List[int] = []
	floor_meta: List[Dict[str, Any]] = []
	transition_meta: List[Dict[str, Any]] = []
	static_meta: List[Dict[str, Any]] = []
	aisle_meta: List[Dict[str, Any]] = []

	use_perimeter_walls = layout.get("use_perimeter_walls")
	if use_perimeter_walls is None:
		# default: create perimeter unless custom walls are provided, in which case opt out unless explicitly enabled
		use_perimeter_walls = not bool(layout.get("walls"))

	if use_perimeter_walls:
		H = 2.0
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
		perimeter_walls = [w0, w1, w2, w3]
		walls.extend(perimeter_walls)
		static_bodies.extend(perimeter_walls)

	geom_result = _build_static_geometry_from_layout(client, scn, issues=geom_issues)
	patch_bodies = list(geom_result.get("patches", []))
	floor_meta = list(geom_result.get("floor_meta", []))
	transition_meta = list(geom_result.get("transition_meta", []))
	static_meta = list(geom_result.get("static_meta", []))
	aisle_meta = list(geom_result.get("aisle_meta", []))
	static_bodies.extend(geom_result.get("walls_extra", []))

	if geom_issues:
		for msg in geom_issues:
			print(f"[geometry] {msg}")

	vehicle_meta: List[Dict[str, Any]] = []
	# Parked / dynamic industrial vehicles
	for veh in hazards.get("vehicles", []):
		body_id, vmeta = _spawn_vehicle_body(veh)
		vehicle_meta.append(vmeta)

	if use_gui:
		_draw_debug_overlays(layout, hazards, bounds)

	return {
		"client": client,
		"plane_id": plane_id,
		"walls": walls,  # perimeter only
		"perimeter_walls": perimeter_walls,
		"static_bodies": static_bodies,
		"patches": patch_bodies,
		"bounds": bounds,
		"floor_zones": floor_meta,
		"transition_zones": transition_meta,
		"static_obstacles": static_meta,
		"vehicles_meta": vehicle_meta,
		"aisles_meta": aisle_meta,
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

from __future__ import annotations
from typing import Dict, Any, Tuple, List
import math
import random
import hashlib
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

def _stable_rng(seed: str) -> random.Random:
	digest = hashlib.sha256(seed.encode("utf-8")).digest()
	seed_int = int.from_bytes(digest[:8], "big")
	return random.Random(seed_int)


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

def _oriented_aabb(center: Tuple[float, float], half_extents: Tuple[float, float], yaw: float) -> List[float]:
	"""Axis-aligned bounding box for a rectangle centered at center with half_extents rotated by yaw."""
	hx, hy = half_extents
	c = abs(math.cos(yaw))
	s = abs(math.sin(yaw))
	ax = hx * c + hy * s
	ay = hx * s + hy * c
	return [center[0] - ax, center[1] - ay, center[0] + ax, center[1] + ay]

def _pose_from_centerline(start: Tuple[float, float], end: Tuple[float, float], width: float) -> Dict[str, Any]:
	dx, dy = (end[0] - start[0]), (end[1] - start[1])
	length = math.hypot(dx, dy)
	if length < 1e-6:
		length = 1e-6
	yaw = math.atan2(dy, dx)
	cx, cy = (0.5 * (start[0] + end[0]), 0.5 * (start[1] + end[1]))
	half_l = 0.5 * length
	half_w = max(0.05, float(width) * 0.5)
	return {
		"center": (cx, cy),
		"half_extents": (half_l, half_w),
		"yaw": yaw,
		"length_m": length,
		"width_m": 2.0 * half_w,
	}

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

def _spawn_walkway(zone, lane_type: str, mu: float, rgba=None, pose: Dict[str, Any] | None = None) -> Tuple[int, Dict[str, Any]]:
	color = rgba if rgba is not None else _AISLE_COLORS.get(lane_type, (0.85, 0.85, 0.85, 0.2))
	center = _aabb_center(zone)
	half_xy = (max(0.01, (zone[2] - zone[0]) * 0.5), max(0.01, (zone[3] - zone[1]) * 0.5))
	yaw = 0.0
	if pose and pose.get("half_extents"):
		pose_half = pose.get("half_extents") or ()
		if isinstance(pose_half, (list, tuple)) and len(pose_half) >= 2:
			half_xy = (max(0.01, float(pose_half[0])), max(0.01, float(pose_half[1])))
		if pose.get("center") and len(pose.get("center")) == 2:
			center = (float(pose["center"][0]), float(pose["center"][1]))
		yaw = float(pose.get("yaw", 0.0))
		bid = _spawn_floor_patch_oriented(center, half_xy, mu, color, yaw=yaw, thickness=pose.get("thickness", 0.002))
	else:
		bid = _spawn_floor_patch(zone, mu, color)
	meta = {
		"type": f"aisle_{lane_type}",
		"zone": zone,
		"mu": mu,
		"body_id": bid,
		"effects": {"brake_scale": 1.0, "slip_boost": 0.0, "imu_vibe_g": 0.0},
		"center": center,
		"half_extents": half_xy,
		"yaw": yaw,
		"width_m": min(half_xy[0], half_xy[1]) * 2.0,
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

def _spawn_floor_patch_oriented(center: Tuple[float, float], half_extents: Tuple[float, float],
                                mu: float, rgba, yaw: float = 0.0, thickness: float = 0.002) -> int:
	cx, cy = center
	hx = max(0.01, float(half_extents[0]))
	hy = max(0.01, float(half_extents[1]))
	hz = max(0.0005, float(thickness) * 0.5)
	quat = p.getQuaternionFromEuler([0.0, 0.0, float(yaw)])
	vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[hx, hy, hz], rgbaColor=rgba)
	col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[hx, hy, hz])
	bid = p.createMultiBody(
		baseMass=0,
		baseCollisionShapeIndex=col,
		baseVisualShapeIndex=vis,
		basePosition=[cx, cy, hz],
		baseOrientation=quat,
	)
	p.changeDynamics(bid, -1, lateralFriction=float(mu))
	return bid

def _spawn_box_centered_xyz(center: Tuple[float, float, float], half_extents: Tuple[float, float, float],
                            rgba, mass: float = 0.0) -> int:
	vis, col = _mk_box(list(half_extents), rgba=rgba)
	bid = p.createMultiBody(
		baseMass=mass,
		baseCollisionShapeIndex=col,
		baseVisualShapeIndex=vis,
		basePosition=[center[0], center[1], center[2]],
	)
	return bid

def _spawn_aisle_racks(zone: Tuple[float, float, float, float],
                       bounds: Tuple[float, float],
                       rack_cfg: Dict[str, Any],
                       static_meta: List[Dict[str, Any]],
                       walls_extra: List[int],
                       cutouts: List[Tuple[float, float, float, float]] | None = None) -> None:
	x0, y0, x1, y1 = zone
	Lx, Ly = bounds
	dx, dy = (x1 - x0), (y1 - y0)
	orientation = "horizontal" if abs(dx) >= abs(dy) else "vertical"
	gap = float(rack_cfg.get("gap_m", 0.05))
	depth_default = 0.55 if max(abs(dx), abs(dy)) > 4.0 else 0.45
	depth = float(rack_cfg.get("depth_m", depth_default))
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
				rack_id = rack_cfg.get("id") or "Rack"
				_spawn_detailed_rack(s_rect, height, f"{rack_id}_{len(static_meta)+1:02d}", rack_type, reflective, static_meta, walls_extra)

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

def _spawn_detailed_rack(aabb: List[float] | Tuple[float, float, float, float],
                         height: float,
                         rack_id: str,
                         rack_type: str,
                         reflective: bool,
                         static_meta: List[Dict[str, Any]],
                         walls_extra: List[int]) -> None:
	rect = _normalize_aabb(aabb)
	if not rect:
		return
	x0, y0, x1, y1 = rect
	dx, dy = (x1 - x0), (y1 - y0)
	horizontal = abs(dx) >= abs(dy)
	length = abs(dx) if horizontal else abs(dy)
	depth = abs(dy) if horizontal else abs(dx)
	if length < 0.2 or depth < 0.2:
		return
	rng = _stable_rng(f"{rack_id}:{round(x0,3)}:{round(y0,3)}:{round(x1,3)}:{round(y1,3)}:{round(height,2)}")
	pillar_w = min(0.12, max(0.06, depth * 0.35))
	frame_pad = max(0.05, pillar_w * 0.75)
	axis_start = (x0 if horizontal else y0) + frame_pad
	axis_end = (x1 if horizontal else y1) - frame_pad
	depth_start = (y0 if horizontal else x0) + frame_pad
	depth_end = (y1 if horizontal else x1) - frame_pad
	if axis_end <= axis_start or depth_end <= depth_start:
		return
	length_use = axis_end - axis_start
	n_bays = max(1, min(8, int(round(length_use / max(0.9, min(2.0, length_use))) or 1)))
	bay_len = length_use / n_bays
	depth_span = depth_end - depth_start
	beam_t = max(0.04, min(0.08, height * 0.03))
	usable_height = max(1.0, height - 0.25)
	shelf_count = max(2, min(5, int(round(usable_height / 1.1))))
	gap_h = usable_height / shelf_count
	# include floor-level shelf plus elevated shelves
	shelf_zs = [0.0]
	shelf_zs.extend([0.2 + gap_h * (i + 1) for i in range(shelf_count)])
	frame_color = (0.25, 0.35, 0.7, 1.0)
	beam_color = (0.95, 0.6, 0.25, 1.0)
	box_color = (0.75, 0.6, 0.25, 1.0)
	if reflective:
		frame_color = (0.6, 0.65, 0.9, 1.0)
		beam_color = (0.95, 0.7, 0.35, 1.0)
		box_color = (0.8, 0.65, 0.3, 1.0)

	# vertical posts at bay edges (two rows)
	axis_positions = [axis_start + bay_len * i for i in range(n_bays + 1)]
	depth_positions = [depth_start + pillar_w * 0.5, depth_end - pillar_w * 0.5]
	post_half = (pillar_w * 0.5, pillar_w * 0.5, height * 0.5)
	for ai in axis_positions:
		for dj in depth_positions:
			cx, cy = (ai, dj) if horizontal else (dj, ai)
			bid = _spawn_box_centered_xyz((cx, cy, post_half[2]), post_half, frame_color)
			walls_extra.append(bid)
			static_meta.append({
				"id": f"{rack_id}_post_{len(static_meta)+1:03d}",
				"type": rack_type,
				"body_id": bid,
				"aabb": [cx - post_half[0], cy - post_half[1], cx + post_half[0], cy + post_half[1]],
				"center": (cx, cy),
				"half_extents": (post_half[0], post_half[1]),
				"yaw": 0.0,
				"height": height,
				"occlusion": True,
				"reflective": reflective,
				"component": "post",
			})

	# shelves/beams spanning the rack
	axis_center = 0.5 * (axis_start + axis_end)
	depth_center = 0.5 * (depth_start + depth_end)
	for sz in shelf_zs:
		if sz >= height - 0.1:
			continue
		if horizontal:
			half = (0.5 * (axis_end - axis_start), depth_span * 0.45, beam_t * 0.5)
			center = (axis_center, depth_center, sz + half[2])
		else:
			half = (depth_span * 0.45, 0.5 * (axis_end - axis_start), beam_t * 0.5)
			center = (depth_center, axis_center, sz + half[2])
		bid = _spawn_box_centered_xyz(center, half, beam_color)
		walls_extra.append(bid)
		static_meta.append({
			"id": f"{rack_id}_beam_{len(static_meta)+1:03d}",
			"type": rack_type,
				"body_id": bid,
				"aabb": [center[0] - half[0], center[1] - half[1], center[0] + half[0], center[1] + half[1]],
				"center": (center[0], center[1]),
				"half_extents": (half[0], half[1]),
				"yaw": 0.0,
				"height": beam_t,
				"occlusion": True,
				"reflective": reflective,
				"component": "beam",
			})

	# random boxes per bay and shelf
	for idx, sz in enumerate(shelf_zs):
		next_z = shelf_zs[idx + 1] if idx + 1 < len(shelf_zs) else height - 0.05
		vertical_gap = max(0.25, next_z - sz - beam_t)
		if vertical_gap <= 0.2:
			continue
		for bay_idx in range(n_bays):
			axis_seg_start = axis_start + bay_len * bay_idx + pillar_w * 0.35
			axis_seg_end = axis_start + bay_len * (bay_idx + 1) - pillar_w * 0.35
			if axis_seg_end - axis_seg_start <= 0.25:
				continue
			pos = axis_seg_start
			while pos < axis_seg_end - 0.25:
				remaining = axis_seg_end - pos
				max_w = min(1.2, remaining - 0.05)
				min_w = min(0.45, max_w)
				if max_w <= 0.2:
					break
				w = rng.uniform(min_w, max_w)
				d = rng.uniform(max(0.35, depth_span * 0.45), max(0.5, depth_span * 0.85))
				h = rng.uniform(0.25, min(vertical_gap - 0.05, 0.9))
				if horizontal:
					cx, cy = pos + 0.5 * w, depth_center + rng.uniform(-0.15 * depth_span, 0.15 * depth_span)
					half = (0.5 * w, 0.5 * d, 0.5 * h)
				else:
					cx, cy = depth_center + rng.uniform(-0.15 * depth_span, 0.15 * depth_span), pos + 0.5 * w
					half = (0.5 * d, 0.5 * w, 0.5 * h)
				center_z = sz + beam_t + half[2]
				if center_z + half[2] > height:
					break
				bid = _spawn_box_centered_xyz((cx, cy, center_z), half, box_color)
				walls_extra.append(bid)
				static_meta.append({
					"id": f"{rack_id}_box_{len(static_meta)+1:03d}",
					"type": rack_type,
					"body_id": bid,
					"aabb": [cx - half[0], cy - half[1], cx + half[0], cy + half[1]],
					"center": (cx, cy),
					"half_extents": (half[0], half[1]),
					"yaw": 0.0,
					"height": h,
					"occlusion": True,
					"reflective": reflective,
					"component": "load",
				})
				pos += w + rng.uniform(0.05, 0.18)

def _spawn_detailed_rack_oriented(center: Tuple[float, float],
                                  half_extents: Tuple[float, float],
                                  height: float,
                                  rack_id: str,
                                  rack_type: str,
                                  reflective: bool,
                                  static_meta: List[Dict[str, Any]],
                                  walls_extra: List[int],
                                  yaw: float = 0.0) -> None:
	hx, hy = half_extents
	length = max(0.0, hx * 2.0)
	depth = max(0.0, hy * 2.0)
	if length < 0.2 or depth < 0.2:
		return
	rng = _stable_rng(f"{rack_id}:{round(center[0],3)}:{round(center[1],3)}:{round(length,3)}:{round(depth,3)}:{round(height,2)}:{round(yaw,3)}")
	pillar_w = min(0.12, max(0.06, depth * 0.35))
	frame_pad = max(0.05, pillar_w * 0.75)
	axis_start = -hx + frame_pad
	axis_end = hx - frame_pad
	depth_start = -hy + frame_pad
	depth_end = hy - frame_pad
	if axis_end <= axis_start or depth_end <= depth_start:
		return
	length_use = axis_end - axis_start
	n_bays = max(1, min(8, int(round(length_use / max(0.9, min(2.0, length_use))) or 1)))
	bay_len = length_use / n_bays
	depth_span = depth_end - depth_start
	beam_t = max(0.04, min(0.08, height * 0.03))
	usable_height = max(1.0, height - 0.25)
	shelf_count = max(2, min(5, int(round(usable_height / 1.1))))
	gap_h = usable_height / shelf_count
	shelf_zs = [0.0]
	shelf_zs.extend([0.2 + gap_h * (i + 1) for i in range(shelf_count)])
	frame_color = (0.25, 0.35, 0.7, 1.0)
	beam_color = (0.95, 0.6, 0.25, 1.0)
	box_color = (0.75, 0.6, 0.25, 1.0)
	if reflective:
		frame_color = (0.6, 0.65, 0.9, 1.0)
		beam_color = (0.95, 0.7, 0.35, 1.0)
		box_color = (0.8, 0.65, 0.3, 1.0)

	def _to_world(px: float, py: float) -> Tuple[float, float]:
		c = math.cos(yaw)
		s = math.sin(yaw)
		return (center[0] + px * c - py * s, center[1] + px * s + py * c)

	# vertical posts at bay edges (two rows)
	axis_positions = [axis_start + bay_len * i for i in range(n_bays + 1)]
	depth_positions = [depth_start + pillar_w * 0.5, depth_end - pillar_w * 0.5]
	post_half = (pillar_w * 0.5, pillar_w * 0.5, height * 0.5)
	for ai in axis_positions:
		for dj in depth_positions:
			wx, wy = _to_world(ai, dj)
			bid = _spawn_box_centered((wx, wy), post_half, yaw, frame_color)
			walls_extra.append(bid)
			static_meta.append({
				"id": f"{rack_id}_post_{len(static_meta)+1:03d}",
				"type": rack_type,
				"body_id": bid,
				"aabb": _oriented_aabb((wx, wy), (post_half[0], post_half[1]), yaw),
				"center": (wx, wy),
				"half_extents": (post_half[0], post_half[1]),
				"height": height,
				"occlusion": True,
				"reflective": reflective,
				"component": "post",
				"yaw": yaw,
			})

	# shelves/beams spanning the rack
	axis_center = 0.5 * (axis_start + axis_end)
	depth_center = 0.5 * (depth_start + depth_end)
	for sz in shelf_zs:
		if sz >= height - 0.1:
			continue
		half = (0.5 * (axis_end - axis_start), depth_span * 0.45, beam_t * 0.5)
		wx, wy = _to_world(axis_center, depth_center)
		bid = _spawn_box_centered((wx, wy), half, yaw, beam_color, center_z=sz + half[2])
		walls_extra.append(bid)
		static_meta.append({
			"id": f"{rack_id}_beam_{len(static_meta)+1:03d}",
			"type": rack_type,
			"body_id": bid,
			"aabb": _oriented_aabb((wx, wy), (half[0], half[1]), yaw),
			"center": (wx, wy),
			"half_extents": (half[0], half[1]),
			"height": beam_t,
			"occlusion": True,
			"reflective": reflective,
			"component": "beam",
			"yaw": yaw,
		})

	# random boxes per bay and shelf
	for idx, sz in enumerate(shelf_zs):
		next_z = shelf_zs[idx + 1] if idx + 1 < len(shelf_zs) else height - 0.05
		vertical_gap = max(0.25, next_z - sz - beam_t)
		if vertical_gap <= 0.2:
			continue
		for bay_idx in range(n_bays):
			axis_seg_start = axis_start + bay_len * bay_idx + pillar_w * 0.35
			axis_seg_end = axis_start + bay_len * (bay_idx + 1) - pillar_w * 0.35
			if axis_seg_end - axis_seg_start <= 0.25:
				continue
			pos = axis_seg_start
			while pos < axis_seg_end - 0.25:
				remaining = axis_seg_end - pos
				max_w = min(1.2, remaining - 0.05)
				min_w = min(0.45, max_w)
				if max_w <= 0.2:
					break
				w = rng.uniform(min_w, max_w)
				d = rng.uniform(max(0.35, depth_span * 0.45), max(0.5, depth_span * 0.85))
				h = rng.uniform(0.25, min(vertical_gap - 0.05, 0.9))
				cx_local = pos + 0.5 * w
				cy_local = depth_center + rng.uniform(-0.15 * depth_span, 0.15 * depth_span)
				pos += w + rng.uniform(0.05, 0.18)
				if cx_local > axis_seg_end or cy_local < depth_start or cy_local > depth_end:
					continue
				half = (0.5 * w, 0.5 * d, 0.5 * h)
				world_x, world_y = _to_world(cx_local, cy_local)
				center_z = sz + beam_t + half[2]
				if center_z + half[2] > height:
					break
				bid = _spawn_box_centered((world_x, world_y), half, yaw, box_color, center_z=center_z)
				walls_extra.append(bid)
				static_meta.append({
					"id": f"{rack_id}_box_{len(static_meta)+1:03d}",
					"type": rack_type,
					"body_id": bid,
					"aabb": _oriented_aabb((world_x, world_y), (half[0], half[1]), yaw),
					"center": (world_x, world_y),
					"half_extents": (half[0], half[1]),
					"height": h,
					"occlusion": True,
					"reflective": reflective,
					"component": "load",
					"yaw": yaw,
				})

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
                        yaw: float, rgba, mass: float = 0.0, center_z: float | None = None) -> int:
	vis, col = _mk_box(list(half_extents), rgba=rgba)
	quat = p.getQuaternionFromEuler([0.0, 0.0, yaw])
	base_z = float(half_extents[2] if center_z is None else center_z)
	bid = p.createMultiBody(
		baseMass=mass,
		baseCollisionShapeIndex=col,
		baseVisualShapeIndex=vis,
		basePosition=[pos[0], pos[1], base_z],
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

def _draw_oriented_outline(center: Tuple[float, float], half_extents: Tuple[float, float], yaw: float,
                           color: Tuple[float, float, float] = (0.1, 0.1, 0.1), z: float = 0.05) -> None:
	cx, cy = center
	hx, hy = half_extents
	c = math.cos(yaw)
	s = math.sin(yaw)
	corners = []
	for sx in (-hx, hx):
		for sy in (-hy, hy):
			x = cx + sx * c - sy * s
			y = cy + sx * s + sy * c
			corners.append([x, y, z])
	order = [0, 1, 3, 2]  # connect in rectangle order
	for i in range(4):
		p.addUserDebugLine(corners[order[i]], corners[order[(i + 1) % 4]], color, lineWidth=2.0, lifeTime=0)

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
		start = veh.get("start")
		if isinstance(start, (list, tuple)) and len(start) == 2:
			_check_point((float(start[0]), float(start[1])), f"vehicle_{v_idx}_start")

	return issues

def _draw_debug_overlays(layout: Dict[str, Any], hazards: Dict[str, Any], bounds: Tuple[float, float]) -> None:
	Lx, Ly = bounds
	geom = layout.get("geometry", {}) or {}
	for aisle in layout.get("aisles", []) or []:
		rect = aisle.get("rect")
		if rect and len(rect) == 4:
			zone = _clamp_zone(list(rect), (Lx, Ly))
			if zone:
				if aisle.get("center") and aisle.get("half_extents"):
					half = aisle.get("half_extents") or []
					if isinstance(half, (list, tuple)) and len(half) >= 2:
						_draw_oriented_outline(tuple(map(float, aisle.get("center"))), (float(half[0]), float(half[1])), float(aisle.get("yaw", 0.0)), color=(0.1, 0.6, 0.85))
				else:
					_draw_rect_outline(zone, color=(0.1, 0.6, 0.85))
	for junc in layout.get("junctions", []) or []:
		rect = junc.get("rect") or junc.get("zone")
		if rect and len(rect) == 4:
			zone = _clamp_zone(list(rect), (Lx, Ly))
			if zone:
				if junc.get("center") and junc.get("half_extents"):
					half = junc.get("half_extents") or []
					if isinstance(half, (list, tuple)) and len(half) >= 2:
						_draw_oriented_outline(tuple(map(float, junc.get("center"))), (float(half[0]), float(half[1])), float(junc.get("yaw", 0.0)), color=(0.25, 0.75, 0.55))
				else:
					_draw_rect_outline(zone, color=(0.25, 0.75, 0.55))
	for rack in geom.get("racking", []) or []:
		rect = rack.get("aabb")
		if rect and len(rect) == 4:
			zone = _clamp_zone(list(rect), (Lx, Ly))
			if zone:
				if rack.get("center") and rack.get("half_extents"):
					half = rack.get("half_extents") or []
					if isinstance(half, (list, tuple)) and len(half) >= 2:
						_draw_oriented_outline(tuple(map(float, rack.get("center"))), (float(half[0]), float(half[1])), float(rack.get("yaw", 0.0)), color=(0.45, 0.25, 0.15))
				else:
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
				if obs.get("center") and obs.get("half_extents"):
					half = obs.get("half_extents") or []
					if isinstance(half, (list, tuple)) and len(half) >= 2:
						_draw_oriented_outline(tuple(map(float, obs.get("center"))), (float(half[0]), float(half[1])), float(obs.get("yaw", 0.0)), color=(0.35, 0.35, 0.35))
				else:
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
	load_overhang = float(cfg.get("load_overhang_m", 0.0))
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
		load_overhang = float(cfg.get("load_overhang_m", 0.0))
		if load_overhang > 0.0:
			seed_bytes = str(cfg.get("id") or "forklift_load").encode("utf-8")
			rng = random.Random(int(hashlib.md5(seed_bytes).hexdigest(), 16) & 0xFFFFFFFF)
			box_half = [
				min(0.85, 0.35 + rng.random() * 0.25 + 0.3 * load_overhang),
				min(0.7, 0.32 + rng.random() * 0.18),
				min(0.6, 0.28 + rng.random() * 0.22),
			]
			offset_x = base_half[0] + box_half[0]
			offset_z = -base_half[2] + box_half[2] + max(0.05, float(cfg.get("fork_height_m", 0.0)))
			shape_types.append(p.GEOM_BOX)
			half_extents_list.append(box_half)
			frame_positions.append([offset_x, 0.0, offset_z])
			frame_orientations.append([0, 0, 0, 1])
			color_list.append((0.65, 0.45, 0.2, 1.0))
			full_half[0] = max(full_half[0], offset_x + box_half[0])
			full_half[2] = max(full_half[2], offset_z + box_half[2])

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
		"raw_coords": bool(cfg.get("raw_coords", False)),
		"speed_mps": cfg.get("speed_mps", 1.0),
		"fork_height_m": cfg.get("fork_height_m", 0.0),
		"carrying_pallet": cfg.get("carrying_pallet", False),
		"reversing_bias": cfg.get("reversing_bias", False),
		"reversing_mode": cfg.get("reversing_mode", False),
		"warning_lights": cfg.get("warning_lights", False),
		"half_extents": full_half,
		"reflective": bool(cfg.get("reflective", False)),
		"load_overhang_m": load_overhang if vtype == "forklift" else 0.0,
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
		rgba = (0.95, 0.95, 0.2, 0.55)
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
		# Optional door/dock plate geometry for fire doors / dock thresholds.
		if (tz.get("type") or "").lower() in ("fire_door", "fire door", "doorway", "loading dock", "dock"):
			door_rect = list(zone)
			span_x = abs(door_rect[2] - door_rect[0])
			span_y = abs(door_rect[3] - door_rect[1])
			# Make door a thin slice along the aisle axis to look like a panel/frame instead of a block.
			if span_x >= span_y:
				thickness_door = max(0.08, min(0.25, span_x * 0.35))
				door_rect = [door_rect[2] - thickness_door, door_rect[1], door_rect[2], door_rect[3]]
			else:
				thickness_door = max(0.08, min(0.25, span_y * 0.35))
				door_rect = [door_rect[0], door_rect[3] - thickness_door, door_rect[2], door_rect[3]]
			door_height = float(tz.get("door_height_m", 2.2))
			door_id = _spawn_box_from_aabb(door_rect, door_height, rgba=(0.55, 0.5, 0.5, 1.0))
			walls_extra.append(door_id)
			static_meta.append({
				"id": f"{tz.get('id', 'Door')}_panel",
				"type": "door_panel",
				"body_id": door_id,
				"aabb": door_rect,
				"center": ((door_rect[0] + door_rect[2]) * 0.5, (door_rect[1] + door_rect[3]) * 0.5),
				"half_extents": ((door_rect[2] - door_rect[0]) * 0.5, (door_rect[3] - door_rect[1]) * 0.5),
				"yaw": 0.0,
				"height": door_height,
				"occlusion": True,
			})
			# Dock plate slightly inset within the zone for occlusion/traction.
			dx = max(0.05, 0.12 * (door_rect[2] - door_rect[0]))
			dy = max(0.05, 0.12 * (door_rect[3] - door_rect[1]))
			plate_rect = [
				door_rect[0] + dx,
				door_rect[1] + dy,
				door_rect[2] - dx,
				door_rect[3] - dy,
			]
			plate_height = float(tz.get("plate_height_m", 0.12))
			plate_id = _spawn_box_from_aabb(plate_rect, plate_height, rgba=(0.65, 0.65, 0.7, 1.0))
			walls_extra.append(plate_id)
			static_meta.append({
				"id": f"{tz.get('id', 'Door')}_dock_plate",
				"type": "dock_plate",
				"body_id": plate_id,
				"aabb": plate_rect,
				"center": ((plate_rect[0] + plate_rect[2]) * 0.5, (plate_rect[1] + plate_rect[3]) * 0.5),
				"half_extents": ((plate_rect[2] - plate_rect[0]) * 0.5, (plate_rect[3] - plate_rect[1]) * 0.5),
				"yaw": 0.0,
				"height": plate_height,
				"occlusion": True,
				"reflective": False,
			})

	aisle_entries: List[Dict[str, Any]] = []
	aisles_cfg = layout.get("aisles") or []
	if isinstance(aisles_cfg, list):
		for idx, aisle in enumerate(aisles_cfg):
			rect = aisle.get("rect")
			if not rect or len(rect) != 4:
				continue
			pose = None
			if not aisle.get("center") and aisle.get("centerline") and len(aisle["centerline"]) == 2:
				start, end = aisle["centerline"]
				width_est = float(aisle.get("width_m", min(abs(rect[2] - rect[0]), abs(rect[3] - rect[1]))))
				pose = _pose_from_centerline(tuple(map(float, start)), tuple(map(float, end)), width_est)
				aisle["center"] = pose["center"]
				aisle["half_extents"] = pose["half_extents"]
				aisle["yaw"] = pose["yaw"]
				aisle["width_m"] = pose["width_m"]
				aisle["length_m"] = pose["length_m"]
			base_zone = _clamp_zone(list(rect), (Lx, Ly), issues, label=f"aisle:{aisle.get('id', idx)}")
			if not base_zone:
				continue
			pose_zone = base_zone
			if aisle.get("center") and aisle.get("half_extents"):
				pose_half = aisle.get("half_extents") or ()
				if isinstance(pose_half, (list, tuple)) and len(pose_half) >= 2:
					center_xy = tuple(map(float, aisle.get("center")))
					pose_zone = _oriented_aabb(center_xy, (float(pose_half[0]), float(pose_half[1])), float(aisle.get("yaw", 0.0)))
					pose_zone = _clamp_zone(pose_zone, (Lx, Ly), issues, label=f"aisle_pose:{aisle.get('id', idx)}")
			zone = pose_zone
			if aisle.get("pad") and len(aisle["pad"]) == 4:
				zone = _expand_aisle_zone(zone, (Lx, Ly), scale=tuple(aisle["pad"]))
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
		pose = None
		if aisle.get("center") and aisle.get("half_extents"):
			pose_half = aisle.get("half_extents") or ()
			if isinstance(pose_half, (list, tuple)) and len(pose_half) >= 2:
				pose = {
					"center": tuple(map(float, aisle.get("center"))),
					"half_extents": (float(pose_half[0]), float(pose_half[1])),
					"yaw": float(aisle.get("yaw", 0.0)),
				}
		bid, meta = _spawn_walkway(zone, lane_type, mu, pose=pose)
		meta.update({
			"id": entry["id"],
			"name": aisle.get("name"),
			"yaw": float(aisle.get("yaw", meta.get("yaw", 0.0))),
			"width_m": float(aisle.get("width_m", meta.get("width_m", min(zone[2] - zone[0], zone[3] - zone[1])))),
		})
		if aisle.get("length_m") is not None:
			meta["length_m"] = float(aisle["length_m"])
		patch_bodies.append(bid)
		floor_meta.append(meta)
		aisle_meta.append(meta)
		# Solid geometry should come from explicit AABBs (geometry.racking/endcaps/static_obstacles).
		allow_auto_racks = bool(aisle.get("spawn_racking_from_aisle") or aisle.get("allow_aisle_racking"))
		if allow_auto_racks and aisle.get("racking"):
			rack_cfg = aisle["racking"] if isinstance(aisle.get("racking"), dict) else {}
			if "height_m" not in rack_cfg and aisle.get("rack_height_m"):
				rack_cfg["height_m"] = aisle.get("rack_height_m")
			if "type" not in rack_cfg:
				rack_cfg["type"] = "high_bay" if aisle.get("high_bay") else "rack"
			_spawn_aisle_racks(zone, (Lx, Ly), rack_cfg, static_meta, walls_extra, cutouts=aisle_cutouts.get(entry["id"], []))

	for entry in junction_entries:
		zone = entry["zone"]
		junction = entry["cfg"]
		j_type = (junction.get("type") or "cross").lower()
		mu = float(junction.get("mu", 0.9))
		bid, meta = _spawn_walkway(zone, f"junction_{j_type}", mu, rgba=_AISLE_COLORS.get("cross"))
		meta.update({
			"yaw": float(junction.get("yaw", meta.get("yaw", 0.0))),
			"width_m": meta.get("width_m", min(zone[2] - zone[0], zone[3] - zone[1])),
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
		if not rack.get("center") and rack.get("centerline") and len(rack["centerline"]) == 2:
			start, end = rack["centerline"]
			width_est = float(rack.get("width_m", 3.0))
			pose_r = _pose_from_centerline(tuple(map(float, start)), tuple(map(float, end)), width_est)
			rack["center"] = pose_r["center"]
			rack["half_extents"] = pose_r["half_extents"]
			rack["yaw"] = pose_r["yaw"]
			rack["width_m"] = pose_r["width_m"]
			rack["length_m"] = pose_r["length_m"]
		center_pose = rack.get("center")
		half_pose = rack.get("half_extents")
		yaw_pose = float(rack.get("yaw", 0.0))
		if center_pose and half_pose and isinstance(half_pose, (list, tuple)) and len(half_pose) >= 2:
			center_xy = (float(center_pose[0]), float(center_pose[1]))
			aabb = _oriented_aabb(center_xy, (float(half_pose[0]), float(half_pose[1])), yaw_pose)
		if not aabb:
			continue
		aabb = _clamp_zone(list(aabb), (Lx, Ly), issues, label=f"rack:{rack.get('id')}")
		height = float(rack.get("height_m", 3.0))
		reflective = bool((rack.get("type") or "").lower() == "high_bay" or rack.get("reflective", False))
		if center_pose and half_pose and isinstance(half_pose, (list, tuple)) and len(half_pose) >= 2:
			center_xy = (float(center_pose[0]), float(center_pose[1]))
			half_xy = (max(0.01, float(half_pose[0])), max(0.01, float(half_pose[1])))
			_spawn_detailed_rack_oriented(center_xy, half_xy, height, rack.get("id") or "Rack", rack.get("type", "rack"), reflective, static_meta, walls_extra, yaw=yaw_pose)
		else:
			_spawn_detailed_rack(aabb, height, rack.get("id") or "Rack", rack.get("type", "rack"), reflective, static_meta, walls_extra)

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
			"center": ((aabb[0] + aabb[2]) * 0.5, (aabb[1] + aabb[3]) * 0.5),
			"half_extents": ((aabb[2] - aabb[0]) * 0.5, (aabb[3] - aabb[1]) * 0.5),
			"yaw": 0.0,
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
			"center": ((aabb[0] + aabb[2]) * 0.5, (aabb[1] + aabb[3]) * 0.5),
			"half_extents": ((aabb[2] - aabb[0]) * 0.5, (aabb[3] - aabb[1]) * 0.5),
			"yaw": 0.0,
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
			"center": ((aabb[0] + aabb[2]) * 0.5, (aabb[1] + aabb[3]) * 0.5),
			"half_extents": ((aabb[2] - aabb[0]) * 0.5, (aabb[3] - aabb[1]) * 0.5),
			"yaw": 0.0,
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
		center_pose = wall.get("center")
		half_pose = wall.get("half_extents")
		center_hint = ((aabb[0] + aabb[2]) * 0.5, (aabb[1] + aabb[3]) * 0.5)
		half_hint = ((aabb[2] - aabb[0]) * 0.5, (aabb[3] - aabb[1]) * 0.5)
		yaw_hint = float(wall.get("yaw", 0.0))
		if center_pose and half_pose and isinstance(half_pose, (list, tuple)) and len(half_pose) >= 2:
			hx = max(0.01, float(half_pose[0]))
			hy = max(0.01, float(half_pose[1]))
			cx, cy = float(center_pose[0]), float(center_pose[1])
			yaw_hint = float(wall.get("yaw", 0.0))
			center_hint = (cx, cy)
			half_hint = (hx, hy)
			bid = _spawn_box_centered((cx, cy), (hx, hy, height * 0.5), yaw_hint, rgba)
		else:
			bid = _spawn_box_from_aabb(aabb, height, rgba=rgba)
		walls_extra.append(bid)
		static_meta.append({
			"id": wall.get("id"),
			"type": wall.get("type", "wall"),
			"body_id": bid,
			"aabb": aabb,
			"center": center_hint,
			"half_extents": half_hint,
			"height": height,
			"occlusion": occlusion,
			"reflective": reflective,
			"yaw": yaw_hint,
		})

	for obs in layout.get("static_obstacles", []):
		aabb_final = None
		otype = obs.get("type", "obstacle")
		rgba = _OBSTACLE_COLORS.get(otype, (0.5, 0.5, 0.5, 1.0))
		body_id = None
		aabb = obs.get("aabb")
		occlusion = obs.get("occlusion", True)
		reflective = bool(obs.get("reflective", False))
		center_hint = None
		half_hint = None
		yaw_hint = float(obs.get("yaw", 0.0))
		if obs.get("shape") == "cylinder":
			pos = obs.get("pos")
			if not pos:
				continue
			radius = float(obs.get("radius", 0.1))
			height = float(obs.get("height", 1.0))
			body_id = _spawn_cylinder_obstacle(pos, radius, height, rgba)
			aabb_corr = [pos[0] - radius, pos[1] - radius, pos[0] + radius, pos[1] + radius]
			aabb_final = _clamp_zone(list(aabb_corr), (Lx, Ly), issues, label=f"{otype}:{obs.get('id')}")
			center_hint = (float(pos[0]), float(pos[1]))
			half_hint = (radius, radius)
			yaw_hint = 0.0
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
				center_hint = (float(pos[0]), float(pos[1]))
				half_hint = (half[0], half[1])
				yaw_hint = yaw
		elif aabb:
			aabb_final = _clamp_zone(list(aabb), (Lx, Ly), issues, label=f"{otype}:{obs.get('id')}")
			height = float(obs.get("height", 1.0))
			body_id = _spawn_box_from_aabb(aabb_final, height, rgba)
			center_hint = ((aabb_final[0] + aabb_final[2]) * 0.5, (aabb_final[1] + aabb_final[3]) * 0.5)
			half_hint = ((aabb_final[2] - aabb_final[0]) * 0.5, (aabb_final[3] - aabb_final[1]) * 0.5)
			yaw_hint = float(obs.get("yaw", 0.0))
		if body_id is not None:
			walls_extra.append(body_id)
			static_meta.append({
				"id": obs.get("id"),
				"type": otype,
				"body_id": body_id,
				"aabb": aabb_final if aabb_final is not None else aabb,
				"center": center_hint,
				"half_extents": half_hint,
				"yaw": yaw_hint,
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
	if use_gui:
		p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # hide built-in side panels
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

	if use_gui and scn.get("debug_visuals", False):
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

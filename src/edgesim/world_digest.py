from __future__ import annotations
import json, hashlib
from typing import Dict, Any, List, Tuple
from pathlib import Path

def _sha256_bytes(b: bytes) -> str:
	h = hashlib.sha256(); h.update(b); return h.hexdigest()

def _round3(x: float) -> float:
	return float(round(x, 3))

def build_world_digest(
	aabbs: List[Tuple[float,float,float,float]],
	hazards: Dict[str, Any],
	start_xy: Tuple[float,float],
	goal_xy: Tuple[float,float],
	map_size_m: Tuple[float, float] | List[float] | None = None,
	floor_zones: List[Dict[str, Any]] | None = None,
	transition_zones: List[Dict[str, Any]] | None = None,
	static_objects: List[Dict[str, Any]] | None = None,
	vehicles: List[Dict[str, Any]] | None = None,
	safety_zones: List[Dict[str, Any]] | None = None,
	environment: Dict[str, Any] | None = None,
	aisles: List[Dict[str, Any]] | None = None,
) -> Dict[str, Any]:
	def _pack_zone_list(rows: List[Dict[str, Any]] | None, key: str = "zone") -> List[Dict[str, Any]]:
		out: List[Dict[str, Any]] = []
		if not rows:
			return out
		for row in rows:
			if not isinstance(row, dict):
				continue
			zone = row.get(key)
			if not isinstance(zone, (list, tuple)) or len(zone) != 4:
				continue
			entry = {
				"id": row.get("id"),
				"type": row.get("type"),
				"zone": [_round3(float(v)) for v in zone],
				"mu": float(row.get("mu")) if row.get("mu") is not None else None,
			}
			center = row.get("center")
			half = row.get("half_extents")
			yaw = row.get("yaw")
			if isinstance(center, (list, tuple)) and len(center) == 2:
				entry["center"] = [_round3(float(center[0])), _round3(float(center[1]))]
			if isinstance(half, (list, tuple)) and len(half) >= 2:
				entry["half_extents"] = [_round3(float(half[0])), _round3(float(half[1]))]
			if yaw is not None:
				try:
					entry["yaw"] = _round3(float(yaw))
				except Exception:
					pass
			if row.get("width_m") is not None:
				entry["width_m"] = float(row.get("width_m"))
			if row.get("length_m") is not None:
				entry["length_m"] = float(row.get("length_m"))
			out.append(entry)
		return out

	def _pack_vehicle_list(rows: List[Dict[str, Any]] | None) -> List[Dict[str, Any]]:
		out: List[Dict[str, Any]] = []
		if not rows:
			return out
		for row in rows:
			if not isinstance(row, dict):
				continue
			path = row.get("path") or []
			path_fmt = [[_round3(float(pt[0])), _round3(float(pt[1]))] for pt in path if isinstance(pt, (list, tuple)) and len(pt) == 2]
			out.append({
				"id": row.get("id"),
				"type": row.get("type"),
				"path": path_fmt,
				"speed_mps": row.get("speed_mps"),
			})
		return out

	def _pack_aabb_list(rows: List[Dict[str, Any]] | None) -> List[Dict[str, Any]]:
		out: List[Dict[str, Any]] = []
		if not rows:
			return out
		for row in rows:
			if not isinstance(row, dict):
				continue
			aabb = row.get("aabb")
			if not isinstance(aabb, (list, tuple)) or len(aabb) != 4:
				continue
			entry = {
				"id": row.get("id"),
				"type": row.get("type"),
				"aabb": [_round3(float(v)) for v in aabb],
				"height": row.get("height"),
			}
			center = row.get("center")
			half = row.get("half_extents")
			yaw = row.get("yaw")
			if isinstance(center, (list, tuple)) and len(center) == 2:
				entry["center"] = [_round3(float(center[0])), _round3(float(center[1]))]
			if isinstance(half, (list, tuple)) and len(half) >= 2:
				entry["half_extents"] = [_round3(float(half[0])), _round3(float(half[1]))]
			if yaw is not None:
				try:
					entry["yaw"] = _round3(float(yaw))
				except Exception:
					pass
			out.append(entry)
		return out

	# Summaries based on provided geometry
	env_out: Dict[str, Any] = dict(environment or {})
	if map_size_m:
		try:
			env_out["map_size_m"] = [float(map_size_m[0]), float(map_size_m[1])]  # type: ignore[index]
		except Exception:
			pass
	if "map_size_m" not in env_out:
		try:
			# derive a bound from AABBs if none provided
			max_x = max((a[2] for a in aabbs), default=0.0)
			max_y = max((a[3] for a in aabbs), default=0.0)
			env_out["map_size_m"] = [_round3(max_x), _round3(max_y)]
		except Exception:
			pass

	# summarize floor surfaces for quick inspection
	floor_mu = [float(z.get("mu", 0.0)) for z in (floor_zones or []) if isinstance(z, dict) and z.get("mu") is not None]
	if floor_mu:
		env_out["floor_summary"] = {
			"mu_min": _round3(min(floor_mu)),
			"mu_max": _round3(max(floor_mu)),
			"wet_count": sum(1 for z in (floor_zones or []) if isinstance(z, dict) and str(z.get("type","")).lower() in {"wet","oil","cleaning_liquid","smooth_plastic"}),
			"zones": len(floor_mu),
		}
	rack_area = 0.0
	endcap_count = 0
	blind_count = 0
	obstacle_counts: Dict[str, int] = {}
	obstructed_area = 0.0
	for obj in (static_objects or []):
		if not isinstance(obj, dict):
			continue
		aabb = obj.get("aabb")
		if isinstance(aabb, (list, tuple)) and len(aabb) == 4:
			area = max(0.0, float(aabb[2]) - float(aabb[0])) * max(0.0, float(aabb[3]) - float(aabb[1]))
			obstructed_area += area
			if obj.get("type") == "rack":
				rack_area += area
		otype = obj.get("type")
		if otype:
			obstacle_counts[otype] = obstacle_counts.get(otype, 0) + 1
			if otype == "endcap":
				endcap_count += 1
			if otype == "blind_corner":
				blind_count += 1

	aisle_count = 0
	junction_count = 0
	for a in (aisles or []):
		if not isinstance(a, dict):
			continue
		atype = (a.get("type") or "")
		if atype.startswith("aisle"):
			aisle_count += 1
		if "junction" in atype:
			junction_count += 1

	env_out["layout_summary"] = {
		"aisle_count": aisle_count,
		"junction_count": junction_count,
		"rack_area_m2": _round3(rack_area),
		"obstructed_area_m2": _round3(obstructed_area),
		"endcaps": endcap_count,
		"blind_corners": blind_count,
		"static_obstacles": obstacle_counts,
	}

	return {
		"version": 1,
		"start": [_round3(start_xy[0]), _round3(start_xy[1])],
		"goal":  [_round3(goal_xy[0]),  _round3(goal_xy[1])],
		"aabbs": [[_round3(x0), _round3(y0), _round3(x1), _round3(y1)] for (x0,y0,x1,y1) in sorted(aabbs)],
		"hazards": hazards or {},
		"floor_zones": _pack_zone_list(floor_zones),
		"transition_zones": _pack_zone_list(transition_zones),
		"static_obstacles": _pack_aabb_list(static_objects),
		"vehicles": _pack_vehicle_list(vehicles),
		"safety_zones": _pack_zone_list(safety_zones),
		"aisles": _pack_zone_list(aisles),
		"environment": env_out,
	}

def hash_world_digest(d: Dict[str, Any]) -> str:
	js = json.dumps(d, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
	return _sha256_bytes(js)

def write_world_digest(run_dir: Path, digest: Dict[str, Any]) -> str:
	world_path = run_dir / "world.json"
	world_path.write_text(json.dumps(digest, indent=2), encoding="utf-8")
	h = hash_world_digest(digest)
	# Stamp manifest
	man_path = run_dir / "manifest.json"
	try:
		man = json.loads(man_path.read_text(encoding="utf-8")) if man_path.exists() else {}
	except Exception:
		man = {}
	man["world_sha256"] = h
	man_path.write_text(json.dumps(man, indent=2), encoding="utf-8")
	return h

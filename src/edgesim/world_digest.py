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
	floor_zones: List[Dict[str, Any]] | None = None,
	transition_zones: List[Dict[str, Any]] | None = None,
	static_objects: List[Dict[str, Any]] | None = None,
	vehicles: List[Dict[str, Any]] | None = None,
	safety_zones: List[Dict[str, Any]] | None = None,
	environment: Dict[str, Any] | None = None,
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
			out.append({
				"id": row.get("id"),
				"type": row.get("type"),
				"zone": [_round3(float(v)) for v in zone],
				"mu": float(row.get("mu")) if row.get("mu") is not None else None,
			})
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
			out.append({
				"id": row.get("id"),
				"type": row.get("type"),
				"aabb": [_round3(float(v)) for v in aabb],
				"height": row.get("height"),
			})
		return out

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
		"environment": environment or {},
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

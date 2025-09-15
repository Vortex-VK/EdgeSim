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
) -> Dict[str, Any]:
	return {
		"version": 1,
		"start": [_round3(start_xy[0]), _round3(start_xy[1])],
		"goal":  [_round3(goal_xy[0]),  _round3(goal_xy[1])],
		"aabbs": [[_round3(x0), _round3(y0), _round3(x1), _round3(y1)] for (x0,y0,x1,y1) in sorted(aabbs)],
		"hazards": hazards or {},
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

# src/edgesim/schema_validate.py
from __future__ import annotations
from typing import Any, Dict, List, Tuple

_Number = (int, float)

def _is_rect(v: Any) -> bool:
	if not isinstance(v, (list, tuple)) or len(v) != 4:
		return False
	return all(isinstance(x, (int, float)) for x in v)

def _err(errors: List[str], msg: str) -> None:
	errors.append(msg)

def validate_scenario(scn: Dict[str, Any]) -> Tuple[bool, List[str], Dict[str, Any]]:
	errors: List[str] = []
	summary: Dict[str, Any] = {}

	# layout
	layout = scn.get("layout", {})
	ms = layout.get("map_size_m", [20.0, 20.0])
	if (not isinstance(ms, (list, tuple))) or len(ms) != 2:
		_err(errors, "layout.map_size_m must be [Lx, Ly].")
	else:
		Lx, Ly = ms
		if not (isinstance(Lx, _Number) and isinstance(Ly, _Number) and Lx > 0 and Ly > 0):
			_err(errors, "layout.map_size_m values must be positive numbers.")

	start = layout.get("start", [1.0, 1.0])
	goal  = layout.get("goal", [19.0, 19.0])
	if (not isinstance(start, (list, tuple))) or len(start) != 2:
		_err(errors, "layout.start must be [x, y].")
	if (not isinstance(goal, (list, tuple))) or len(goal) != 2:
		_err(errors, "layout.goal must be [x, y].")

	# agents
	agents = scn.get("agents", [])
	if not isinstance(agents, list) or not agents:
		_err(errors, "agents must be a non-empty list.")
	else:
		for i, a in enumerate(agents):
			r = a.get("radius_m", 0.4)
			if not (isinstance(r, _Number) and 0.05 <= r <= 2.0):
				_err(errors, f"agents[{i}].radius_m looks out of range (0.05..2.0).")

	# runtime
	rt = scn.get("runtime", {})
	dur = rt.get("duration_s", 120)
	dt  = rt.get("dt", 0.1)
	if not (isinstance(dur, _Number) and 1 <= dur <= 3600):
		_err(errors, "runtime.duration_s must be 1..3600.")
	if not (isinstance(dt, _Number) and 1e-3 <= dt <= 1.0):
		_err(errors, "runtime.dt must be 0.001..1.0.")

	# hazards
	hz = scn.get("hazards", {})
	traction = hz.get("traction", [])
	if not isinstance(traction, list):
		_err(errors, "hazards.traction must be a list.")
	else:
		for i, patch in enumerate(traction):
			zone = patch.get("zone")
			mu = patch.get("mu", 0.45)
			if not _is_rect(zone):
				_err(errors, f"hazards.traction[{i}].zone must be [x0,y0,x1,y1].")
			if not (isinstance(mu, _Number) and 0.05 <= mu <= 1.0):
				_err(errors, f"hazards.traction[{i}].mu must be 0.05..1.0.")
	human = hz.get("human", [])
	if not isinstance(human, list):
		_err(errors, "hazards.human must be a list.")
	else:
		for i, h in enumerate(human):
			rpm = h.get("rate_per_min", None)
			if rpm is not None and not (isinstance(rpm, _Number) and rpm >= 0):
				_err(errors, f"hazards.human[{i}].rate_per_min must be ≥ 0.")
			sp = h.get("speed_mps", [0.8, 1.4])
			if (not isinstance(sp, (list, tuple))) or len(sp) != 2 or not all(isinstance(x, _Number) for x in sp):
				_err(errors, f"hazards.human[{i}].speed_mps must be [min,max].")

	# taxonomy (optional but helpful)
	tax = scn.get("taxonomy", {})
	if not isinstance(tax, dict):
		_err(errors, "taxonomy must be a dict if present.")

	# Build a small human-readable summary for the report
	summary = {
		"map": {"size_m": list(ms), "start": list(start), "goal": list(goal)},
		"hazards": {
			"traction_patches": [
				{"zone": list(p["zone"]), "mu": float(p.get("mu", 0.45))}
				for p in traction if isinstance(p, dict) and "zone" in p
			],
			"human": [
				{
					"path": h.get("path", "line"),
					"rate_per_min": h.get("rate_per_min", "default"),
					"speed_mps": list(h.get("speed_mps", [0.8, 1.4])),
				}
				for h in human if isinstance(h, dict)
			],
		},
		"runtime": {"duration_s": float(dur), "dt": float(dt)},
		"flags": {"taxonomy": tax},
	}

	ok = len(errors) == 0
	return ok, errors, summary

from __future__ import annotations

import csv
import hashlib
import json
import math
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import yaml

# ---------------------------------------------------------------------------
# HTML template (Tailwind/shadcn-inspired styling to match the frontend vibe)
# ---------------------------------------------------------------------------

HTML_TEMPLATE = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <title>EdgeSim Report — {title}</title>
  <style>
    :root {{
      --bg: #f8fafc;
      --card: #ffffff;
      --muted: #6b7280;
      --border: #e5e7eb;
      --accent: #0f172a;
      --accent-2: #1d4ed8;
      --accent-3: #0ea5e9;
      --good: #16a34a;
      --warn: #f59e0b;
      --bad: #dc2626;
      --rack: #a16207;
      --aisle: #e2e8f0;
      --traction: #bae6fd;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      padding: 0;
      background: radial-gradient(circle at 10% 20%, #e0f2fe 0%, #f8fafc 30%, #eef2ff 60%, #f8fafc 100%);
      font-family: "Inter", system-ui, -apple-system, "Segoe UI", sans-serif;
      color: #0f172a;
    }}
    .page {{
      max-width: 1180px;
      margin: 24px auto;
      padding: 0 18px 32px;
    }}
    h1 {{ font-size: 28px; margin: 0 0 6px 0; letter-spacing: -0.02em; }}
    h2 {{ font-size: 20px; margin: 0 0 12px 0; letter-spacing: -0.01em; }}
    h3 {{ font-size: 15px; margin: 0 0 6px 0; color: var(--muted); }}
    .muted {{ color: var(--muted); font-size: 13px; }}
    .card {{
      background: var(--card);
      border: 1px solid var(--border);
      border-radius: 16px;
      padding: 16px 18px;
      box-shadow: 0 18px 45px rgba(15, 23, 42, 0.06);
    }}
    .cards-2 {{ display: grid; gap: 14px; grid-template-columns: repeat(auto-fit, minmax(360px, 1fr)); margin-top: 14px; }}
    .cards-3 {{ display: grid; gap: 14px; grid-template-columns: repeat(auto-fit, minmax(360px, 1fr)); margin-top: 14px; }}
    .cards-1 {{ display: grid; gap: 14px; grid-template-columns: 1fr; margin-top: 14px; }}
    .chip {{
      display: inline-flex;
      align-items: center;
      gap: 6px;
      padding: 4px 10px;
      border-radius: 999px;
      background: #eef2ff;
      color: #312e81;
      font-size: 12px;
      border: 1px solid #e0e7ff;
    }}
    .metric-grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 12px; margin-top: 8px; }}
    .metric-grid.tight {{ grid-template-columns: repeat(auto-fit, minmax(100px, 1fr)); }}
    .metric {{
      padding: 10px 12px;
      border-radius: 12px;
      background: #f8fafc;
      border: 1px solid var(--border);
    }}
    .metric .label {{ font-size: 12px; color: var(--muted); margin-bottom: 4px; display: block; }}
    .metric .value {{ font-size: 18px; font-weight: 700; }}
    .pill {{ padding: 2px 8px; border-radius: 10px; font-size: 12px; display: inline-block; }}
    .pill.good {{ background: #dcfce7; color: #166534; }}
    .pill.bad {{ background: #fee2e2; color: #991b1b; }}
    .pill.warn {{ background: #fef3c7; color: #92400e; }}
    .table {{ width: 100%; border-collapse: collapse; margin-top: 4px; }}
    .table th, .table td {{ padding: 8px 6px; border-bottom: 1px solid var(--border); text-align: left; font-size: 13px; }}
    .table th {{ font-size: 12px; color: var(--muted); font-weight: 600; letter-spacing: 0.01em; }}
    .tag {{ display: inline-block; background: #f1f5f9; border: 1px solid #e2e8f0; border-radius: 12px; padding: 3px 9px; margin: 2px 4px 2px 0; font-size: 12px; }}
    .gallery {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(220px, 1fr)); gap: 10px; }}
    .gallery figure {{ margin: 0; }}
    .gallery img {{ width: 100%; border-radius: 12px; border: 1px solid var(--border); }}
    .legend {{ display: flex; gap: 10px; flex-wrap: wrap; font-size: 12px; color: var(--muted); margin-top: 6px; }}
    .legend span {{ display: inline-flex; align-items: center; gap: 6px; }}
    .dot {{ width: 10px; height: 10px; border-radius: 50%; display: inline-block; }}
    .map-wrapper {{ width: 100%; height: 420px; border: 1px solid var(--border); border-radius: 14px; overflow: hidden; background: #f8fafc; }}
    svg {{ font-family: "Inter", system-ui, sans-serif; }}
    .grid-2 {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(380px, 1fr)); gap: 14px; }}
    .section {{ margin-top: 16px; }}
    .section > h2 {{ margin-bottom: 8px; }}
    footer {{ margin-top: 18px; color: var(--muted); font-size: 12px; }}
    .badge {{ padding: 3px 8px; border-radius: 999px; background: #e0f2fe; color: #075985; font-size: 12px; }}
    .code {{ font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; background: #f1f5f9; padding: 2px 6px; border-radius: 8px; font-size: 12px; }}
    .prewrap {{ white-space: pre-wrap; word-break: break-word; }}
    .prompt-block {{ background:#f8fafc; border:1px solid var(--border); border-radius:12px; padding:10px 12px; margin-top:8px; }}
    .note {{ font-size:12px; color: var(--muted); margin-top:6px; }}
  </style>
</head>
<body>
  <div class="page">
    <div class="section">
      <h1>EdgeSim Report</h1>
      <div class="muted">Scenario: <span class="code">{scenario_name}</span> • Runs: <b>{runs}</b> • Generated: {generated_at}</div>
      <div class="muted">Seeds: <span class="code">{seeds_hint}</span> • Manifest digests: CSV <span class="code">{per_run_digest_short}</span> • world.json <span class="code">{world_sha_short}</span></div>
    </div>

    <div class="cards-2 section">
      <div class="card">
        <h2>Scenario & Randomization</h2>
        <div class="prompt-block prewrap">{prompt}</div>
        <div class="metric-grid" style="margin-top:12px;">
          <div class="metric"><span class="label">Site / Profile</span><div class="value">{site_profile}</div></div>
          <div class="metric"><span class="label">LiDAR noise / dropout</span><div class="value">{lidar_noise}</div></div>
          <div class="metric"><span class="label">Traction μ (dry / wet)</span><div class="value">{traction_hint}</div></div>
          <div class="metric"><span class="label">Runtime / dt</span><div class="value">{runtime_hint}</div></div>
          <div class="metric"><span class="label">Seeds</span><div class="value">{seeds_hint}</div></div>
        </div>
        <div style="margin-top:10px;">{taxonomy_tags}</div>
      </div>
      <div class="card">
        <h2>Run Overview</h2>
        <div class="metric-grid">
          <div class="metric"><span class="label">Success</span><div class="value">{success}</div></div>
          <div class="metric"><span class="label">Collisions</span><div class="value">{collisions}</div></div>
          <div class="metric"><span class="label">Other / Timeout</span><div class="value">{other}</div></div>
          <div class="metric"><span class="label">Worst clearance</span><div class="value">{worst_clearance}</div></div>
          <div class="metric"><span class="label">Min TTC</span><div class="value">{min_ttc}</div></div>
        </div>
        <div class="prompt-block prewrap" style="margin-top:12px;"><b>Representative run:</b> {rep_run}</div>
      </div>
    </div>

    <div class="cards-2 section">
      <div class="card">
        <h2>Hazards & Layout</h2>
        <div class="metric-grid">
          <div class="metric"><span class="label">Aisles</span><div class="value">{aisle_count}</div></div>
          <div class="metric"><span class="label">Traction patches</span><div class="value">{patch_count}</div></div>
          <div class="metric"><span class="label">Humans</span><div class="value">{human_count}</div></div>
          <div class="metric"><span class="label">Vehicles</span><div class="value">{vehicle_count}</div></div>
          <div class="metric"><span class="label">Static obstacles</span><div class="value">{static_count}</div></div>
        </div>
        <div class="muted" style="margin-top:8px;">Traction zones, aisles, racks, start/goal are drawn on the map below.</div>
      </div>
      <div class="card">
        <h2>Repro & digests</h2>
        <div class="metric-grid">
          <div class="metric"><span class="label">CSV digest</span><div class="value" style="font-size:13px;">{per_run_digest_short}</div></div>
          <div class="metric"><span class="label">scenario.yaml</span><div class="value" style="font-size:13px;">{scenario_sha_short}</div></div>
          <div class="metric"><span class="label">seeds.json</span><div class="value" style="font-size:13px;">{seeds_sha_short}</div></div>
          <div class="metric"><span class="label">world.json</span><div class="value" style="font-size:13px;">{world_sha_short}</div></div>
        </div>
        <div class="muted" style="margin-top:8px;">Run <span class="code">edgesim verify --run {verify_path}</span> to re-check digests.</div>
      </div>
    </div>

    <div class="cards-1 section">
      <div class="card">
        <h2>Top-down Map & Trajectory</h2>
        <div class="map-wrapper" style="height:480px;">{map_svg}</div>
        <div class="legend">
          <span><span class="dot" style="background:{color_robot};"></span>Robot path</span>
          <span><span class="dot" style="background:{color_event_collision};"></span>Collisions</span>
          <span><span class="dot" style="background:{color_event_slip};"></span>Slips</span>
          <span><span class="dot" style="background:{color_event_near};"></span>Near-miss / occluded</span>
          <span><span class="dot" style="background:{color_traction};"></span>Traction patches</span>
          <span><span class="dot" style="background:{color_rack};"></span>Racks / obstacles</span>
        </div>
      </div>
    </div>

    <div class="cards-2 section">
      <div class="card">
        <h2>Clearance over time</h2>
        <div>{clearance_chart}</div>
      </div>
      <div class="card">
        <h2>Time-to-collision (TTC)</h2>
        <div>{ttc_chart}</div>
      </div>
    </div>

    <div class="cards-1 section">
      <div class="card">
        <h2>Edge-case events</h2>
        {events_table}
      </div>
    </div>

    <div class="cards-2 section">
      <div class="card">
        <h2>Actors</h2>
        {actors_block}
      </div>
      <div class="card">
        <h2>Coverage & Outcomes</h2>
        {coverage_block}
      </div>
    </div>

    <footer>EdgeSim • offline HTML report • v1 • Digests: CSV {per_run_digest_short}, scenario {scenario_sha_short}, seeds {seeds_sha_short}</footer>
  </div>
</body>
</html>
"""


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    h.update(path.read_bytes())
    return h.hexdigest()


def _digest_csvs(per_run_dir: Path) -> str:
    h = hashlib.sha256()
    for p in sorted(per_run_dir.rglob("*.csv")):
        h.update(p.read_bytes())
    return h.hexdigest()


def _safe_load_json(path: Path) -> dict:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def _safe_load_yaml(path: Path) -> dict:
    try:
        return yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except Exception:
        return {}


def _parse_float(val: str | float | int | None, default: float = math.nan) -> float:
    try:
        return float(val)
    except Exception:
        return default


def _find_run_dirs(batch_dir: Path) -> List[Path]:
    """Return list of run directories (handles batch/per_run and single-run folders)."""
    per_run = batch_dir / "per_run"
    if per_run.exists():
        runs = sorted([p for p in per_run.glob("run_*") if p.is_dir()])
        if runs:
            return runs
    # fallback: single run folder
    if (batch_dir / "run_one.csv").exists():
        return [batch_dir]
    return []


def _event_color_key(typ: str) -> str:
    t = (typ or "").lower()
    if "collision" in t:
        return "ev_collision"
    if "slip" in t:
        return "ev_floor_slip"
    if "occluded" in t:
        return "ev_near_miss"
    if "near" in t or "hard_brake" in t:
        return "ev_near_miss"
    if "success" in t:
        return "ev_success"
    if "timeout" in t:
        return "ev_timeout"
    return "event_other"


# ---------------------------------------------------------------------------
# Run parsing
# ---------------------------------------------------------------------------

Event = Dict[str, object]


def _parse_run(run_dir: Path) -> dict:
    csv_path = run_dir / "run_one.csv"
    events: List[Event] = []
    path: List[Tuple[float, float]] = []
    clearance_series: List[Tuple[float, float]] = []
    ttc_series: List[Tuple[float, float]] = []
    outcome = "unknown"
    min_clearance = math.inf
    min_ttc = math.inf
    t_end = 0.0

    if not csv_path.exists():
        return {
            "run_name": run_dir.name,
            "events": events,
            "path": path,
            "clearance": clearance_series,
            "ttc": ttc_series,
            "outcome": outcome,
            "min_clearance": math.nan,
            "min_ttc": math.nan,
            "t_end": 0.0,
        }

    with csv_path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t = _parse_float(row.get("t"), default=t_end)
            t_end = t
            x = _parse_float(row.get("x"))
            y = _parse_float(row.get("y"))
            if not math.isnan(x) and not math.isnan(y):
                path.append((x, y))

            mc_geom = _parse_float(row.get("min_clearance_geom"))
            mc_lidar = _parse_float(row.get("min_clearance_lidar"))
            mc = mc_geom if not math.isnan(mc_geom) else mc_lidar
            if math.isnan(mc):
                mc = _parse_float(row.get("min_clearance"))
            if not math.isnan(mc):
                clearance_series.append((t, mc))
                if mc < min_clearance:
                    min_clearance = mc

            ttc_val = _parse_float(row.get("min_ttc"))
            if not math.isnan(ttc_val):
                ttc_series.append((t, ttc_val))
                if ttc_val < min_ttc:
                    min_ttc = ttc_val

            ev = (row.get("event") or "").strip()
            detail = (row.get("event_detail") or "").strip()
            near_stop = row.get("near_stop", "0")
            hard_brake = row.get("hard_brake", "0")
            near_miss = row.get("near_miss", "0")
            occluded = row.get("occluded_hazard", "0")

            def _add_event(label: str, kind: str) -> None:
                events.append({
                    "t": t,
                    "x": x,
                    "y": y,
                    "type": kind,
                    "label": label,
                    "detail": detail,
                    "min_clearance": mc,
                    "min_ttc": ttc_val,
                })

            if ev:
                outcome = ev
                _add_event(ev, ev)
            if near_stop and near_stop != "0":
                _add_event("near_stop", "near_miss")
            if hard_brake and hard_brake != "0":
                _add_event("hard_brake", "near_miss")
            if near_miss and near_miss != "0":
                _add_event("near_miss", "near_miss")
            if occluded and occluded != "0":
                _add_event("occluded_hazard", "occluded")

    if min_clearance == math.inf:
        min_clearance = math.nan
    if min_ttc == math.inf:
        min_ttc = math.nan

    return {
        "run_name": run_dir.name,
        "events": events,
        "path": path,
        "clearance": clearance_series,
        "ttc": ttc_series,
        "outcome": outcome,
        "min_clearance": min_clearance,
        "min_ttc": min_ttc,
        "t_end": t_end,
    }


def _parse_actors(run_dir: Path) -> Dict[str, dict]:
    actors_path = run_dir / "actors.csv"
    if not actors_path.exists():
        return {}
    actors: Dict[str, dict] = {}
    with actors_path.open(newline="", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            aid = row.get("actor_id") or "unknown"
            atype = row.get("type") or "actor"
            t = _parse_float(row.get("t"))
            x = _parse_float(row.get("x"))
            y = _parse_float(row.get("y"))
            phase = row.get("phase") or ""
            actors.setdefault(aid, {"id": aid, "type": atype, "path": [], "phases": set()})
            if not math.isnan(t) and not math.isnan(x) and not math.isnan(y):
                actors[aid]["path"].append((t, x, y))
            if phase:
                actors[aid]["phases"].add(phase)
    # finalize phases to list
    for a in actors.values():
        a["phases"] = sorted(a["phases"])
    return actors


# ---------------------------------------------------------------------------
# SVG render helpers
# ---------------------------------------------------------------------------

def _scale_builder(world_size: Tuple[float, float], width: float, height: float, margin: float = 24.0):
    Lx, Ly = max(world_size[0], 1e-3), max(world_size[1], 1e-3)
    def sx(x: float) -> float:
        return margin + (x / Lx) * (width - 2 * margin)
    def sy(y: float) -> float:
        # invert y to keep top-down orientation consistent
        return height - margin - (y / Ly) * (height - 2 * margin)
    return sx, sy


def _render_rect(zone: List[float], sx, sy, fill: str, stroke: str, opacity: float = 1.0, dash: Optional[str] = None) -> str:
	if len(zone) != 4:
		return ""
	x0, y0, x1, y1 = zone
	x = sx(x0)
	y = sy(y1)  # top-left in SVG
	w = sx(x1) - sx(x0)
	h = sy(y0) - sy(y1)
	dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
	return f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" fill="{fill}" fill-opacity="{opacity}" stroke="{stroke}" stroke-width="1" {dash_attr}/>'


def _render_oriented_rect(center: List[float] | Tuple[float, float],
                          half_extents: List[float] | Tuple[float, float],
                          yaw: float,
                          sx,
                          sy,
                          fill: str,
                          stroke: str,
                          opacity: float = 1.0,
                          dash: Optional[str] = None) -> str:
	try:
		cx, cy = float(center[0]), float(center[1])  # type: ignore[index]
		hx, hy = float(half_extents[0]), float(half_extents[1])  # type: ignore[index]
	except Exception:
		return ""
	c = math.cos(yaw)
	s = math.sin(yaw)
	corners = []
	for sxh, syh in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
		x = cx + sxh * c - syh * s
		y = cy + sxh * s + syh * c
		corners.append((x, y))
	pts = " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in corners)
	dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
	return f'<polygon points="{pts}" fill="{fill}" fill-opacity="{opacity}" stroke="{stroke}" stroke-width="1" {dash_attr}/>'


def _render_polyline(points: List[Tuple[float, float]], sx, sy, stroke: str, width: float = 2.0, opacity: float = 1.0) -> str:
	if len(points) < 2:
		return '<text x="50%" y="50%" text-anchor="middle" fill="#9ca3af">no trajectory</text>'
	pts = " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in points)
	return f'<polyline points="{pts}" fill="none" stroke="{stroke}" stroke-width="{width}" stroke-linecap="round" stroke-linejoin="round" opacity="{opacity}"/>'


def _render_map(world: dict, run: dict, colors: dict) -> str:
	Lx, Ly = (20.0, 20.0)
	env = world.get("environment", {}) if isinstance(world, dict) else {}
	if isinstance(env, dict):
		msize = env.get("map_size_m")
		if isinstance(msize, (list, tuple)) and len(msize) == 2:
			Lx, Ly = float(msize[0] or Lx), float(msize[1] or Ly)
	sx, sy = _scale_builder((Lx, Ly), width=1000.0, height=420.0, margin=28.0)
	svg_parts = []
	svg_parts.append(f'<rect x="0" y="0" width="1000" height="420" fill="url(#bg)" />')

	floor_zones = world.get("floor_zones", []) if isinstance(world, dict) else []
	for zone in floor_zones:
		z = zone.get("zone")
		if isinstance(z, list) and len(z) == 4:
			svg_parts.append(_render_rect(z, sx, sy, fill=colors["traction"], stroke=colors["traction"], opacity=0.35))

	aisles = world.get("aisles", []) if isinstance(world, dict) else []
	for a in aisles:
		z = a.get("zone") if isinstance(a, dict) else None
		rendered = ""
		if isinstance(a, dict) and isinstance(a.get("center"), (list, tuple)) and isinstance(a.get("half_extents"), (list, tuple)):
			yaw = _parse_float(a.get("yaw"), default=0.0)
			rendered = _render_oriented_rect(a.get("center"), a.get("half_extents"), yaw, sx, sy, fill=colors["aisle"], stroke="#cbd5e1", opacity=0.7)
		if not rendered and isinstance(z, list) and len(z) == 4:
			rendered = _render_rect(z, sx, sy, fill=colors["aisle"], stroke="#cbd5e1", opacity=0.7)
		if rendered:
			svg_parts.append(rendered)

	static_obs = world.get("static_obstacles", []) if isinstance(world, dict) else []
	for ob in static_obs:
		z = ob.get("aabb")
		rendered = ""
		if isinstance(ob, dict) and isinstance(ob.get("center"), (list, tuple)) and isinstance(ob.get("half_extents"), (list, tuple)):
			yaw = _parse_float(ob.get("yaw"), default=0.0)
			rendered = _render_oriented_rect(ob.get("center"), ob.get("half_extents"), yaw, sx, sy, fill=colors["rack"], stroke="#8b5cf6", opacity=0.65)
		if not rendered and isinstance(z, list) and len(z) == 4:
			rendered = _render_rect(z, sx, sy, fill=colors["rack"], stroke="#8b5cf6", opacity=0.65)
		if rendered:
			svg_parts.append(rendered)

	# Start/goal markers from world start/goal if present
	start = world.get("start") if isinstance(world, dict) else None
	goal = world.get("goal") if isinstance(world, dict) else None
	if isinstance(start, (list, tuple)) and len(start) == 2:
		svg_parts.append(f'<circle cx="{sx(start[0]):.1f}" cy="{sy(start[1]):.1f}" r="7" fill="#16a34a" stroke="#065f46" stroke-width="2"/>')
		svg_parts.append(f'<text x="{sx(start[0]):.1f}" y="{sy(start[1])-12:.1f}" font-size="12" text-anchor="middle" fill="#065f46">start</text>')
	if isinstance(goal, (list, tuple)) and len(goal) == 2:
		svg_parts.append(f'<rect x="{sx(goal[0])-7:.1f}" y="{sy(goal[1])-7:.1f}" width="14" height="14" fill="#f97316" stroke="#c2410c" stroke-width="2" rx="3"/>')
		svg_parts.append(f'<text x="{sx(goal[0]):.1f}" y="{sy(goal[1])+20:.1f}" font-size="12" text-anchor="middle" fill="#c2410c">goal</text>')

	# Robot path
	path_pts = run.get("path", [])
	if path_pts:
		svg_parts.append(_render_polyline(path_pts, sx, sy, stroke=colors["robot"], width=3.0, opacity=0.9))

	# Events
	for ev in run.get("events", []):
		if not isinstance(ev, dict):
			continue
		x, y = ev.get("x"), ev.get("y")
		if x is None or y is None or math.isnan(_parse_float(x)) or math.isnan(_parse_float(y)):
			continue
		t = ev.get("t", 0)
		typ = ev.get("type", "")
		color = colors.get(_event_color_key(str(typ)), colors["event_other"])
		svg_parts.append(f'<circle cx="{sx(float(x)):.1f}" cy="{sy(float(y)):.1f}" r="6" fill="{color}" stroke="#0f172a" stroke-width="1.2" opacity="0.95"><title>{typ} @ {t:.1f}s</title></circle>')

	svg = [
		'<svg viewBox="0 0 1000 420" preserveAspectRatio="xMidYMid meet">',
		'<defs><linearGradient id="bg" x1="0" y1="0" x2="1" y2="1"><stop offset="0%" stop-color="#f8fafc"/><stop offset="100%" stop-color="#eef2ff"/></linearGradient></defs>',
		"".join(svg_parts),
		'</svg>',
	]
	return "".join(svg)


def _render_line_chart(series: List[Tuple[float, float]], events: List[Event], title: str, color: str, y_label: str, y_min: float | None = None, y_max: float | None = None) -> str:
    if not series:
        return '<div class="muted">No data</div>'
    width, height, margin = 520, 240, 36
    finite_series = [(t, v) for t, v in series if math.isfinite(t) and math.isfinite(v)]
    if not finite_series:
        return '<div class="muted">No data</div>'
    ts = [p[0] for p in finite_series]
    vs = [p[1] for p in finite_series]
    if not ts or not vs:
        return '<div class="muted">No data</div>'
    tmin, tmax = min(ts), max(ts)
    vmin = y_min if y_min is not None else min(vs)
    vmax_raw = y_max if y_max is not None else max(vs)
    # cap extreme spikes so the curve is readable
    v_sorted = sorted(vs)
    v95 = v_sorted[int(0.95 * (len(v_sorted) - 1))] if v_sorted else vmax_raw
    if y_max is not None:
        vmax = vmax_raw
    else:
        vmax = min(vmax_raw, v95 * 1.5) if v95 > 0 else vmax_raw
    if vmax - vmin < 1e-6:
        vmax = vmin + 1.0
    def sx(t: float) -> float:
        return margin + (t - tmin) / max(1e-6, (tmax - tmin)) * (width - 2 * margin)
    def sy(v: float) -> float:
        return height - margin - (v - vmin) / max(1e-6, (vmax - vmin)) * (height - 2 * margin)
    clamped_series = [(t, min(max(v, vmin), vmax)) for t, v in finite_series]
    pts = " ".join(f"{sx(t):.1f},{sy(v):.1f}" for t, v in clamped_series)
    ev_marks = []
    for ev in events:
        t = _parse_float(ev.get("t"))
        if math.isnan(t):
            continue
        ev_marks.append(f'<line x1="{sx(t):.1f}" x2="{sx(t):.1f}" y1="{margin}" y2="{height-margin}" stroke="#e5e7eb" stroke-dasharray="4 4"/>')
    # axes ticks (3-point)
    xm = (tmin + tmax) / 2.0
    ym = (vmin + vmax) / 2.0
    x_labels = "".join([
        f'<text x="{sx(tmin):.1f}" y="{height - margin + 18:.1f}" font-size="11" text-anchor="start" fill="#6b7280">{tmin:.1f}</text>',
        f'<text x="{sx(xm):.1f}" y="{height - margin + 18:.1f}" font-size="11" text-anchor="middle" fill="#6b7280">{xm:.1f}</text>',
        f'<text x="{sx(tmax):.1f}" y="{height - margin + 18:.1f}" font-size="11" text-anchor="end" fill="#6b7280">{tmax:.1f}</text>',
    ])
    y_labels = "".join([
        f'<text x="{margin - 6:.1f}" y="{sy(vmin):.1f}" font-size="11" text-anchor="end" fill="#6b7280">{vmin:.2f}</text>',
        f'<text x="{margin - 6:.1f}" y="{sy(ym):.1f}" font-size="11" text-anchor="end" fill="#6b7280">{ym:.2f}</text>',
        f'<text x="{margin - 6:.1f}" y="{sy(vmax):.1f}" font-size="11" text-anchor="end" fill="#6b7280">{vmax:.2f}</text>',
    ])
    svg = f"""
    <svg viewBox="0 0 {width} {height}" preserveAspectRatio="xMidYMid meet">
      <rect x="0" y="0" width="{width}" height="{height}" fill="#f8fafc" rx="10" />
      <line x1="{margin}" y1="{height - margin}" x2="{width - margin}" y2="{height - margin}" stroke="#e5e7eb" />
      <line x1="{margin}" y1="{margin}" x2="{margin}" y2="{height - margin}" stroke="#e5e7eb" />
      <polyline points="{pts}" fill="none" stroke="{color}" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round" />
      {''.join(ev_marks)}
      {x_labels}
      {y_labels}
      <text x="{width - margin}" y="{margin - 10}" font-size="12" text-anchor="end" fill="#6b7280">{y_label}</text>
    </svg>
    """
    return svg


def _coverage_block(coverage: dict, outcomes: dict) -> str:
    parts = []
    if outcomes:
        parts.append("<h3>Outcomes</h3>")
        parts.append("<div class='metric-grid tight'>")
        for k, v in outcomes.items():
            parts.append(f"<div class='metric'><span class='label'>{k}</span><div class='value'>{v}</div></div>")
        parts.append("</div>")
    parts.append("<h3>Coverage</h3>")
    if not coverage:
        parts.append('<div class="muted" style="margin-top:6px;">coverage.json not found; coverage stats skipped.</div>')
        return "".join(parts)
    for title, key in [("Traction", "traction_pct"), ("Human phase", "human_phase_pct"), ("Clearance bands", "clearance_bands_pct"), ("Outcomes pct", "outcomes_pct")]:
        sec = coverage.get(key, {})
        if not isinstance(sec, dict) or not sec:
            continue
        parts.append(f"<h3>{title}</h3>")
        parts.append("<div>")
        for b, v in sec.items():
            parts.append(f"<span class='tag'>{b}: {v}%</span>")
        parts.append("</div>")
    return "".join(parts)


def _events_table(events: List[Event]) -> str:
    if not events:
        return '<div class="muted">No notable events recorded.</div>'
    rows = ["<table class='table'><thead><tr><th>Event</th><th>t (s)</th><th>min_clear (m)</th><th>min_ttc (s)</th><th>Note</th></tr></thead><tbody>"]
    for ev in sorted(events, key=lambda e: e.get("t", 0)):
        label = ev.get("label") or ev.get("type")
        t = ev.get("t", 0.0)
        mc = ev.get("min_clearance")
        ttc = ev.get("min_ttc")
        note = ev.get("detail", "") or ""
        rows.append(f"<tr><td>{label}</td><td>{t:.1f}</td><td>{'' if mc is None or math.isnan(_parse_float(mc)) else f'{float(mc):.2f}'}</td><td>{'' if ttc is None or math.isnan(_parse_float(ttc)) else f'{float(ttc):.2f}'}</td><td>{note}</td></tr>")
    rows.append("</tbody></table>")
    return "".join(rows)


def _actors_block(actors: Dict[str, dict]) -> str:
    if not actors:
        return '<div class="muted">actors.csv not found or empty.</div>'
    parts = ["<table class='table'><thead><tr><th>ID</th><th>Type</th><th>Path pts</th><th>Phases seen</th></tr></thead><tbody>"]
    for aid, a in sorted(actors.items()):
        phases = ", ".join(a.get("phases", [])) or "—"
        parts.append(f"<tr><td>{aid}</td><td>{a.get('type','')}</td><td>{len(a.get('path', []))}</td><td>{phases}</td></tr>")
    parts.append("</tbody></table>")
    return "".join(parts)


def _pick_gallery(batch_dir: Path, prefer_failures: bool = True, limit: int = 6) -> List[Tuple[str, str]]:
    per_run = batch_dir / "per_run"
    frames = batch_dir / "frames_sample"
    chosen: List[Tuple[str, str]] = []
    outcomes: Dict[str, Tuple[str, float, float]] = {}
    for run_csv in sorted(per_run.glob("run_*/run_one.csv")):
        run_name = run_csv.parent.name
        with run_csv.open(newline="", encoding="utf-8") as f:
            r = csv.DictReader(f)
            end_event = ""
            last_min = math.inf
            last_t = 0.0
            for row in r:
                ev = row.get("event", "")
                if ev:
                    end_event = ev
                mc = _parse_float(row.get("min_clearance_geom"), default=math.inf)
                if math.isnan(mc):
                    mc = _parse_float(row.get("min_clearance_lidar"), default=math.inf)
                if mc < last_min:
                    last_min = mc
                try:
                    last_t = float(row.get("t", "") or "0")
                except Exception:
                    pass
            outcomes[run_name] = (end_event or "other", last_min if last_min < math.inf else float("nan"), last_t)

    candidates: List[Tuple[str, str]] = []
    if frames.exists():
        for run_dir in sorted(frames.glob("run_*")):
            run_name = run_dir.name
            imgs = sorted(run_dir.glob("*.png"))
            if not imgs:
                continue
            img_rel = f"frames_sample/{run_name}/{imgs[-1].name}"
            ev, mclr, tend = outcomes.get(run_name, ("other", float("nan"), float("nan")))
            caption = f"{run_name}: {ev}, min_clear={mclr:.2f}, t_end={tend:.1f}s"
            candidates.append((caption, img_rel))

    if prefer_failures:
        for cap, rel in candidates:
            if "collision" in cap and len(chosen) < limit:
                chosen.append((cap, rel))
    for cap, rel in candidates:
        if len(chosen) >= limit:
            break
        if (cap, rel) not in chosen:
            chosen.append((cap, rel))
    return chosen[:limit]


# ---------------------------------------------------------------------------
# Main report builder
# ---------------------------------------------------------------------------

def generate_report(batch_dir: Path) -> Path:
    title = batch_dir.name
    scenario_path = batch_dir / "scenario.yaml"
    seeds_path = batch_dir / "seeds.json"
    per_run_dir = batch_dir / "per_run"
    coverage_path = batch_dir / "coverage.json"
    validation_path = batch_dir / "validation.json"
    manifest_path = batch_dir / "manifest.json"

    scenario = _safe_load_yaml(scenario_path)
    seeds = _safe_load_json(seeds_path)
    manifest = _safe_load_json(manifest_path)

    # Prefer root world.json; otherwise take first run world.json
    world_path = batch_dir / "world.json"
    if not world_path.exists():
        runs_candidate = _find_run_dirs(batch_dir)
        for rdir in runs_candidate:
            cand = rdir / "world.json"
            if cand.exists():
                world_path = cand
                break
    world = _safe_load_json(world_path) if world_path.exists() else {}

    run_dirs = _find_run_dirs(batch_dir)
    runs_data = [_parse_run(rd) for rd in run_dirs]
    actors_data = _parse_actors(run_dirs[0]) if run_dirs else {}

    # Layout / hazard counts for summary
    layout = scenario.get("layout", {}) if isinstance(scenario, dict) else {}
    hazards_cfg = scenario.get("hazards", {}) if isinstance(scenario, dict) else {}
    aisle_count = len(world.get("aisles", []) or layout.get("aisles", []) or [])
    patch_count = len(world.get("floor_zones", []) or hazards_cfg.get("traction", []) or [])
    human_count = len(hazards_cfg.get("human", []) or [])
    vehicle_count = len(hazards_cfg.get("vehicles", []) or [])
    static_from_world = world.get("static_obstacles", []) if isinstance(world, dict) else []
    geom = layout.get("geometry", {}) if isinstance(layout, dict) else {}
    racking = geom.get("racking", []) if isinstance(geom, dict) else []
    static_count = len(static_from_world) if static_from_world else len(racking)

    # Aggregate outcomes
    success = sum(1 for r in runs_data if r["outcome"] in ("success", "mission_success"))
    collisions = sum(1 for r in runs_data if "collision" in str(r["outcome"]))
    other = max(0, len(runs_data) - success - collisions)
    runs = len(runs_data)

    # Representative run: prefer collision, then lowest clearance
    rep_run = None
    for r in runs_data:
        if "collision" in str(r["outcome"]):
            rep_run = r
            break
    if rep_run is None and runs_data:
        rep_run = min(runs_data, key=lambda r: (r["min_clearance"] if not math.isnan(r["min_clearance"]) else 1e9))
    rep_run = rep_run or (runs_data[0] if runs_data else {"run_name": "N/A", "events": [], "path": [], "clearance": [], "ttc": [], "min_clearance": math.nan, "min_ttc": math.nan})

    # Coverage
    coverage = _safe_load_json(coverage_path) if coverage_path.exists() else {}
    coverage_block = _coverage_block(coverage, {"success": success, "collisions": collisions, "other": other})

    # Domain/randomization hints
    lidar_cfg = (scenario.get("sensors") or {}).get("lidar", {}) if isinstance(scenario, dict) else {}
    lidar_noise = f"{lidar_cfg.get('noise_sigma','?')}, drop {lidar_cfg.get('dropout_pct','?')}"
    floor_surfaces = (scenario.get("layout") or {}).get("floor_surfaces", []) if isinstance(scenario, dict) else []
    mu_dry = [fs.get("mu") for fs in floor_surfaces if isinstance(fs, dict) and str(fs.get("type","")).lower() == "dry" and fs.get("mu") is not None]
    mu_wet = [fs.get("mu") for fs in floor_surfaces if isinstance(fs, dict) and str(fs.get("type","")).lower() != "dry" and fs.get("mu") is not None]
    traction_hint = f"dry {min(mu_dry):.2f}-{max(mu_dry):.2f}" if mu_dry else "—"
    if mu_wet:
        traction_hint += f" • wet {min(mu_wet):.2f}-{max(mu_wet):.2f}"
    hazards = scenario.get("hazards", {}) if isinstance(scenario, dict) else {}
    tax = scenario.get("taxonomy", {}) if isinstance(scenario, dict) else {}
    taxonomy_tags = "".join(f"<span class='tag'>{k}={v}</span>" for k, v in tax.items()) if isinstance(tax, dict) and tax else '<span class="muted">No taxonomy flags</span>'
    runtime_cfg = scenario.get("runtime", {}) if isinstance(scenario, dict) else {}
    runtime_hint = f"{runtime_cfg.get('duration_s','?')}s @ dt={runtime_cfg.get('dt','?')}"

    # Digests and manifest stamps
    per_run_digest = _digest_csvs(per_run_dir) if per_run_dir.exists() else (_sha256_file(batch_dir / "run_one.csv") if (batch_dir / "run_one.csv").exists() else "NA")
    per_run_digest_short = (per_run_digest[:12] + "…") if per_run_digest != "NA" else "NA"
    scenario_sha = _sha256_file(scenario_path) if scenario_path.exists() else "NA"
    seeds_sha = _sha256_file(seeds_path) if seeds_path.exists() else "NA"
    world_sha = _sha256_file(world_path) if world_path.exists() else "NA"

    # Update manifest with latest digests
    try:
        manifest.update({
            "per_run_digest": per_run_digest,
            "scenario_sha256": scenario_sha,
            "seeds_sha256": seeds_sha,
            "world_sha256": world_sha,
            "generated_at": datetime.utcnow().isoformat(timespec="seconds") + "Z",
        })
        manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    except Exception:
        pass

    # Charts
    clearance_chart = _render_line_chart(rep_run.get("clearance", []), rep_run.get("events", []), "Clearance", color="#2563eb", y_label="meters")
    ttc_chart = _render_line_chart(rep_run.get("ttc", []), rep_run.get("events", []), "TTC", color="#f97316", y_label="seconds", y_min=0)

    # Events table
    events_table = _events_table(rep_run.get("events", []))
    actors_block = _actors_block(actors_data)

    # Gallery
    gallery_html = ""

    # Map SVG colors
    colors = {
        "robot": "#2563eb",
        "aisle": "#e2e8f0",
        "rack": "#a16207",
        "traction": "#bae6fd",
        "ev_collision": "#dc2626",
        "ev_floor_slip": "#06b6d4",   # slips: cyan
        "ev_near_miss": "#f97316",    # near-miss/occluded: orange
        "ev_timeout": "#9ca3af",
        "ev_success": "#16a34a",
        "event_other": "#6366f1",
    }
    map_svg = _render_map(world, rep_run, colors)

    # Outcome legend badges
    outcome_legend = "".join([
        '<span class="chip">success</span>',
        '<span class="chip" style="background:#fee2e2;color:#991b1b;border-color:#fecdd3;">collision</span>',
        '<span class="chip" style="background:#fef3c7;color:#92400e;border-color:#fde68a;">other/timeout</span>',
    ])

    seeds_hint_val = str(seeds.get("base_seed", "NA")) if isinstance(seeds, dict) else "NA"

    html = HTML_TEMPLATE.format(
        title=title,
        scenario_name=title,
        runs=runs,
        generated_at=datetime.utcnow().strftime("%Y-%m-%d %H:%M UTC"),
        seeds_hint=seeds_hint_val,
        per_run_digest_short=per_run_digest_short,
        world_sha_short=(world_sha[:12] + "…") if world_sha != "NA" else "NA",
        prompt=manifest.get("prompt", title) if isinstance(manifest, dict) else title,
        site_profile=manifest.get("site", "default") if isinstance(manifest, dict) else "default",
        lidar_noise=lidar_noise,
        traction_hint=traction_hint,
        runtime_hint=runtime_hint,
        taxonomy_tags=taxonomy_tags,
        success=success,
        collisions=collisions,
        other=other,
        worst_clearance=("NA" if math.isnan(rep_run.get("min_clearance", math.nan)) else f"{rep_run.get('min_clearance', 0):.2f} m"),
        min_ttc=("NA" if math.isnan(rep_run.get("min_ttc", math.nan)) else f"{rep_run.get('min_ttc', 0):.2f} s"),
        rep_run=rep_run.get("run_name", "N/A"),
        outcome_legend=outcome_legend,
        map_svg=map_svg,
        color_robot=colors["robot"],
        color_event_collision=colors["ev_collision"],
        color_event_slip=colors["ev_floor_slip"],
        color_event_near=colors["ev_near_miss"],
        color_traction=colors["traction"],
        color_rack=colors["rack"],
        clearance_chart=clearance_chart,
        ttc_chart=ttc_chart,
        events_table=events_table,
        actors_block=actors_block,
        coverage_block=coverage_block,
        gallery=gallery_html,
        per_run_digest=per_run_digest,
        scenario_sha=scenario_sha,
        seeds_sha=seeds_sha,
        scenario_sha_short=(scenario_sha[:12] + "…") if scenario_sha != "NA" else "NA",
        seeds_sha_short=(seeds_sha[:12] + "…") if seeds_sha != "NA" else "NA",
        aisle_count=aisle_count,
        patch_count=patch_count,
        human_count=human_count,
        vehicle_count=vehicle_count,
        static_count=static_count,
        verify_path=f"runs/{title}",
    )

    out_path = batch_dir / "report.html"
    out_path.write_text(html, encoding="utf-8")
    return out_path


def main():
    import sys
    if len(sys.argv) != 2:
        print("Usage: python -m edgesim.report_html runs/<batch_dir>")
        raise SystemExit(2)
    batch_dir = Path(sys.argv[1]).resolve()
    if not batch_dir.exists():
        print(f"Batch dir not found: {batch_dir}")
        raise SystemExit(2)
    out = generate_report(batch_dir)
    print(f"[OK] Wrote {out}")


if __name__ == "__main__":
    main()

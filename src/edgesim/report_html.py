from __future__ import annotations
import json, csv, hashlib, math
from pathlib import Path
from typing import List, Dict, Tuple, Optional
from datetime import datetime

HTML_TEMPLATE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8" />
<title>EdgeSim Report — {title}</title>
<style>
body {{ font-family: system-ui, -apple-system, Segoe UI, Roboto, Helvetica, Arial, sans-serif; margin: 24px; color: #111; }}
h1 {{ margin: 0 0 8px 0; }}
h2 {{ margin-top: 28px; }}
.card {{ border: 1px solid #e5e7eb; border-radius: 12px; padding: 16px; margin: 12px 0; }}
.grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(240px, 1fr)); gap: 12px; }}
.small {{ color: #555; font-size: 12px; }}
.badge {{ display:inline-block; padding:2px 8px; border-radius:999px; background:#eef2ff; color:#3730a3; font-size:12px; }}
.table {{ width: 100%; border-collapse: collapse; }}
.table th, .table td {{ border-bottom: 1px solid #eee; padding: 8px; text-align: left; }}
.code {{ font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, "Liberation Mono", monospace; font-size: 12px; background: #f8fafc; padding: 2px 6px; border-radius: 6px; }}
.svg {{ width: 100%; height: 160px; border: 1px solid #eee; border-radius: 8px; background: #fafafa; }}
.gallery img {{ width: 100%; height: auto; border-radius: 8px; border:1px solid #eee; }}
footer {{ margin-top: 24px; color:#666; font-size:12px; }}
.kv {{ display:grid; grid-template-columns: 160px 1fr; gap:8px; }}
.kv div:nth-child(odd) {{ color:#555; }}
.tag {{ display:inline-block; background:#f1f5f9; border:1px solid #e2e8f0; border-radius:999px; padding:2px 8px; margin:2px; font-size:12px; }}
.acclist li {{ margin:6px 0; }}
.pass {{ color:#065f46; }}
.fail {{ color:#b91c1c; }}
.warn {{ color:#92400e; }}
</style>
</head>
<body>
<h1>EdgeSim Report</h1>
<div class="small">Scenario: <span class="code">{scenario_name}</span> • Runs: <b>{runs}</b> • Generated: {generated_at}</div>
<div class="small">Manifest: per_run_digest <span class="code">{per_run_digest}</span> • scenario.yaml <span class="code">{scenario_sha}</span> • seeds.json <span class="code">{seeds_sha}</span> • {repro_badge}</div>

<div class="card">
  <h2>Acceptance Checklist</h2>
  <ul class="acclist">
    <li class="{chk_failrate_cls}">Failure rate ≥ 10% (stress): <b>{fail_rate:.2f}%</b></li>
    <li class="{chk_repro_cls}">Reproducibility stamp present (CSV digest): <b>{per_run_digest_short}</b></li>
    <li class="{chk_gap_cls}">Coverage shows at least one uncovered bucket (“gap”): <b>{gap_hint}</b></li>
  </ul>
  <div class="small">Run <span class="code">edgesim validate {title}</span> to see invariant checks in the console.</div>
</div>

<div class="card">
  <h2>Outcomes</h2>
  <div class="grid">
    <div>
      <div>Successes: <b>{success}</b></div>
      <div>Collision (human): <b>{collision_human}</b></div>
      <div>Other/Timeout: <b>{other}</b></div>
    </div>
    <div>
      <svg class="svg" viewBox="0 0 400 160">{svg_outcomes}</svg>
    </div>
  </div>
</div>

<div class="card">
  <h2>Coverage (taxonomy)</h2>
  {coverage_html}
</div>

<div class="card">
  <h2>Min-Clearance (per run)</h2>
  <svg class="svg" viewBox="0 0 400 160">{svg_clearance}</svg>
</div>

<div class="card">
  <h2>Time-to-Goal (successes only)</h2>
  <svg class="svg" viewBox="0 0 400 160">{svg_ttg}</svg>
</div>

<div class="card">
  <h2>Calibration (stub)</h2>
  <div class="kv">
    <div>R² (speed profile)</div><div>{calib_r2}</div>
    <div>KL (obstacle range hist)</div><div>{calib_kl}</div>
    <div>ECE (failure prob)</div><div>{calib_ece}</div>
  </div>
  <div class="small" style="margin-top:8px;">This is a V0 placeholder. Pair with a small real anchor log to compute metrics; auto-tuning loop lands in V0.1.</div>
</div>

<div class="card">
  <h2>Sample Frames (failures preferred)</h2>
  <div class="grid gallery">
    {gallery}
  </div>
</div>

<footer>EdgeSim • offline HTML report • no external JS • v0</footer>
</body>
</html>
"""

def _sha256_file(path: Path) -> str:
	h = hashlib.sha256()
	h.update(path.read_bytes())
	return h.hexdigest()

def _digest_csvs(per_run_dir: Path) -> str:
	h = hashlib.sha256()
	for p in sorted(per_run_dir.rglob("*.csv")):
		h.update(p.read_bytes())
	return h.hexdigest()

def _load_counts_and_metrics(batch_dir: Path) -> Tuple[int, int, int, List[float], List[float]]:
	"""Return (success, coll_human, other, min_clearances, ttg_success)."""
	per_run = batch_dir / "per_run"
	success = coll_human = other = 0
	mincls: List[float] = []
	ttg: List[float] = []

	for run_csv in sorted(per_run.glob("run_*/run_one.csv")):
		with run_csv.open(newline="", encoding="utf-8") as f:
			r = csv.DictReader(f)
			last_t = 0.0
			last_min = math.inf
			end_event = ""
			for row in r:
				try:
					t = float(row["t"])
					last_t = t
				except Exception:
					pass
				try:
					mc = float(row["min_clearance"])
					if mc < last_min:
						last_min = mc
				except Exception:
					pass
				ev = row.get("event","")
				if ev:
					end_event = ev
			if last_min is math.inf:
				last_min = float("nan")
			mincls.append(last_min)
			if end_event == "success":
				success += 1
				ttg.append(last_t)
			elif end_event == "collision_human":
				coll_human += 1
			else:
				other += 1
	return success, coll_human, other, mincls, ttg

def _svg_bar_chart(values: List[int], labels: List[str], colors: Optional[List[str]] = None, max_height: int = 120) -> str:
	total = max(1, max(values))
	w = 360; x0 = 20; y0 = 120
	bar_w = int((w - x0 - 20) / max(1, len(values)))
	parts = []
	for i, v in enumerate(values):
		hp = int(max_height * (v / total))
		x = x0 + i * bar_w
		y = y0 - hp
		fill = colors[i] if colors and i < len(colors) else "#60a5fa"
		parts.append(f'<rect x="{x}" y="{y}" width="{bar_w-8}" height="{hp}" fill="{fill}" />')
		parts.append(f'<text x="{x + (bar_w-8)/2}" y="{y0 + 14}" font-size="12" text-anchor="middle">{labels[i]}</text>')
	return "".join(parts)

def _svg_hist(data: List[float], bins: int, low: float, high: float, color="#34d399", max_height: int = 120) -> str:
	if not data:
		return '<text x="200" y="80" text-anchor="middle" fill="#999">no data</text>'
	step = (high - low) / bins if bins > 0 else 1.0
	hist = [0] * bins
	for v in data:
		if math.isnan(v):
			continue
		k = int((v - low) / step)
		if k < 0: k = 0
		if k >= bins: k = bins - 1
		hist[k] += 1
	total = max(1, max(hist))
	w = 360; x0 = 20; y0 = 120
	bar_w = int((w - x0 - 20) / max(1, bins))
	parts = []
	for i, v in enumerate(hist):
		hp = int(max_height * (v / total))
		x = x0 + i * bar_w
		y = y0 - hp
		parts.append(f'<rect x="{x}" y="{y}" width="{bar_w-2}" height="{hp}" fill="{color}" />')
	parts.append(f'<text x="{x0}" y="{y0 + 14}" font-size="12">{low:.1f}</text>')
	parts.append(f'<text x="{x0 + (w-x0-20)/2}" y="{y0 + 14}" font-size="12" text-anchor="middle">{(low+high)/2:.1f}</text>')
	parts.append(f'<text x="{w}" y="{y0 + 14}" font-size="12" text-anchor="end">{high:.1f}</text>')
	return "".join(parts)

def _pick_gallery(batch_dir: Path, prefer_failures: bool = True, limit: int = 8) -> List[Tuple[str, str]]:
	per_run = batch_dir / "per_run"
	frames = batch_dir / "frames_sample"
	chosen: List[Tuple[str, str]] = []

	outcomes: Dict[str, Tuple[str, float, float]] = {}
	for run_csv in sorted(per_run.glob("run_*/run_one.csv")):
		run_name = run_csv.parent.name
		with run_csv.open(newline="", encoding="utf-8") as f:
			r = csv.DictReader(f)
			end_event = ""; last_min = math.inf; last_t = 0.0
			for row in r:
				ev = row.get("event", "")
				if ev:
					end_event = ev
				try:
					mc = float(row["min_clearance"])
					if mc < last_min:
						last_min = mc
				except Exception:
					pass
				try:
					last_t = float(row["t"])
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
			if "collision_human" in cap and len(chosen) < limit:
				chosen.append((cap, rel))
	for cap, rel in candidates:
		if len(chosen) >= limit:
			break
		if (cap, rel) not in chosen:
			chosen.append((cap, rel))
	return chosen[:limit]

def _render_coverage_table(coverage: Dict[str, Dict[str, float] | Dict[str, int]]) -> str:
	if not coverage:
		return '<div class="small">No coverage computed. (Run built without coverage module or no per_run CSVs yet.)</div>'
	def rows_of(key: str) -> List[Tuple[str, str]]:
		sec = coverage.get(key, {})
		if not isinstance(sec, dict):
			return []
		items = list(sec.items())
		return [(k, f"{v:.2f}%") if isinstance(v, float) else (k, str(v)) for k, v in items]

	def table_for(title: str, key: str) -> str:
		rows = rows_of(key)
		if not rows:
			return ""
		t = [f"<h3 style='margin:8px 0 6px 0;'>{title}</h3>",
		     "<table class='table'><thead><tr><th>Bucket</th><th>Value</th></tr></thead><tbody>"]
		for k, v in rows:
			t.append(f"<tr><td>{k}</td><td>{v}</td></tr>")
		t.append("</tbody></table>")
		return "\n".join(t)

	parts = []
	parts.append(table_for("Traction (pct)", "traction_pct"))
	parts.append(table_for("Human phase (pct)", "human_phase_pct"))
	parts.append(table_for("Clearance bands (pct)", "clearance_bands_pct"))
	parts.append(table_for("Outcomes (pct)", "outcomes_pct"))
	parts.append(table_for("Traction (counts)", "traction"))
	parts.append(table_for("Human phase (counts)", "human_phase"))
	parts.append(table_for("Clearance bands (counts)", "clearance_bands"))
	parts.append(table_for("Outcomes (counts)", "outcomes"))
	return "\n".join(x for x in parts if x)

def _any_coverage_gap(coverage: Dict[str, Dict[str, float] | Dict[str, int]]) -> Tuple[bool, str]:
	"""
	Heuristic: if any percentage block has a 0.00% bucket, consider that a 'gap'.
	Returns (has_gap, hint).
	"""
	pct_keys = ["traction_pct","human_phase_pct","clearance_bands_pct","outcomes_pct"]
	for key in pct_keys:
		sec = coverage.get(key, {})
		if isinstance(sec, dict):
			for b, v in sec.items():
				try:
					if float(v) <= 1e-9:
						return True, f"{key}:{b}=0%"
				except Exception:
					continue
	return False, "none detected"

def generate_report(batch_dir: Path) -> Path:
	title = batch_dir.name
	scenario_path = batch_dir / "scenario.yaml"
	seeds_path = batch_dir / "seeds.json"
	per_run_dir = batch_dir / "per_run"
	summary_path = batch_dir / "summary.json"
	coverage_path = batch_dir / "coverage.json"

	per_run_digest = _digest_csvs(per_run_dir) if per_run_dir.exists() else "NA"
	per_run_digest_short = per_run_digest[:12] + "…" if per_run_digest != "NA" else "NA"
	scenario_sha = _sha256_file(scenario_path) if scenario_path.exists() else "NA"
	seeds_sha = _sha256_file(seeds_path) if seeds_path.exists() else "NA"

	manifest = {
		"per_run_digest": per_run_digest,
		"scenario_sha256": scenario_sha,
		"seeds_sha256": seeds_sha,
		"generated_at": datetime.utcnow().isoformat(timespec="seconds") + "Z",
	}
	manifest_path = batch_dir / "manifest.json"
	try:
		if manifest_path.exists():
			old = json.loads(manifest_path.read_text(encoding="utf-8"))
			old.update(manifest)
			manifest = old
		manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
	except Exception:
		pass

	success, coll_human, other, mincls, ttg = _load_counts_and_metrics(batch_dir)
	runs = success + coll_human + other

	# Load coverage.json if present
	coverage: Dict[str, Dict] = {}
	if coverage_path.exists():
		try:
			coverage = json.loads(coverage_path.read_text(encoding="utf-8"))
		except Exception:
			coverage = {}
	coverage_html = _render_coverage_table(coverage)

	# Acceptance checklist computation
	total = max(1, runs)
	fail_rate = 100.0 * (coll_human + other) / total
	chk_failrate_cls = "pass" if fail_rate >= 10.0 else "fail"
	chk_repro_cls = "pass" if per_run_digest != "NA" else "fail"
	has_gap, gap_hint = _any_coverage_gap(coverage)
	chk_gap_cls = "pass" if has_gap else "warn"  # warn if no gap detected (not necessarily bad, but suggests low coverage variety)

	# Calibration stub (NA until wired to anchors)
	calib_r2 = "NA"
	calib_kl = "NA"
	calib_ece = "NA"

	svg_outcomes = _svg_bar_chart([success, coll_human, other],
	                              ["success","collision","other"],
	                              colors=["#10b981","#ef4444","#9ca3af"])
	svg_clearance = _svg_hist(mincls, bins=16, low=0.0, high=2.0, color="#60a5fa")
	svg_ttg = _svg_hist(ttg, bins=16, low=0.0, high=120.0, color="#f59e0b")

	gallery_items = []
	for cap, rel in _pick_gallery(batch_dir, prefer_failures=True, limit=8):
		gallery_items.append(f'<figure><img src="{rel}"/><figcaption class="small">{cap}</figcaption></figure>')
	gallery_html = "\n".join(gallery_items) if gallery_items else '<div class="small">No sample frames available. Re-run with frames enabled.</div>'

	repro_badge = '<span class="badge">Repro stamp ready</span>' if per_run_digest != "NA" else '<span class="badge" style="background:#fee2e2;color:#7f1d1d;">Repro stamp missing</span>'

	html = HTML_TEMPLATE.format(
		title=title,
		scenario_name=title,
		runs=runs,
		generated_at=datetime.utcnow().strftime("%Y-%m-%d %H:%M UTC"),
		per_run_digest=per_run_digest,
		per_run_digest_short=per_run_digest_short,
		scenario_sha=scenario_sha,
		seeds_sha=seeds_sha,
		repro_badge=repro_badge,
		success=success,
		collision_human=coll_human,
		other=other,
		svg_outcomes=svg_outcomes,
		svg_clearance=svg_clearance,
		svg_ttg=svg_ttg,
		coverage_html=coverage_html,
		calib_r2=calib_r2,
		calib_kl=calib_kl,
		calib_ece=calib_ece,
		chk_failrate_cls=chk_failrate_cls,
		chk_repro_cls=chk_repro_cls,
		chk_gap_cls=chk_gap_cls,
		gap_hint=gap_hint,
		fail_rate=fail_rate,
		gallery=gallery_html,
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

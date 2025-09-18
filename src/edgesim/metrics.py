from __future__ import annotations
from pathlib import Path
from typing import Dict, Any, List, Tuple
import csv, json, math

# -------- batch aggregation (unchanged API) --------

def _read_index(per_run_dir: Path) -> List[Dict[str, Any]]:
	index = per_run_dir / "index.csv"
	if not index.exists():
		return []
	out: List[Dict[str, Any]] = []
	with open(index, "r", newline="", encoding="utf-8") as f:
		for i, row in enumerate(csv.reader(f)):
			if i == 0:  # header
				continue
			run_id, success, time_s, steps = row
			out.append({"run_id": run_id, "success": (success.lower() == "true"), "time_s": float(time_s), "steps": int(steps)})
	return out

def aggregate(per_run_dir: Path) -> Dict[str, Any]:
	rows = _read_index(per_run_dir)
	if not rows:
		return {"runs": 0, "successes": 0, "failures": 0, "avg_time": 0.0, "avg_steps": 0}
	runs = len(rows)
	successes = sum(1 for r in rows if r["success"])
	failures = runs - successes
	avg_time = sum(r["time_s"] for r in rows) / runs
	avg_steps = int(round(sum(r["steps"] for r in rows) / runs))
	return {"runs": runs, "successes": successes, "failures": failures, "avg_time": avg_time, "avg_steps": avg_steps}

def write_summary(run_dir: Path, summary: Dict[str, Any]) -> None:
	(run_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

# -------- calibration (new) --------

def _safe_rows(path: Path) -> List[Dict[str,str]]:
	rows = []
	with path.open("r", newline="", encoding="utf-8") as f:
		reader = csv.DictReader(f)
		for row in reader:
			if not row:
				continue
			lrow: Dict[str,str] = {}
			for k, v in row.items():
				if k is None:
					continue
				ks = k.strip().lower()
				vs = (v or "").strip()
				if vs.startswith("#"):  # allow comments in any cell
					vs = ""
				lrow[ks] = vs
			if any(lrow.values()):
				rows.append(lrow)
	return rows

def _r2_from_pairs(pairs: List[Tuple[float,float]]) -> float:
	if len(pairs) < 2: return float("nan")
	y = [b for _, b in pairs]; yhat = [a for a, _ in pairs]
	ybar = sum(y)/len(y)
	ss_tot = sum((yi - ybar)**2 for yi in y)
	ss_res = sum((yi - yhi)**2 for yi, yhi in zip(y, yhat))
	if ss_tot <= 0: return float("nan")
	return max(-1.0, min(1.0, 1.0 - ss_res/ss_tot))

def _kl_from_pairs(pq: List[Tuple[float,float]]) -> float:
	if not pq: return float("nan")
	eps = 1e-9
	p = [max(eps, float(a)) for a,_ in pq]
	q = [max(eps, float(b)) for _,b in pq]
	sp = sum(p); sq = sum(q)
	p = [pi/sp for pi in p]
	q = [qi/sq for qi in q]
	return sum(pi * math.log(pi/qi) for pi,qi in zip(p,q))

def _ece_from_bins(bins: List[Tuple[float,float,int]]) -> float:
	N = sum(c for _,_ ,c in bins)
	if N <= 0: return float("nan")
	return sum((c/N) * abs(conf - acc) for conf, acc, c in bins)

def calibrate_from_anchors(anchors_csv: Path) -> Dict[str, float]:
	rows = _safe_rows(anchors_csv)

	r2_pairs: List[Tuple[float,float]] = []
	kl_pairs: List[Tuple[float,float]] = []
	ece_bins: List[Tuple[float,float,int]] = []

	for r in rows:
		if "pred" in r and "target" in r and r["pred"] != "" and r["target"] != "":
			try: r2_pairs.append((float(r["pred"]), float(r["target"])))
			except: pass
		if "p" in r and "q" in r and r["p"] != "" and r["q"] != "":
			try: kl_pairs.append((float(r["p"]), float(r["q"])))
			except: pass
		if "conf" in r and "acc" in r:
			try:
				c = int(r.get("count","1") or "1")
				ece_bins.append((float(r["conf"]), float(r["acc"]), c))
			except: pass

	metrics = {
		"r2": _r2_from_pairs(r2_pairs),
		"kl": _kl_from_pairs(kl_pairs),
		"ece": _ece_from_bins(ece_bins),
	}
	return metrics

def write_site_profile(site: str, metrics: Dict[str,float]) -> Path:
	out = Path("site_profiles"); out.mkdir(parents=True, exist_ok=True)
	path = out / f"{site}.json"
	path.write_text(json.dumps({"site": site, "metrics": metrics}, indent=2), encoding="utf-8")
	return path

def write_calibration_json(batch_dir: Path, metrics: Dict[str,float]) -> Path:
	path = Path(batch_dir) / "calibration.json"
	path.write_text(json.dumps(metrics, indent=2), encoding="utf-8")
	return path

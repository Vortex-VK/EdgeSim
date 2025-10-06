from __future__ import annotations
from pathlib import Path
from typing import Dict, Any, List, Tuple
import csv, json, math, os

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

def _safe_sum(xs: List[float]) -> float:
	return float(sum(xs)) if xs else 0.0

def parse_anchors_csv(path: Path) -> tuple[
	List[Tuple[float, float]],
	List[Tuple[float, float]],
	List[Tuple[float, float, int]],
]:
	"""
	Accept ONE CSV file that may contain up to three tables back-to-back,
	with blank lines allowed (no comment lines '#'):
	  - R² table: header exactly 'pred,target'
	  - KL table: header exactly 'p,q'
	  - ECE table: header exactly 'conf,acc,count'
	Returns (r2_pairs, kl_pairs, ece_bins).
	"""
	r2_pairs: List[Tuple[float, float]] = []
	kl_pairs: List[Tuple[float, float]] = []
	ece_bins: List[Tuple[float, float, int]] = []

	with path.open("r", newline="", encoding="utf-8") as f:
		reader = csv.reader(f)
		mode = None
		for row in reader:
			if not row or all(not str(c).strip() for c in row):
				continue
			header = [str(c).strip().lower() for c in row]
			# detect headers
			if header == ["pred", "target"]:
				mode = "r2"; continue
			if header == ["p", "q"]:
				mode = "kl"; continue
			if header == ["conf", "acc", "count"]:
				mode = "ece"; continue
			# parse rows under the active mode
			try:
				if mode == "r2":
					r2_pairs.append((float(row[0]), float(row[1])))
				elif mode == "kl":
					kl_pairs.append((float(row[0]), float(row[1])))
				elif mode == "ece":
					# count may be float or int in some sheets; coerce to int safely
					cnt_val = row[2].strip()
					cnt = int(float(cnt_val)) if cnt_val else 1
					ece_bins.append((float(row[0]), float(row[1]), cnt))
				else:
					# ignore lines until a known header appears
					continue
			except Exception:
				# ignore malformed rows
				continue

	return r2_pairs, kl_pairs, ece_bins

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
	eps = 1e-12
	p = [max(eps, float(a)) for a,_ in pq]
	q = [max(eps, float(b)) for _,b in pq]
	sp = _safe_sum(p); sq = _safe_sum(q)
	if sp <= 0 or sq <= 0: return float("nan")
	p = [pi/sp for pi in p]
	q = [qi/sq for qi in q]
	return sum(pi * math.log(pi/qi) for pi,qi in zip(p,q))

def _ece_from_bins(bins: List[Tuple[float,float,int]]) -> float:
	N = sum(max(0, int(c)) for _,_ ,c in bins)
	if N <= 0: return float("nan")
	return sum((max(0, int(c))/N) * abs(conf - acc) for conf, acc, c in bins)

def calibrate_from_anchors(anchors_csv: Path) -> Dict[str, float]:
	r2_pairs, kl_pairs, ece_bins = parse_anchors_csv(anchors_csv)
	metrics = {
		"r2": _r2_from_pairs(r2_pairs),
		"kl": _kl_from_pairs(kl_pairs),
		"ece": _ece_from_bins(ece_bins),
	}
	return metrics

def _heuristic_fit_params(metrics: Dict[str, float]) -> Dict[str, Any]:
	"""
	Very lightweight defaults that nudge realism based on metrics.
	Adjust as you collect more anchors.
	"""
	r2 = metrics.get("r2", float("nan"))
	kl = metrics.get("kl", float("nan"))
	ece = metrics.get("ece", float("nan"))

	tuned: Dict[str, Any] = {
		"lidar":   {"noise_sigma": 0.02, "dropout": 0.01, "latency_ms_max": 50},
		"odom":    {"bias_v": 0.01, "bias_w": 0.01},
		"traction":{"mu_wet_min": 0.55, "mu_wet_max": 0.70},
		"human":   {"slip_prob": 0.08, "fall_duration_s": 4.0},
		"risk":    {"temperature": 1.3},
	}
	# If KL is high, increase lidar noise/dropout a bit
	if isinstance(kl, float) and kl == kl:
		if kl > 0.15:
			tuned["lidar"]["noise_sigma"] = 0.03
			tuned["lidar"]["dropout"] = 0.02
		elif kl > 0.05:
			tuned["lidar"]["noise_sigma"] = 0.025
			tuned["lidar"]["dropout"] = 0.015
	# If ECE is high, soften confidences more
	if isinstance(ece, float) and ece == ece:
		if ece > 0.06:   tuned["risk"]["temperature"] = 1.6
		elif ece > 0.02: tuned["risk"]["temperature"] = 1.3
	# If R² is poor, make environment less punitive so base curve holds speed better
	if isinstance(r2, float) and r2 == r2 and r2 < 0.70:
		tuned["traction"]["mu_wet_min"] = 0.60
		tuned["human"]["slip_prob"]     = 0.06
	return tuned

def write_site_profile(site: str, metrics: Dict[str,float], anchors_path: Path | None = None) -> Path:
	out = Path("site_profiles"); out.mkdir(parents=True, exist_ok=True)
	path = out / f"{site}.json"
	obj: Dict[str, Any] = {"site": site, "metrics": metrics, "tuned": _heuristic_fit_params(metrics)}
	if anchors_path is not None:
		obj["anchors"] = str(anchors_path)
	path.write_text(json.dumps(obj, indent=2), encoding="utf-8")
	return path

def write_calibration_json(batch_dir: Path, site: str, anchors_path: Path, metrics: Dict[str,float]) -> Path:
    path = Path(batch_dir) / "calibration.json"
    obj = {
        "site": site,
        "anchors": str(anchors_path),
        "metrics": metrics,
        # duplicate at top-level for older report_html versions:
        "r2": metrics.get("r2"),
        "kl": metrics.get("kl"),
        "ece": metrics.get("ece"),
    }
    path.write_text(json.dumps(obj, indent=2), encoding="utf-8")
    return path

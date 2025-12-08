"""
Summarize EdgeSim run_one.csv files for quick health checks.

Usage:
  python tools/summarize_runs.py runs/<run_dir> [runs/<run_dir2> ...]

Outputs per run:
  - rows
  - min/mean of min_clearance_lidar and min_clearance_geom
  - min of min_ttc
  - counts of in_wet>0 rows
  - counts of human_phase states (including fallen)
  - event histogram (event field)
"""

from __future__ import annotations

import argparse
import csv
import statistics
from pathlib import Path
from typing import Dict, List


def summarize(run_dir: Path) -> Dict[str, object]:
    csv_path = run_dir / "run_one.csv"
    out: Dict[str, object] = {"run": run_dir.name}
    if not csv_path.exists():
        out["error"] = "missing run_one.csv"
        return out
    rows = []
    with csv_path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    out["rows"] = len(rows)
    def _float_list(field: str) -> List[float]:
        vals = []
        for r in rows:
            try:
                vals.append(float(r.get(field, "")))
            except Exception:
                continue
        return vals
    lid = _float_list("min_clearance_lidar")
    geo = _float_list("min_clearance_geom")
    ttc = _float_list("min_ttc")
    out["min_clearance_lidar_min"] = min(lid) if lid else None
    out["min_clearance_lidar_mean"] = statistics.mean(lid) if lid else None
    out["min_clearance_geom_min"] = min(geo) if geo else None
    out["min_clearance_geom_mean"] = statistics.mean(geo) if geo else None
    out["min_ttc_min"] = min(ttc) if ttc else None
    out["in_wet_rows"] = sum(1 for r in rows if r.get("in_wet") in ("1", "True", "true"))
    # human phases
    phase_counts: Dict[str, int] = {}
    for r in rows:
        phase = (r.get("human_phase") or "none").strip()
        phase_counts[phase] = phase_counts.get(phase, 0) + 1
    out["human_phase_counts"] = phase_counts
    # events
    ev_counts: Dict[str, int] = {}
    for r in rows:
        ev = (r.get("event") or "").strip()
        if not ev:
            continue
        ev_counts[ev] = ev_counts.get(ev, 0) + 1
    out["event_counts"] = ev_counts
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description="Summarize run_one.csv for one or more runs.")
    ap.add_argument("run_dirs", nargs="+", type=Path)
    args = ap.parse_args()
    for rd in args.run_dirs:
        s = summarize(rd)
        if "error" in s:
            print(f"{s['run']}: {s['error']}")
            continue
        print(f"== {s['run']} ==")
        print(f"rows: {s['rows']}")
        print(f"min_clearance_lidar: min={s['min_clearance_lidar_min']} mean={s['min_clearance_lidar_mean']}")
        print(f"min_clearance_geom:  min={s['min_clearance_geom_min']} mean={s['min_clearance_geom_mean']}")
        print(f"min_ttc_min: {s['min_ttc_min']}")
        print(f"in_wet rows: {s['in_wet_rows']}")
        print(f"human phases: {s['human_phase_counts']}")
        print(f"events: {s['event_counts']}")
        print()


if __name__ == "__main__":
    main()

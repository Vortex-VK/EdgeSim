from __future__ import annotations
from pathlib import Path
from typing import Dict, Any, List
import csv
import math

def _band_clearance(c: float) -> str:
    if c < 0.20: return "<0.20"
    if c < 0.50: return "0.20–0.50"
    return ">=0.50"

def _update_count(d: Dict[str,int], k: str, v: int=1) -> None:
    d[k] = d.get(k, 0) + v

def _last_min_clearance(csv_path: Path) -> float:
    """
    Scan file once, track minimum across rows.
    Prefer new fields: min_clearance_geom, then min_clearance_lidar,
    then fall back to legacy 'min_clearance' or column index 6.
    """
    mn = math.inf
    with csv_path.open("r", newline="", encoding="utf-8") as f:
        r = csv.reader(f)
        header = next(r, [])
        idx = None
        for name in ("min_clearance_geom", "min_clearance_lidar", "min_clearance"):
            if name in header:
                idx = header.index(name)
                break
        if idx is None:
            idx = 6  # legacy positional fallback
        for row in r:
            try:
                mn = min(mn, float(row[idx]))
            except Exception:
                pass
    return (mn if mn != math.inf else 1e9)

def build_coverage(per_run_dir: Path) -> Dict[str, Any]:
    """
    Walk per_run/*/run_one.csv and compute simple coverage counts:
      - traction: runs that ever entered wet
      - human_phase participation
      - clearance bands by run min
      - outcomes: success vs. collision_human vs. other
    """
    per_run_dir = Path(per_run_dir)
    runs = 0
    counts = {
        "traction": {"wet_encountered": 0, "dry_only": 0},
        "human_phase": {"none": 0, "running": 0, "fallen": 0},
        "clearance_bands": {"<0.20": 0, "0.20–0.50": 0, ">=0.50": 0},
        "outcomes": {"success": 0, "collision_human": 0, "other_failure": 0},
    }

    for run_folder in sorted(per_run_dir.glob("run_*")):
        csv_path = run_folder / "run_one.csv"
        if not csv_path.exists():
            continue
        runs += 1

        ever_wet = False
        ever_running = False
        ever_fallen = False
        outcome = "other"  # default, may be overridden by events
        min_clear = _last_min_clearance(csv_path)

        with csv_path.open("r", newline="", encoding="utf-8") as f:
            r = csv.reader(f)
            header = next(r, [])
            # figure out indices
            idx_event = header.index("event") if "event" in header else 8  # shifted with new header
            idx_in_wet = header.index("in_wet") if "in_wet" in header else None
            idx_hphase = header.index("human_phase") if "human_phase" in header else None

            for row in r:
                # traction
                if idx_in_wet is not None and idx_in_wet < len(row):
                    try:
                        if int(row[idx_in_wet]) == 1:
                            ever_wet = True
                    except Exception:
                        pass

                # human phase
                if idx_hphase is not None and idx_hphase < len(row):
                    hp = row[idx_hphase]
                    if hp == "running": ever_running = True
                    if hp == "fallen":  ever_fallen = True

                # outcomes from events
                ev = row[idx_event] if idx_event < len(row) else ""
                if ev == "collision_human":
                    outcome = "collision_human"
                elif ev == "success":
                    outcome = "success"
                elif ev == "other":
                    outcome = "other"

        # tally
        _update_count(counts["traction"], "wet_encountered" if ever_wet else "dry_only")
        if ever_fallen:
            _update_count(counts["human_phase"], "fallen")
        elif ever_running:
            _update_count(counts["human_phase"], "running")
        else:
            _update_count(counts["human_phase"], "none")
        _update_count(counts["clearance_bands"], _band_clearance(min_clear))
        if outcome in counts["outcomes"]:
            _update_count(counts["outcomes"], outcome)
        else:
            _update_count(counts["outcomes"], "other_failure")

    # percentages
    pct = lambda x: (x / runs * 100.0) if runs else 0.0
    summary = {
        "runs": runs,
        "traction_pct": {k: round(pct(v), 2) for k, v in counts["traction"].items()},
        "human_phase_pct": {k: round(pct(v), 2) for k, v in counts["human_phase"].items()},
        "clearance_bands_pct": {k: round(pct(v), 2) for k, v in counts["clearance_bands"].items()},
        "outcomes_pct": {k: round(pct(v), 2) for k, v in counts["outcomes"].items()},
        "counts": counts,
    }
    return summary

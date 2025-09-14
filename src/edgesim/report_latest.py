from __future__ import annotations
import os
from pathlib import Path
from typing import Optional
from datetime import datetime

from .report_html import generate_report

def _most_recent_batch(runs_root: Path) -> Optional[Path]:
    """
    Return the most recently modified batch folder under `runs/`.
    A "batch" is any directory directly under runs/ that contains a `per_run` subdir.
    """
    if not runs_root.exists():
        return None
    # collect candidates with mtime
    candidates = []
    for p in runs_root.iterdir():
        if not p.is_dir():
            continue
        if (p / "per_run").is_dir():
            try:
                mtime = p.stat().st_mtime
            except Exception:
                mtime = 0.0
            candidates.append((mtime, p))
    if not candidates:
        return None
    candidates.sort(key=lambda x: x[0], reverse=True)
    return candidates[0][1]

def main():
    """
    Usage:
        edgesim-report                     # build report for the most recent batch in runs/
        edgesim-report runs/<batch_dir>    # build report for a specific batch
    """
    import sys

    if len(sys.argv) == 2:
        # explicit batch dir
        batch_dir = Path(sys.argv[1]).resolve()
        if not batch_dir.exists():
            print(f"[ERR] Batch dir not found: {batch_dir}")
            raise SystemExit(2)
    else:
        # infer latest under runs/
        runs_root = Path("runs").resolve()
        batch_dir = _most_recent_batch(runs_root)
        if batch_dir is None:
            print(f"[ERR] No batches found under {runs_root}")
            raise SystemExit(2)

    out = generate_report(batch_dir)
    print(f"[OK] Wrote {out}")
    print(f"[INFO] Open this file in your browser: {out}")

if __name__ == "__main__":
    main()

from __future__ import annotations
from typing import Dict, Any, List
from pathlib import Path
import json
import time
import numpy as np

from .scenario import Scenario
from .sim_one import run_one  # your existing single-run
# sim_one writes per-run CSV already; here we orchestrate many runs.

def _mk_run_folder(root: Path, k: int) -> Path:
    d = root / f"run_{k:04d}"
    d.mkdir(parents=True, exist_ok=True)
    return d

def _seed_rng(master_seed: int, k: int) -> int:
    # per-run seed determinism (stable across machines)
    return int(np.random.SeedSequence([master_seed, k]).entropy % (2**31 - 1))

def run_batch(scn: Dict[str, Any], out_dir: Path, runs: int = 100, master_seed: int = 12345,
              profile: str = "minimal") -> Dict[str, Any]:
    out_dir.mkdir(parents=True, exist_ok=True)
    seeds_manifest: List[int] = []

    t0 = time.time()
    successes = 0
    collisions = 0
    times: List[float] = []
    steps: List[int] = []

    for k in range(runs):
        run_dir = _mk_run_folder(out_dir, k)
        seed_k = _seed_rng(master_seed, k)
        seeds_manifest.append(seed_k)

        # attach seed + profile to scenario
        scn_local = dict(scn)
        scn_local["profile"] = profile
        scn_local["seed"] = seed_k

        # single rollout
        res = run_one(prompt=scn_local.get("prompt",""),
                      scn=scn_local,
                      out_dir=run_dir,
                      gui=False, realtime=False)

        successes += int(bool(res.get("success", False)))
        # we can infer collisions by checking success==False and time < timeout, but you already log events in CSV
        times.append(float(res.get("time", 0.0)))
        steps.append(int(res.get("steps", 0)))

    t1 = time.time()
    summary = {
        "runs": runs,
        "successes": successes,
        "failures": runs - successes,
        "avg_time": (sum(times)/len(times)) if times else 0.0,
        "avg_steps": (sum(steps)/len(steps)) if steps else 0,
        "wallclock_s": round(t1 - t0, 3),
        "profile": profile,
    }

    # Write seeds + summary
    (out_dir / "seeds.json").write_text(json.dumps(seeds_manifest, indent=2))
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2))
    return summary

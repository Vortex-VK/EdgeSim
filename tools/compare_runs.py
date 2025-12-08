"""
Quick comparison tool for two EdgeSim run folders.

Usage:
  python tools/compare_runs.py /path/to/runA /path/to/runB

What it does:
  - Loads scenario.yaml (if PyYAML available) or falls back to raw text.
  - Loads manifest.json and seeds.json.
  - Flattens dicts and reports per-field differences.
  - Warns when expected files are missing.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Tuple


def _load_yaml(path: Path) -> Any:
    try:
        import yaml  # type: ignore
    except Exception:
        return path.read_text(encoding="utf-8")
    try:
        return yaml.safe_load(path.read_text(encoding="utf-8"))
    except Exception:
        return path.read_text(encoding="utf-8")


def _flatten(obj: Any, prefix: str = "") -> Dict[str, Any]:
    flat: Dict[str, Any] = {}
    if isinstance(obj, dict):
        for k, v in obj.items():
            key = f"{prefix}.{k}" if prefix else str(k)
            flat.update(_flatten(v, key))
    elif isinstance(obj, list):
        for idx, v in enumerate(obj):
            key = f"{prefix}[{idx}]" if prefix else f"[{idx}]"
            flat.update(_flatten(v, key))
    else:
        flat[prefix] = obj
    return flat


def _diff_maps(a: Dict[str, Any], b: Dict[str, Any]) -> List[Tuple[str, Any, Any]]:
    out: List[Tuple[str, Any, Any]] = []
    keys = set(a) | set(b)
    for k in sorted(keys):
        va = a.get(k, "<missing>")
        vb = b.get(k, "<missing>")
        if va != vb:
            out.append((k, va, vb))
    return out


def _load_run(run_dir: Path) -> Dict[str, Any]:
    data: Dict[str, Any] = {}
    scenario_path = run_dir / "scenario.yaml"
    manifest_path = run_dir / "manifest.json"
    seeds_path = run_dir / "seeds.json"
    if scenario_path.exists():
        data["scenario"] = _load_yaml(scenario_path)
    if manifest_path.exists():
        data["manifest"] = json.loads(manifest_path.read_text(encoding="utf-8"))
    if seeds_path.exists():
        data["seeds"] = json.loads(seeds_path.read_text(encoding="utf-8"))
    data["missing"] = [
        p.name
        for p in (scenario_path, manifest_path, seeds_path)
        if not p.exists()
    ]
    return data


def compare_runs(run_a: Path, run_b: Path) -> int:
    a = _load_run(run_a)
    b = _load_run(run_b)
    if a["missing"] or b["missing"]:
        print("Missing files:")
        if a["missing"]:
            print(f"  {run_a}: {a['missing']}")
        if b["missing"]:
            print(f"  {run_b}: {b['missing']}")
    # scenario
    if "scenario" in a and "scenario" in b:
        fa = _flatten(a["scenario"])
        fb = _flatten(b["scenario"])
        diff = _diff_maps(fa, fb)
        print(f"Scenario differences ({len(diff)}):")
        for k, va, vb in diff:
            print(f"  {k}: {va!r} -> {vb!r}")
    # manifest
    if "manifest" in a and "manifest" in b:
        fa = _flatten(a["manifest"])
        fb = _flatten(b["manifest"])
        diff = _diff_maps(fa, fb)
        print(f"Manifest differences ({len(diff)}):")
        for k, va, vb in diff:
            print(f"  {k}: {va!r} -> {vb!r}")
    # seeds
    if "seeds" in a and "seeds" in b:
        if a["seeds"] != b["seeds"]:
            print("Seeds differ:")
            print(f"  {run_a.name}: {a['seeds']}")
            print(f"  {run_b.name}: {b['seeds']}")
        else:
            print("Seeds identical.")
    return 0


def main(argv: List[str]) -> int:
    parser = argparse.ArgumentParser(description="Compare two EdgeSim run folders.")
    parser.add_argument("run_a", type=Path)
    parser.add_argument("run_b", type=Path)
    args = parser.parse_args(argv)
    return compare_runs(args.run_a, args.run_b)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

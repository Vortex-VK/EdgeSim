from __future__ import annotations
from pathlib import Path
import json
from .io_utils import write_text

def write_report(run_dir: Path, prompt: str, manifest: dict, coverage: dict, summary: dict) -> Path:
	md = []
	md.append("# EdgeSim – Coverage & Causality (V0 stub)")
	md.append("")
	md.append(f"**Prompt:** {prompt}")
	md.append("")
	md.append("## Batch Summary")
	md.append(f"- Runs: **{summary.get('runs', 0)}**")
	md.append(f"- Successes: **{summary.get('successes', 0)}** · Failures: **{summary.get('failures', 0)}**")
	md.append(f"- Avg rollout time: **{summary.get('avg_time', 0.0):.2f}s** · Avg steps: **{summary.get('avg_steps', 0)}**")
	md.append(f"- Wallclock: **{summary.get('wallclock_s', 0.0):.2f}s** · Profile: `{summary.get('profile','minimal')}`")
	md.append("")
	md.append("## Coverage (stub)")
	md.append("```json")
	md.append(json.dumps(coverage or {}, indent=2))
	md.append("```")
	md.append("")
	md.append("## Reproduce")
	md.append(f"- Version: `{manifest.get('version','?')}`  · Git: `{manifest.get('git','?')}`")
	md.append("- Re-run with the same `seeds.json` for identical results.")
	out = run_dir / "report.md"
	write_text(out, "\n".join(md))
	return out

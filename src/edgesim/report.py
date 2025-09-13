from __future__ import annotations
from pathlib import Path
import json

def write_markdown_report(batch_dir: Path, title: str = "EdgeSim – Coverage & Causality (V0 stub)") -> Path:
    s = json.loads((batch_dir / "summary.json").read_text())
    md = []
    md.append(f"# {title}")
    md.append("")
    md.append(f"- **Runs**: {s['runs']}")
    md.append(f"- **Successes**: {s['successes']}  • **Failures**: {s['failures']}")
    md.append(f"- **Avg rollout time**: {s['avg_time']:.2f}s  • **Avg steps**: {s['avg_steps']}")
    md.append(f"- **Batch wallclock**: {s['wallclock_s']:.2f}s  • **Profile**: `{s['profile']}`")
    md.append("")
    md.append("> V0 stub report. Next: taxonomy coverage, factor correlations, and failing frame thumbnails.")
    out = batch_dir / "report.md"
    out.write_text("\n".join(md))
    return out

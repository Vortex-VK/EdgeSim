#!/usr/bin/env python3
"""Basic clarity/content lint for demo copy.

This is intentionally simple: it catches placeholder language and verifies
that key explanatory terms exist for first-time viewers.
"""

from __future__ import annotations

from pathlib import Path
import sys

APP_PATH = Path(__file__).resolve().parents[1] / "src" / "app" / "App.tsx"

FORBIDDEN = [
    "coming soon",
    "lorem ipsum",
    "todo",
    "tbd",
    "placeholder",
    "alert(",
]

REQUIRED_HINTS = [
    "single test run",
    "100-run batch",
    "scenario explorer",
    "artifacts",
    "not a motion planner",
]


def main() -> int:
    if not APP_PATH.exists():
        print(f"FAIL: missing {APP_PATH}")
        return 1

    content = APP_PATH.read_text(encoding="utf-8").lower()
    failures: list[str] = []

    for phrase in FORBIDDEN:
        if phrase in content:
            failures.append(f"forbidden phrase found: '{phrase}'")

    for phrase in REQUIRED_HINTS:
        if phrase not in content:
            failures.append(f"required explanatory phrase missing: '{phrase}'")

    if failures:
        print("FAIL: demo text check")
        for issue in failures:
            print(f"- {issue}")
        return 1

    print("PASS: demo text check")
    print(f"Checked file: {APP_PATH}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

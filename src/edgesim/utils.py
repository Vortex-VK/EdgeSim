## src/edgesim/utils.py
from __future__ import annotations
import subprocess
import datetime as _dt
import re


def timestamp_slug() -> str:
# e.g., 2025-09-12_20-00-00
    return _dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")


def git_hash_fallback() -> str:
    try:
        return (
        subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], stderr=subprocess.DEVNULL)
        .decode()
        .strip()
        )
    except Exception:
        return "nogit"


def slugify(text: str, max_len: int = 40) -> str:
    text = re.sub(r"[^a-zA-Z0-9\-\_\s]", "", text)
    text = re.sub(r"\s+", " ", text).strip()
    text = text.replace(" ", "_")
    return text[:max_len] or "run"

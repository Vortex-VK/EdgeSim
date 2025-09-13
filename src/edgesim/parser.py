from __future__ import annotations
import re
from typing import Any, Dict

from .schema import new_scenario

# Simple keyword map for V0 rule-based parsing
KEYWORDS = {
	"traction": ["wet", "slippery", "spill", "oil"],
	"visibility": ["night", "dim", "dark", "low light", "reflective"],
	"human": ["human", "picker", "worker", "crossing"],
	"traffic": ["high pallet traffic", "rush", "busy", "heavy traffic"],
	"overhang": ["overhang", "irregular load"],
}

def _has_any(text: str, words: list[str]) -> bool:
	text_l = text.lower()
	return any(w in text_l for w in words)

def parse_numbers(text: str) -> Dict[str, float]:
	"""
	Tiny extractor for a few knobs:
	- friction like "mu 0.35" or "friction 0.35"
	- duration like "time limit 200" / "200s"
	- human period like "every 30s" -> rate_per_min = 2.0
	"""
	text_l = text.lower()
	out: Dict[str, float] = {}

	# friction
	m = re.search(r"(mu|friction)\s*([:=]?\s*)?(?P<val>0\.[0-9]+)", text_l)
	if m:
		out["mu"] = float(m.group("val"))

	# duration seconds
	m = re.search(r"(duration|time limit|limit)\s*([:=]?\s*)?(?P<sec>\d{2,4})\s*s?", text_l)
	if m:
		out["duration_s"] = float(m.group("sec"))

	# human crossing period like "every 30s"
	m = re.search(r"every\s*(?P<p>\d{1,4})\s*s", text_l)
	if m:
		period_s = float(m.group("p"))
		if period_s > 0:
			out["human_rate_per_min"] = 60.0 / period_s

	return out

def prompt_to_scenario(prompt: str, n_runs: int = 100) -> Dict[str, Any]:
	scn = new_scenario(n_runs)
	text = prompt.lower()

	# visibility
	if _has_any(text, KEYWORDS["visibility"]):
		scn["sensors"]["lighting_dim"] = True
		scn["taxonomy"]["visibility"] = True

	# traction (wet patch with default mu or parsed mu)
	if _has_any(text, KEYWORDS["traction"]):
		mu = parse_numbers(prompt).get("mu", 0.45)
		# simple 2x3m patch near "dock" (heuristic)
		patch = {"zone": [12.0, 2.0, 14.0, 5.0], "mu": float(mu)}
		scn["hazards"]["traction"].append(patch)
		scn["taxonomy"]["traction"] = True

	# human crossing
	if _has_any(text, KEYWORDS["human"]):
		rate = parse_numbers(prompt).get("human_rate_per_min", 2.0)  # every 30s
		scn["hazards"]["human"].append({
			"path": "line",
			"rate_per_min": float(rate),
			"speed_mps": [0.8, 1.4],
		})
		scn["taxonomy"]["human_behavior"] = True

	# traffic → clutter density
	if _has_any(text, KEYWORDS["traffic"]):
		scn["hazards"]["clutter"]["aisle_density"] = 0.25

	# overhang
	if _has_any(text, KEYWORDS["overhang"]):
		scn["hazards"]["clutter"]["overhang_prob"] = 0.3

	# duration override if present
	nums = parse_numbers(prompt)
	if "duration_s" in nums:
		scn["runtime"]["duration_s"] = int(nums["duration_s"])  # type: ignore

	return scn

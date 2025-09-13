from __future__ import annotations
from dataclasses import dataclass, field, asdict
from typing import Any, Dict, List, Tuple
import json
import re

# ---------- Schema & Defaults ----------

@dataclass
class RuntimeCfg:
    duration_s: float = 90.0
    dt: float = 0.05

@dataclass
class LayoutCfg:
    map_size_m: Tuple[float, float] = (20.0, 12.0)   # Lx, Ly
    start: Tuple[float, float] = (2.0, 2.0)
    goal: Tuple[float, float] = (18.0, 10.0)
    walls: List[Dict[str, Any]] = field(default_factory=list)

@dataclass
class AgentCfg:
    name: str = "amr_0"
    radius_m: float = 0.4

@dataclass
class HazardHumanCfg:
    duration_s: float = 6.0            # normal walk
    duration_s_running: float = 2.5    # running (late)

@dataclass
class HazardTractionPatch:
    zone: Tuple[float, float, float, float]  # x0,y0,x1,y1
    mu: float = 0.45

@dataclass
class HazardsCfg:
    human: List[HazardHumanCfg] = field(default_factory=lambda: [HazardHumanCfg()])
    traction: List[HazardTractionPatch] = field(default_factory=list)

@dataclass
class Scenario:
    prompt: str
    runtime: RuntimeCfg = field(default_factory=RuntimeCfg)
    layout: LayoutCfg = field(default_factory=LayoutCfg)
    agents: List[AgentCfg] = field(default_factory=lambda: [AgentCfg()])
    hazards: HazardsCfg = field(default_factory=HazardsCfg)
    profile: str = "minimal"  # minimal|robot|full

    def to_dict(self) -> Dict[str, Any]:
        def _cv(o):
            if hasattr(o, "__dict__"):
                return {k: _cv(v) for k, v in o.__dict__.items()}
            if isinstance(o, (list, tuple)):
                return [ _cv(x) for x in o ]
            return o
        return _cv(self)

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=2)

# ---------- Very small rule-based English → Scenario ----------

_WET_RE = re.compile(r"(wet|slippery)\s+(floor|zone|patch)(?:\s+near\s+(?:x\s*=\s*(\d+(?:\.\d+)?)|y\s*=\s*(\d+(?:\.\d+)?)))?", re.I)
_HUMAN_RE = re.compile(r"(human|pedestrian)\s+(cross|crossing|runner|running)", re.I)
_SIZE_RE = re.compile(r"(map|area)\s*(?:size)?\s*(\d+(?:\.\d+)?)\s*[x×]\s*(\d+(?:\.\d+)?)\s*m", re.I)
_START_RE = re.compile(r"start\s*\((\d+(?:\.\d+)?),\s*(\d+(?:\.\d+)?)\)", re.I)
_GOAL_RE  = re.compile(r"goal\s*\((\d+(?:\.\d+)?),\s*(\d+(?:\.\d+)?)\)", re.I)

def parse_prompt_to_scenario(prompt: str) -> Scenario:
    scn = Scenario(prompt=prompt)

    # map size
    m = _SIZE_RE.search(prompt)
    if m:
        Lx = float(m.group(2)); Ly = float(m.group(3))
        scn.layout.map_size_m = (Lx, Ly)

    # start/goal
    m = _START_RE.search(prompt)
    if m:
        scn.layout.start = (float(m.group(1)), float(m.group(2)))
    m = _GOAL_RE.search(prompt)
    if m:
        scn.layout.goal = (float(m.group(1)), float(m.group(2)))

    # wet patch (default: centered band across Y midline)
    if _WET_RE.search(prompt):
        Lx, Ly = scn.layout.map_size_m
        band_w = 3.0
        band_h = 1.2
        cx = Lx * 0.5
        y0 = max(0.6, Ly * 0.5 - band_h/2)
        y1 = min(Ly - 0.6, Ly * 0.5 + band_h/2)
        scn.hazards.traction.append(HazardTractionPatch(
            zone=(cx - band_w/2, y0, cx + band_w/2, y1),
            mu=0.35
        ))

    # human crosser / runner
    if _HUMAN_RE.search(prompt):
        scn.hazards.human = [HazardHumanCfg()]  # defaults ok

    # profile flag
    if "robot profile" in prompt.lower():
        scn.profile = "robot"
    elif "full profile" in prompt.lower():
        scn.profile = "full"

    return scn

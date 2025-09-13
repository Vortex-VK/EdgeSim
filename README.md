## README.md
# EdgeSim (V0) — Step 1


**Goal (Step 1):** minimal CLI that converts an English prompt into a structured **Scenario Graph** and writes a timestamped results folder with:
- `scenario.yaml` (final scenario),
- `seeds.json` (batch seeds),
- `manifest.json` (run metadata),
- a tiny `README.md` in the run folder.


Physics, planning, rendering, metrics, etc. come in later steps.


## Quick start
# Create venv (recommended)
python -m venv .venv && source .venv/bin/activate # (Linux/macOS)
# On Windows: .venv\\Scripts\\activate


# Option A: install as package
pip install -e .


# Option B: just install deps
pip install -r requirements.txt


# Run CLI (package script)
edgesim simulate "Night shift, wet floor near loading dock; heavy pallet traffic; reflective tape; occasional human crossing."


# Or directly via module
python -m edgesim.cli simulate "Night shift near loading dock; wet floor patch; occasional human crossing"


## Outputs
runs/2025-09-12_20-00-00_NightShiftWetFloor/
├─ scenario.yaml
├─ seeds.json
├─ manifest.json
└─ RUN_README.md
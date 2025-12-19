# EdgeSim

EdgeSim turns natural-language warehouse prompts into reproducible PyBullet rollouts with logs you can replay, analyze, and report on. It is an environment (geometry, actors, sensors), not a planner.

## Highlights
- Prompt-to-scenario graph in `parser.py` (aisles, racks/walls, humans, vehicles, traction, events) with taxonomy tags and site overrides.
- PyBullet world builder (`world.py`) plus single-rollout executor (`sim_one.py`): traction/wet zones, occlusions, blind corners, forklifts/carts, human behaviors, slip/falling/thrown-object injectors.
- Sensors and effects: LiDAR noise/dropout/fog/reflections, IMU/odom bias/noise, slip boosts, reflective surfaces, traction-aware slip risk.
- GUI or headless runs, debug overlays, deterministic seeds; outputs under `runs/`.
- Batch runner with coverage metrics, dataset manifest, digest verification, auto-degrade for time budgets, HTML/Markdown reporting, Matplotlib replay/export.
- Optional FastAPI job wrapper (`backend/`) and Vite frontend scaffold (`frontend/`).

## Repository layout
- `src/edgesim/cli.py` – entrypoint `edgesim` (simulate, run-one, run-batch, replay, verify, calibrate, assert-stress).
- `src/edgesim/parser.py` – prompt parsing to scenario graph.
- `src/edgesim/world.py`, `src/edgesim/sim_one.py` – world construction and single rollout; `viewer.py` – 2D replay/MP4 export.
- `src/edgesim/report_html.py`, `reporter.py`, `metrics.py`, `coverage.py`, `dataset_manifest.py` – reporting, coverage, summaries.
- `site_profiles/` (tunable site JSONs), `data/anchors.csv` (calibration anchors), `runs/` (output root).
- `tools/` (data quality checks: `check_data_quality.py`, `summarize_runs.py`, `compare_runs.py`).
- `backend/` (FastAPI launcher), `frontend/` (optional UI; see `frontend/README_frontend.md`).

## Install
- Requires Python 3.10+ and system OpenGL for PyBullet GUI.
```bash
python -m venv .venv
source .venv/bin/activate  # Windows: .venv\Scripts\activate
pip install -e .
# optional API/frontend extras (FastAPI, uvicorn, etc.)
pip install -r requirements.txt
```

## Quick start
- Parse a prompt without running physics:
```bash
edgesim simulate "busy aisle with forklift from 4,3 to 4,17 and wet patch from 8,8 to 12,10"
```
- Run one rollout with GUI overlays and slow motion:
```bash
edgesim run-one --gui --realtime --slowmo 5 --debug-visuals \
  "group of workers crossing 10,12 to 10,3, forklift from 4,3 to 4,17, wet patch 8,8 to 12,10"
```
- Batch with reproducible seeds and report artifacts:
```bash
edgesim run-batch "two blind corners and reversing forklift" \
  --runs 50 --seed 7 --profile full --site EdgeCases --lidar-events-only
```
- Replay or export MP4 (Matplotlib viewer):
```bash
edgesim replay runs/<batch>/per_run/run_0000 --viewer --follow-robot --export-mp4 out.mp4
```
- Build the latest HTML report:
```bash
edgesim-report            # or: edgesim-report runs/<batch_dir>
```

## CLI reference
- `simulate` – prompt -> `scenario.yaml`, `seeds.json`, `manifest.json` (no physics).
- `run-one` – single rollout; flags: `--gui`, `--realtime`, `--slowmo N`, `--dt`, `--debug-visuals`, `--lidar-logging` or `--lidar-events-only`, `--site` for site_profiles/<site>.json.
- `run-batch` – seeded batch; `--profile {minimal,robot,full}`, `--site`, `--time-budget-min` + `--auto-degrade` to shorten runs/drop LiDAR Hz when over budget, LiDAR logging flags.
- `replay` – open viewer or export MP4; accepts per-run folder or batch root plus `--run-index`.
- `verify` – check stored digests (`manifest.json`) against current CSV/world files.
- `calibrate` – compute metrics from anchors CSV and write `site_profiles/<site>.json`; optionally mirror into a batch dir.
- `assert-stress` – gate batches by coverage/outcome thresholds (fails CI with exit code 2).
- `edgesim-report` – build HTML report for a batch.

## Outputs
Each `edgesim run-one` folder (or each `per_run/run_xxxx` in a batch) contains:
- `scenario.yaml`, `seeds.json`, `manifest.json` with digests.
- `world.json` + `frames.json` (geometry and frame metadata).
- `run_one.csv` (time series), `actors.csv` (optional), `lidar.npz` (if logging enabled).
- Digests mirrored to batch root for reproducibility.
Batch-level `run-batch` also writes `per_run/index.csv`, `summary.json`, `perf.json`, `coverage.json`, `dataset_manifest.json` (if available), `report.md` and `report.html` when reporter modules are installed.

## Prompting tips
- Prefer explicit coordinates for aisles, racks/walls, humans, vehicles, and traction zones.
- Chain multiple segments with "and ...", e.g. `high rack from 2,12 to 9,12 and 11,12 to 18,12`.
- Use intent words to trigger behaviors: reversing forklifts, wet/slippery/cleaning liquid, blind corners, groups of workers, etc.
- EdgeSim builds the environment and a simple PID drive to the goal; it does not plan around obstacles for you.

## Site profiles and calibration
- Set `EDGESIM_SITE=<slug>` or pass `--site <slug>` to load `site_profiles/<slug>.json` (tunes traction, noise, human slip, etc.).
- Generate a profile from anchors: `edgesim calibrate --anchors data/anchors.csv --site MySite --run runs/<batch_dir>`.

## API / integration
- FastAPI wrapper lives in `backend/main.py`. Run with:
```bash
uvicorn backend.main:app --reload --host 0.0.0.0 --port 8000
```
  - `GET /health`
  - `POST /api/simulate` with `{mode, command, config}` to launch CLI jobs; logs land in `runs/_job_logs`.
- The `frontend/` folder contains a Vite-based UI scaffold.

## Development
- Run tests: `pytest`.
- Quick syntax check: `python -m py_compile src/edgesim/*.py src/edgesim/**/*.py`.
- Data quality tools: `python tools/check_data_quality.py runs/<run_dir>` or `python tests/validate_batch.py runs/<batch_dir>`.
- Viewer deps: Matplotlib and ffmpeg for MP4 export (included in project deps).

EdgeSim overview
- EdgeSim converts natural-language warehouse prompts into structured scenario graphs, builds a PyBullet world from them, and runs deterministic simulations that can be replayed or reported. It focuses on environment fidelity (geometry, actors, sensors) rather than motion planning; the robot follows a simple lane-aligned PID drive toward a goal.

End-to-end workflow
- The CLI accepts prompts via simulate, run-one, run-batch, and replay verbs. simulate parses the prompt and writes scenario.yaml, seeds.json, manifest.json, and validation.json (if schema validation succeeds). run-one executes one rollout with optional GUI, realtime slow motion, and LiDAR logging; run-batch executes seeded batches with profiles and optional auto-degrade to stay within a time budget. replay opens a viewer or exports MP4s from finished runs.
- Runs live under runs/<timestamp>_<slug>/, with per_run/run_XXXX/ folders for batches. Each run captures scenario.yaml, seeds.json, manifest.json (including digests), world.json, frames.json, run_one.csv (time series with pose, velocities, TTC, clearance, events), optional actors.csv and lidar.npz, and auxiliary logs. Batch-level roots also contain per_run/index.csv, summary.json, perf.json, coverage.json, dataset_manifest.json, report.md, and report.html when enabled.

Prompt parsing and scenario graph (src/edgesim/parser.py and schema.py)
- Prompts are matched against keyword sets and regexes to pull coordinates for aisles, racks, walls, patches, vehicles, humans, and static obstacles. The parser seeds a scenario graph with map bounds, start/goal, and taxonomy flags that drive coverage.
- It provides presets for common geometries (main aisle with crosswalk, T-intersection, forklift aisles, parallel aisles) and can carve racking bands, walls, junctions, blind corners, endcaps, and open storage zones. Layout helpers clamp objects to bounds, split racks around aisles, and ensure start/goal points fall on walkable space.
- Hazard generation covers traction patches (wet, oil, cleaning liquid), floor transition zones, static clutter, industrial vehicles (forklifts, pallet jacks, tuggers) with paths and reversing behaviors, and human actors with waypoints, group sizing, occlusion-aware starts, and behavior presets (start/stop, hesitation, emerge from occlusion, close pass, late reactions). It also sets noise/visibility traits (reflective surfaces, dark clothing, sensor faults) and optional site overrides.

World building (src/edgesim/world.py)
- Builds a PyBullet scene from the scenario graph: floor surfaces with friction and slip/IMU effects, aisles/junctions rendered as walkable patches, racks, walls, columns, endcaps, static obstacles, and traction or cleaning zones with tuned friction. Geometry is normalized and clamped for stability, with deterministic randomization based on seeds.
- Spawns vehicles with color/material hints, occlusion geometry for loads or reversing, and optional map overlays for racking and traction. Provides helper routines for oriented rectangles, overlap checks, and seeded RNG.

Simulation loop (src/edgesim/sim_one.py)
- Spawns a disc robot with configurable radius/height, positions it inside aisles or projected to the nearest walkway, and drives it with a PID controller along a straight start-to-goal path. Runtime parameters include dt, duration, GUI/realtime toggles, and slow-motion scaling.
- Applies site tuning and runtime overrides (traction mu ranges, slip boosts, LiDAR dropout/noise, IMU/odom bias). Adds optional auto wet corridors or local wet patches, and enforces clearance from static geometry.
- Human actors: spawned from hazard configs with randomized paths, group offsets, slip probabilities tied to traction, posture changes (upright vs fallen), prop handling (carts, payload boxes), fall/slide distances, and behaviors like late reaction triggers. Vehicles follow segmented paths with loop or ping-pong, reversing duty cycles, alarms, rear occlusion cones, and load overhangs.
- Injectors (src/edgesim/injectors.py) can add LiDAR sector blackouts, ghost obstacles, and falling objects. Floor events spawn spills dynamically. Safety zones and occlusion blockers influence near-miss checks.
- Sensing/logging: LiDAR rays include noise/dropout and can log full scans or event windows; odom/IMU biases are applied; near-miss, collision, slip, occluded hazard, and success events are recorded. run_one.csv logs per-step state (pose, velocities, contact flags, clearances, TTC, event fields), and optional lidar.npz stores scan data. World geometry and frames metadata are written to world.json and frames.json; world_digest.json captures a hashable summary.

Batch runner and policies (src/edgesim/rollout.py)
- Seeds a run per requested seed into per_run/run_XXXX folders, writes per_run/index.csv, and aggregates successes, average time, and steps. Optional auto-degrade enforces wall-clock budgets by shortening runtime and halving LiDAR rate if needed; perf.json records the policy, wall time, sims per minute, and estimated cost per simulated minute.

Metrics, coverage, and gating (src/edgesim/metrics.py and coverage.py)
- aggregate computes batch summaries from per_run/index.csv; write_summary stores summary.json.
- build_coverage scans per-run CSVs to count traction encounters, human phase participation, clearance bands, and outcomes; returns both counts and percentages for coverage.json.
- assert-stress compares coverage/outcome percentages against CLI thresholds (e.g., minimum fallen humans, wet encounters, tight clearances, collisions, maximum success) and returns exit code 2 on failure for CI gating.

Dataset manifest (src/edgesim/dataset_manifest.py)
- Reads run_one.csv across runs to summarize durations, speeds, minimum TTC, and event totals (collision types, slips, near-miss, occluded hazard). Pulls randomization knobs from world.json (traction coefficients) and scenario (LiDAR noise/dropout, clutter level, behavior presets). Writes dataset_manifest.json with per-run stats, aggregates, taxonomy tags, and randomization hints.

Reporting (src/edgesim/report_html.py and reporter.py)
- Generates a styled HTML report with scenario metadata, taxonomy tags, runtime/seeding info, coverage-style metrics (success, collisions, clearance, min TTC), digest hashes for reproducibility, and SVG maps showing aisles, traction patches, racks, and robot trajectory with event markers. Includes replay links/prompts and representative run selection. reporter.py writes a Markdown counterpart when available. edgesim-report builds the latest report for a batch directory.

Calibration and site profiles (src/edgesim/metrics.py, dataset_manifest.py, site_profiles/)
- calibrate consumes an anchors CSV (R², KL, ECE tables) to compute metrics and writes site_profiles/<site>.json plus an optional calibration.json inside a batch. Heuristic tuning adjusts LiDAR noise/dropout, odom bias, traction coefficients, human slip probability, and risk temperature based on those metrics. EDGESIM_SITE or CLI flags load a profile at runtime; site_overrides in scenarios can tweak per-run parameters.

Digest verification (cli.py)
- verify recomputes SHA-256 digests for per-run CSVs and world.json, compares them to manifest.json entries, and reports PASS/FAIL for reproducibility checks.

Backend and frontend
- backend/main.py exposes FastAPI endpoints: /health and /api/simulate, which launches arbitrary edgesim CLI commands as subprocesses, tracks job IDs, and logs output under runs/_job_logs. CORS is open for UI access.
- frontend/ is a Vite + React scaffold (Tailwind v4 config) intended to call the backend, offering text/visual prompt modes; proxy defaults to localhost:8000.

Tools, tests, and data
- tools/ contains data quality utilities such as check_data_quality.py, summarize_runs.py, compare_runs.py. tests/ houses pytest suites and batch validators. data/ anchors.csv is a sample calibration input. site_profiles/ includes example site tunings. todo/ tracks pending improvements.

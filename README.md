# EdgeSim

EdgeSim turns natural-language scene prompts into reproducible warehouse simulation runs. The CLI builds a structured scenario (layout, hazards, agents), runs Bullet-based simulation, and writes artifacts you can inspect, replay, and analyze.

## Features
- Prompt-to-scenario parser (aisles, racks/walls, humans, vehicles, traction patches, runtime events).
- Deterministic seeds per run; artifacts saved under `runs/<timestamp>`.
- GUI and headless modes; realtime/slowmo toggles; debug visuals for geometry.
- Sensor effects: lidar noise/dropout, reflections, occlusion, wet-floor slip events, reversing vehicles, human behaviors.

## Install
```bash
python -m venv .venv && source .venv/bin/activate  # Windows: .venv\Scripts\activate
pip install -e .
```

## Usage
Generate and run one scenario from a prompt:
```bash
edgesim run-one --gui --realtime --slowmo 5 --debug-visuals \
  "busy aisle with forklift from 4,3 to 4,17, human crossing from 10,12 to 10,3, wet patch from 8,8 to 12,10"
```
Key flags:
- `--gui` / `--realtime` / `--slowmo N` for visualization pacing.
- `--debug-visuals` to show aisles, racks, patches, and spawn zones.
- `--no-gui` for headless runs.

Outputs land in `runs/<timestamp>_<slug>/`:
- `scenario.yaml` (final scenario graph)
- `manifest.json` (metadata)
- `seeds.json` (random seeds used)
- `RUN_README.md` (per-run summary)

## Tips
- Provide explicit coordinates to control aisles, racks/walls, humans, vehicles, and traction zones.
- Use phrases like “reversing forklift …” to get reversing behavior on coordinate-defined forklifts.
- For multiple segments of the same type, chain with “and …”, e.g. `high rack from 2,12 to 9,12 and 11,12 to 18,12`.

## Dev
- Code lives under `src/edgesim`. Entry point CLI: `src/edgesim/cli.py`. Parser: `src/edgesim/parser.py`. Simulator: `src/edgesim/sim_one.py`.
- Tests: `pytest`.
- Run a quick syntax check: `python -m py_compile src/edgesim/*.py src/edgesim/**/*.py`.

## License
MIT

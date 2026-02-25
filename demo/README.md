# EdgeSim demo site

This app is a public-facing walkthrough of EdgeSim using premade runs only.

## What it shows

- 4 prompt scenarios
- 1 test run + 1 batch run (100 seeds) per scenario
- Batch coverage/outcome metrics
- Test trajectory + event timeline map
- Direct links to raw artifacts (CSV/JSON/HTML reports)

## Data source

Run data is generated from the repository `runs/` directory and written to:

- `src/app/data/demoData.json`
- `public/artifacts/...`

## Commands

From `demo/`:

```bash
npm run sync-data
npm run text-check
npm run dev
npm run build
```

`sync-data` must be run any time the source runs change.

`sync-data` works even if `PyYAML` is not installed (it automatically falls back to `world.json`).

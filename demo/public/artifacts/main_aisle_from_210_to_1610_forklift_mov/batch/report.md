# EdgeSim – Coverage & Causality (V0 stub)

**Prompt:** main aisle from 2,10 to 16,10, forklift moving from 3,13 to 14,13, obstacle at 16,13, cart block at 13.5,8.5, rack from 1,6 to 15,6, rack from 18.5,4.5 to 18.5,17.5, rack from 1,16 to 15,16, wall from 1,6 to 1,16, falling object at 16,9, human crossing from 16,0.5 to 16,8.5

## Batch Summary
- Runs: **1000**
- Successes: **1000** · Failures: **0**
- Avg rollout time: **8.10s** · Avg steps: **83**
- Wallclock: **0.00s** · Profile: `minimal`

## Coverage (stub)
```json
{
  "runs": 1000,
  "traction_pct": {
    "wet_encountered": 0.0,
    "dry_only": 100.0
  },
  "human_phase_pct": {
    "none": 0.0,
    "running": 100.0,
    "fallen": 0.0
  },
  "clearance_bands_pct": {
    "<0.20": 0.0,
    "0.20\u20130.50": 0.0,
    ">=0.50": 100.0
  },
  "outcomes_pct": {
    "success": 100.0,
    "collision_human": 0.0,
    "other_failure": 0.0
  },
  "counts": {
    "traction": {
      "wet_encountered": 0,
      "dry_only": 1000
    },
    "human_phase": {
      "none": 0,
      "running": 1000,
      "fallen": 0
    },
    "clearance_bands": {
      "<0.20": 0,
      "0.20\u20130.50": 0,
      ">=0.50": 1000
    },
    "outcomes": {
      "success": 1000,
      "collision_human": 0,
      "other_failure": 0
    }
  }
}
```

## Reproduce
- Version: `0.0.1`  · Git: `9dbc433`
- Re-run with the same `seeds.json` for identical results.
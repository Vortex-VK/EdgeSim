# EdgeSim – Coverage & Causality (V0 stub)

**Prompt:** main aisle from 2,10 to 16.5,10, human crossing from 10.5,6 to 10.5,14, rack from 1,7.5 to 9.5,7.5, rack from 11.5,7.5 to 18,7.5, rack from 1,12.5 to 9.5,12.5, rack from 11.5,12.5 to 18,12.5, wet patch from 10,9 to 11,11, wall from 1,11.5 to 1,8.5, wall from 18,11 to 18,9

## Batch Summary
- Runs: **10000**
- Successes: **2539** · Failures: **7461**
- Avg rollout time: **5.42s** · Avg steps: **56**
- Wallclock: **0.00s** · Profile: `minimal`

## Coverage (stub)
```json
{
  "runs": 10000,
  "traction_pct": {
    "wet_encountered": 100.0,
    "dry_only": 0.0
  },
  "human_phase_pct": {
    "none": 0.0,
    "running": 29.16,
    "fallen": 70.84
  },
  "clearance_bands_pct": {
    "<0.20": 1.07,
    "0.20\u20130.50": 78.74,
    ">=0.50": 20.19
  },
  "outcomes_pct": {
    "success": 25.39,
    "collision_human": 74.61,
    "other_failure": 0.0
  },
  "counts": {
    "traction": {
      "wet_encountered": 10000,
      "dry_only": 0
    },
    "human_phase": {
      "none": 0,
      "running": 2916,
      "fallen": 7084
    },
    "clearance_bands": {
      "<0.20": 107,
      "0.20\u20130.50": 7874,
      ">=0.50": 2019
    },
    "outcomes": {
      "success": 2539,
      "collision_human": 7461,
      "other_failure": 0
    }
  }
}
```

## Reproduce
- Version: `0.0.1`  · Git: `9dbc433`
- Re-run with the same `seeds.json` for identical results.
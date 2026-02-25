# EdgeSim – Coverage & Causality (V0 stub)

**Prompt:** main aisle from 2,10 to 18,10, wall from 2,8 to 18,8, human crossing from 2.5,12 to 17.5,12, tugger from 6.5,16 to 18,9, wet patch from 7,11.5 to 11,12.5

## Batch Summary
- Runs: **100**
- Successes: **100** · Failures: **0**
- Avg rollout time: **9.20s** · Avg steps: **94**
- Wallclock: **0.00s** · Profile: `minimal`

## Coverage (stub)
```json
{
  "runs": 100,
  "traction_pct": {
    "wet_encountered": 92.0,
    "dry_only": 8.0
  },
  "human_phase_pct": {
    "none": 0.0,
    "running": 6.0,
    "fallen": 94.0
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
      "wet_encountered": 92,
      "dry_only": 8
    },
    "human_phase": {
      "none": 0,
      "running": 6,
      "fallen": 94
    },
    "clearance_bands": {
      "<0.20": 0,
      "0.20\u20130.50": 0,
      ">=0.50": 100
    },
    "outcomes": {
      "success": 100,
      "collision_human": 0,
      "other_failure": 0
    }
  }
}
```

## Reproduce
- Version: `0.0.1`  · Git: `ac2f46c`
- Re-run with the same `seeds.json` for identical results.
# EdgeSim – Coverage & Causality (V0 stub)

**Prompt:** main aisle from 2,10 to 18,10, rack from 0.5,7.5 to 12.5,7.5, wall from 0.5,11 to 19.5,11, rack from 19.5,7.5 to 14.5,7.5, wet patch from 13,8 to 14,10.5, human crossing from 13.5,6.5 to 13.5,10, wall from 0.5,11 to 0.5,9, wall from 19.5,11 to 19.5,9

## Batch Summary
- Runs: **1000**
- Successes: **438** · Failures: **562**
- Avg rollout time: **7.42s** · Avg steps: **76**
- Wallclock: **0.00s** · Profile: `minimal`

## Coverage (stub)
```json
{
  "runs": 1000,
  "traction_pct": {
    "wet_encountered": 100.0,
    "dry_only": 0.0
  },
  "human_phase_pct": {
    "none": 0.0,
    "running": 24.8,
    "fallen": 75.2
  },
  "clearance_bands_pct": {
    "<0.20": 0.0,
    "0.20\u20130.50": 60.9,
    ">=0.50": 39.1
  },
  "outcomes_pct": {
    "success": 43.8,
    "collision_human": 56.2,
    "other_failure": 0.0
  },
  "counts": {
    "traction": {
      "wet_encountered": 1000,
      "dry_only": 0
    },
    "human_phase": {
      "none": 0,
      "running": 248,
      "fallen": 752
    },
    "clearance_bands": {
      "<0.20": 0,
      "0.20\u20130.50": 609,
      ">=0.50": 391
    },
    "outcomes": {
      "success": 438,
      "collision_human": 562,
      "other_failure": 0
    }
  }
}
```

## Reproduce
- Version: `0.0.1`  · Git: `06390f3`
- Re-run with the same `seeds.json` for identical results.
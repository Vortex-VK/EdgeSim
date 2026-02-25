# EdgeSim – Coverage & Causality (V0 stub)

**Prompt:** main aisle from 2,10 to 14.5,10, rack from 17,16 to 17,4.5, rack from 1.5,7 to 13,7, rack from 1.5,13 to 13,13, human crossing from 14,5.5 to 14,15.5, wet patch from 13.5,9 to 15,11

## Batch Summary
- Runs: **100**
- Successes: **31** · Failures: **69**
- Avg rollout time: **6.75s** · Avg steps: **69**
- Wallclock: **0.00s** · Profile: `minimal`

## Coverage (stub)
```json
{
  "runs": 100,
  "traction_pct": {
    "wet_encountered": 100.0,
    "dry_only": 0.0
  },
  "human_phase_pct": {
    "none": 0.0,
    "running": 31.0,
    "fallen": 69.0
  },
  "clearance_bands_pct": {
    "<0.20": 0.0,
    "0.20\u20130.50": 69.0,
    ">=0.50": 31.0
  },
  "outcomes_pct": {
    "success": 31.0,
    "collision_human": 69.0,
    "other_failure": 0.0
  },
  "counts": {
    "traction": {
      "wet_encountered": 100,
      "dry_only": 0
    },
    "human_phase": {
      "none": 0,
      "running": 31,
      "fallen": 69
    },
    "clearance_bands": {
      "<0.20": 0,
      "0.20\u20130.50": 69,
      ">=0.50": 31
    },
    "outcomes": {
      "success": 31,
      "collision_human": 69,
      "other_failure": 0
    }
  }
}
```

## Reproduce
- Version: `0.0.1`  · Git: `ac2f46c`
- Re-run with the same `seeds.json` for identical results.
# EdgeSim – Coverage & Causality (V0 stub)

**Prompt:** main aisle from 2,10 to 17,10, forklift moving from 3,8 to 17,8, forklift moving from 17,12 to 2,12, human crossing from 15,7 to 5,13.5, wall from 0.5,6 to 18,6, rack from 0.5,15.5 to 18,15.5, wall from 0.5,6 to 0.5,14

## Batch Summary
- Runs: **100**
- Successes: **0** · Failures: **100**
- Avg rollout time: **4.50s** · Avg steps: **46**
- Wallclock: **0.00s** · Profile: `minimal`

## Coverage (stub)
```json
{
  "runs": 100,
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
    "<0.20": 100.0,
    "0.20\u20130.50": 0.0,
    ">=0.50": 0.0
  },
  "outcomes_pct": {
    "success": 0.0,
    "collision_human": 100.0,
    "other_failure": 0.0
  },
  "counts": {
    "traction": {
      "wet_encountered": 0,
      "dry_only": 100
    },
    "human_phase": {
      "none": 0,
      "running": 100,
      "fallen": 0
    },
    "clearance_bands": {
      "<0.20": 100,
      "0.20\u20130.50": 0,
      ">=0.50": 0
    },
    "outcomes": {
      "success": 0,
      "collision_human": 100,
      "other_failure": 0
    }
  }
}
```

## Reproduce
- Version: `0.0.1`  · Git: `ac2f46c`
- Re-run with the same `seeds.json` for identical results.
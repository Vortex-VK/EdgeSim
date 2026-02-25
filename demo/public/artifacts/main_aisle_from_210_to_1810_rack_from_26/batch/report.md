# EdgeSim – Coverage & Causality (V0 stub)

**Prompt:** main aisle from 2,10 to 18,10, rack from 2,6.5 to 18,6.5, rack from 2,13.5 to 7.5,13.5, rack from 12.5,13.5 to 18,13.5, human crossing from 12,14 to 7.5,9, forklift moving from 18,9.5 to 4,11

## Batch Summary
- Runs: **100**
- Successes: **0** · Failures: **100**
- Avg rollout time: **4.22s** · Avg steps: **43**
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
    "<0.20": 93.0,
    "0.20\u20130.50": 7.0,
    ">=0.50": 0.0
  },
  "outcomes_pct": {
    "success": 0.0,
    "collision_human": 58.0,
    "other_failure": 42.0
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
      "<0.20": 93,
      "0.20\u20130.50": 7,
      ">=0.50": 0
    },
    "outcomes": {
      "success": 0,
      "collision_human": 58,
      "other_failure": 42
    }
  }
}
```

## Reproduce
- Version: `0.0.1`  · Git: `ac2f46c`
- Re-run with the same `seeds.json` for identical results.
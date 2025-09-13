# EdgeSim – Coverage & Causality (V0 stub)

**Prompt:** Night shift; wet floor near dock; human running across aisle

## Batch Summary
- Runs: **100**
- Successes: **2** · Failures: **98**
- Avg rollout time: **27.27s** · Avg steps: **275**
- Wallclock: **0.00s** · Profile: `minimal`

## Coverage (stub)
```json
{
  "runs": 100,
  "traction_pct": {
    "wet_encountered": 59.0,
    "dry_only": 41.0
  },
  "human_phase_pct": {
    "none": 0.0,
    "running": 14.0,
    "fallen": 86.0
  },
  "clearance_bands_pct": {
    "<0.20": 88.0,
    "0.20\u20130.50": 0.0,
    ">=0.50": 12.0
  },
  "outcomes_pct": {
    "success": 12.0,
    "collision_human": 88.0,
    "other_failure": 0.0
  },
  "counts": {
    "traction": {
      "wet_encountered": 59,
      "dry_only": 41
    },
    "human_phase": {
      "none": 0,
      "running": 14,
      "fallen": 86
    },
    "clearance_bands": {
      "<0.20": 88,
      "0.20\u20130.50": 0,
      ">=0.50": 12
    },
    "outcomes": {
      "success": 12,
      "collision_human": 88,
      "other_failure": 0
    }
  }
}
```

## Reproduce
- Version: `0.0.1`  · Git: `nogit`
- Re-run with the same `seeds.json` for identical results.
# EdgeSim – Coverage & Causality (V0 stub)

**Prompt:** Night shift; wet floor near dock; human crossing; high pallet traffic

## Batch Summary
- Runs: **50**
- Successes: **14** · Failures: **36**
- Avg rollout time: **9.20s** · Avg steps: **94**
- Wallclock: **0.00s** · Profile: `minimal`

## Coverage (stub)
```json
{
  "runs": 50,
  "traction_pct": {
    "wet_encountered": 74.0,
    "dry_only": 26.0
  },
  "human_phase_pct": {
    "none": 0.0,
    "running": 16.0,
    "fallen": 84.0
  },
  "clearance_bands_pct": {
    "<0.20": 72.0,
    "0.20\u20130.50": 0.0,
    ">=0.50": 28.0
  },
  "outcomes_pct": {
    "success": 28.0,
    "collision_human": 72.0,
    "other_failure": 0.0
  },
  "counts": {
    "traction": {
      "wet_encountered": 37,
      "dry_only": 13
    },
    "human_phase": {
      "none": 0,
      "running": 8,
      "fallen": 42
    },
    "clearance_bands": {
      "<0.20": 36,
      "0.20\u20130.50": 0,
      ">=0.50": 14
    },
    "outcomes": {
      "success": 14,
      "collision_human": 36,
      "other_failure": 0
    }
  }
}
```

## Reproduce
- Version: `0.0.1`  · Git: `5f0922a`
- Re-run with the same `seeds.json` for identical results.
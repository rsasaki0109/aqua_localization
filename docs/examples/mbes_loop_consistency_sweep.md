# MBES Loop Consistency Sweep Example

This is a synthetic example of the markdown produced by:

```bash
ros2 run aqua_localization export_mbes_loop_status.py \
  --bag aqua_localization/datasets/public/mbes_slam/demo_with_estimate \
  --out /tmp/mbes_loop_status.csv \
  --consistency-sweep-out /tmp/mbes_loop_consistency_sweep.md \
  --consistency-rejection-audit-out /tmp/mbes_loop_consistency_rejections.md \
  --batch-consistency-out /tmp/mbes_loop_batch_consistency.md \
  --consistency-min-support-count 1
```

Do not copy these values into `config/mbes_loop_closure.yaml`. Use this file to
understand the report shape, then generate a sweep from your own replay after
the first accepted loops have been visually audited.

## Input

- Source: synthetic example
- Accepted loops with finite corrections: 12
- Pairwise accepted correction pairs: 66
- Delta source: 66 pose-aware pairs, 0 scalar-magnitude fallback pairs
- Consistency thresholds at replay time: disabled
- Runtime status diagnostics in new replays:
  `consistency_support_count`, `consistency_required_support_count`,
  `consistency_nearest_translation_delta_m`,
  `consistency_nearest_rotation_delta_rad`

## Pairwise Accepted Correction Deltas

| Metric | Count | Min | Median | P95 | Max |
|--------|------:|----:|-------:|----:|----:|
| translation_delta_m | 66 | 0.08 | 0.72 | 1.85 | 2.60 |
| rotation_delta_rad | 66 | 0.01 | 0.08 | 0.22 | 0.31 |

## Threshold Candidates

`Would keep accepted` includes the first accepted loop because it bootstraps the
runtime guard. `Supported pairs` counts pairwise agreement among all accepted
corrections and is useful for spotting loose thresholds. New replays use the
recorded correction pose to mirror the runtime guard's SE(3) delta check; older
bags without that field fall back to scalar correction magnitudes. After a
replay with thresholds enabled, use the status CSV diagnostics to see whether a
candidate missed the required support count or only barely exceeded the nearest
translation/rotation delta threshold.

| Translation delta <= m | Rotation delta <= rad | Would keep accepted | Keep % | Supported pairs | Pair % |
|-----------------------:|----------------------:|--------------------:|-------:|----------------:|-------:|
| 0.70 | 0.08 | 7/12 | 58.3% | 31/66 | 47.0% |
| 0.95 | 0.10 | 9/12 | 75.0% | 43/66 | 65.2% |
| 1.30 | 0.16 | 11/12 | 91.7% | 55/66 | 83.3% |
| 1.90 | 0.24 | 12/12 | 100.0% | 63/66 | 95.5% |
| 2.60 | 0.31 | 12/12 | 100.0% | 66/66 | 100.0% |

## How to Use the Table

Start with a row that keeps the visually trusted accepted loops while rejecting
obvious correction outliers. A row that keeps every accepted loop is not
automatically safer; it may simply be too loose to catch false positives.

Translate one row into consistency parameters like this:

```yaml
loop:
  consistency:
    max_correction_translation_delta_m: 1.30
    max_correction_rotation_delta_rad: 0.16
    min_support_count: 1
```

Keep `min_support_count: 1` for first-pass tuning. Raise it to `2` or more only
after multiple accepted loops have been visually audited and the sweep shows
that trusted loops support each other.

Then replay the bag, export `/mbes_loop_closure/status` again, and compare:

- `loop consistency rejected` count
- support count vs. required support count for those rejections
- nearest consistency translation/rotation deltas in the summary and CSV
- `/tmp/mbes_loop_consistency_rejections.md` sorted by support deficit
- `/tmp/mbes_loop_batch_consistency.md` for the largest internally consistent
  correction set across accepted and consistency-rejected candidates
- accepted loop markers in `rviz/mbes_loop_closure.rviz`
- accepted correction translation/rotation tails in the summary
- optimized path changes against the MBES-SLAM reference odometry

If the first accepted loop is suspicious, do not enable this guard yet. The
runtime guard uses the first accepted loop as its initial reference, so a bad
first loop can reject later valid loops.

# AQUA-SLAM Claim Readiness

- Status: `BLOCKED`
- Benchmark sources: `docs/benchmarks/tank_aqua_slam.md`
- Baseline: `AQUA-SLAM`
- Target prefixes: `aqua_`
- Held-out sequence: `Medium`
- Official Tank Dataset download page: https://senseroboticslab.github.io/underwater-tank-dataset/download/

## Current Claim

| Best current system | Sequence | Current RMSE m | AQUA-SLAM RMSE m | Gap |
|---------------------|----------|---------------:|-----------------:|----:|
| aqua_dvl_prior_visual | short_test | 0.0154 | 0.0194 | 0.79x |

- No claimable AQUA-SLAM win is available yet.
- Best numeric blockers: current row is diagnostic; held-out validation not established.

## Claim Gate

```bash
ros2 run aqua_localization aqua_slam_head_to_head_report.py docs/benchmarks/tank_aqua_slam.md --baseline-system AQUA-SLAM --fail-without-claimable-win --target-prefix aqua_
```

## Held-Out Readiness

- Status: `BLOCKED`
- Next action: `Find Medium reference TUM`

| Input | Path | Status | Detail |
|-------|------|--------|--------|
| Reference TUM | `/tmp/tank_medium_gt.tum` | FAIL | missing |
| ROS 2 bag | `/tmp/tank_medium_ros2_visual` | FAIL | missing |
| Rank-1 profile | `/tmp/aqua_tank_dvl_prior_confidence_sweep_short_diag/best_profile.yaml` | PASS | exists |
| AQUA-SLAM source | `/tmp/aqua_slam_medium_orb_odom.csv` / `/tmp/aqua_slam_medium_baseline/Medium_aqua_slam.tum` | FAIL | none |
| AQUA-SLAM baseline row | `/tmp/aqua_slam_medium_baseline/Medium_aqua_slam_benchmark_row.md` | FAIL | usable=0, rejected=0 |
| Visual TUM | `/tmp/tank_medium_visual_frontend.tum` | FAIL | missing |

## Candidate Link Bootstrap

```bash
ros2 run aqua_localization verify_tank_medium_heldout_ready.py --sequence Medium --profile /tmp/aqua_tank_dvl_prior_confidence_sweep_short_diag/best_profile.yaml --locator-max-depth 5 --apply-located-links --archive-out-dir /tmp/tank_medium_download --out /tmp/aqua_slam_medium_heldout_verify/heldout_verify.md --locator-root /tmp
```

## Located Candidates

| Role | Count | First candidate |
|------|------:|-----------------|
| Reference TUM | 0 | none |
| ROS 2 bag | 0 | none |
| ROS 1 bag | 0 | none |
| Visual TUM | 0 | none |
| AQUA-SLAM CSV | 0 | none |
| AQUA-SLAM TUM | 0 | none |
| AQUA-SLAM baseline row | 1 | `/tmp/aqua_ingest_aqua_slam_dQaPVa/out/Medium_aqua_slam_benchmark_row.md` |
| Download archive | 0 | none |

## Next Action Command

Find Medium reference TUM: Copy/download the Medium reference TUM, then rescan. https://senseroboticslab.github.io/underwater-tank-dataset/download/

```bash
ros2 run aqua_localization locate_tank_heldout_inputs.py --sequence Medium --profile /tmp/aqua_tank_dvl_prior_confidence_sweep_short_diag/best_profile.yaml --out /tmp/aqua_slam_medium_heldout_verify/heldout_locator.md
```

## Practical Order

1. Satisfy the first failing held-out input above.
2. Run the Medium held-out validation bundle.
3. Regenerate the head-to-head report and rerun the claim gate.
4. Only publish "beats AQUA-SLAM" wording after this report reaches `CLAIMABLE_WIN`.

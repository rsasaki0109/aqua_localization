# AQUA-SLAM Progress Report

- Sources: `docs/benchmarks/tank_aqua_slam.md`
- Baseline system: `AQUA-SLAM`
- Anchor system: `aqua_localization`
- Target prefixes: `aqua_`

| Dataset | Sequence | Alignment | System | RMSE m | Gap to AQUA-SLAM | Improvement to tie | Improvement vs anchor | Samples | Evidence status | Note |
|---------|----------|-----------|--------|-------:|-----------------:|-------------------:|----------------------:|--------:|--------------|------|
| Tank Dataset | short_test | SE(3) | aqua_dvl_prior_visual | 0.0154 | 0.79x | 0.0% | 96.4% | 218 | diagnostic | profile=tank_short_to_medium_confidence_sweep_rank1; prior=169/217; status=PASS; diagnostic override |
| Tank Dataset | short_test | SE(3) | aqua_visual_frontend | 0.0947 | 4.88x | 79.5% | 77.9% | 200 | diagnostic | tracking.translation_scale=0.169623465, same-sequence scale fit |
| Tank Dataset | short_test | SE(3) | aqua_localization+visual | 0.2175 | 11.21x | 91.1% | 49.3% | 5400 | non-diagnostic | visual warmup, base-frame visual odometry, orb.n_features=700, orb.fast_threshold=16, OpenCV threads 2, replay rate 1.0, visual coverage 300/300 |
| Tank Dataset | short_test | SE(3) | aqua_localization | 0.4291 | 22.12x | 95.5% | 0.0% | 5399 | non-diagnostic | same AprilTag GT export |

## Readout

- Best current row: `aqua_dvl_prior_visual` on `short_test` at 0.0154 m RMSE.
- Gap to `AQUA-SLAM` there: 0.79x.
- Remaining RMSE reduction to tie: 0.0%.
- Best current row is diagnostic; do not use it as a superiority claim.
- Best non-diagnostic row: `aqua_localization+visual` on `short_test` at 0.2175 m RMSE (11.21x `AQUA-SLAM`).
- Improvement versus `aqua_localization` anchor: 96.4%.

This report is a progress meter, not a superiority claim. A win requires a target row below the AQUA-SLAM RMSE on the same dataset, sequence, alignment, and reference trajectory.


# AQUA-SLAM Head-to-Head Diagnosis

- Sources: `docs/benchmarks/tank_aqua_slam.md`
- Baseline system: `AQUA-SLAM`
- Target prefixes: `aqua_`
- Claim gates: baseline >= 10 samples / 10.0s, target >= 10 samples / 10.0s
- Held-out note required: `true`

## Claim Table

| Dataset | Sequence | Alignment | Current system | Verdict | Current RMSE m | AQUA-SLAM RMSE m | Gap | Improvement to tie | Evidence blockers |
|---------|----------|-----------|----------------|---------|---------------:|-----------------:|----:|-------------------:|-------------------|
| Tank Dataset | Medium | TBD | TBD | blocked | TBD | TBD | TBD | TBD | missing AQUA-SLAM measured row; missing current measured row |
| Tank Dataset | Structure_Easy | TBD | TBD | blocked | TBD | TBD | TBD | TBD | missing AQUA-SLAM measured row; missing current measured row |
| Tank Dataset | short_test | SE(3) | aqua_dvl_prior_visual | behind | 0.0323 | 0.0194 | 1.66x | 39.9% | current row is diagnostic; held-out validation not established |
| Tank Dataset | short_test | Sim(3) | aqua_visual_frontend | blocked | 0.0958 | TBD | TBD | TBD | missing AQUA-SLAM measured row; current row is diagnostic; held-out validation not established |
| Tank Dataset | short_test | none | TBD | blocked | TBD | 3.5186 | TBD | TBD | missing current measured row; baseline matched duration missing |
| Tank Dataset | short_test_visual_direct_scaled_check | SE(3) | aqua_visual_frontend_direct | blocked | 0.1907 | TBD | TBD | TBD | missing AQUA-SLAM measured row; held-out validation not established |

## Metric Detail

| Dataset | Sequence | Current / AQUA-SLAM mean m | median m | P95 m | max m | samples | matched s | Pending evidence |
|---------|----------|----------------------------:|---------:|------:|------:|--------:|----------:|------------------|
| Tank Dataset | Medium | TBD / TBD | TBD / TBD | TBD / TBD | TBD / TBD | TBD / TBD | TBD / TBD | aqua_visual_frontend: held-out validation after scale calibration on Structure_Easy |
| Tank Dataset | Structure_Easy | TBD / TBD | TBD / TBD | TBD / TBD | TBD / TBD | TBD / TBD | TBD / TBD | aqua_localization: run closest available input mode; AQUA-SLAM: record AQUA-SLAM output topic to TUM |
| Tank Dataset | short_test | 0.0297 / 0.0173 | 0.0268 / 0.0165 | TBD / TBD | 0.0675 / 0.0579 | 218 / 234 | 11.10 / 11.65 | none |
| Tank Dataset | short_test | 0.0826 / TBD | 0.0786 / TBD | TBD / TBD | 0.2458 / TBD | 200 / TBD | 11.35 / TBD | none |
| Tank Dataset | short_test | TBD / TBD | TBD / TBD | TBD / TBD | TBD / TBD | TBD / 234 | TBD / TBD | none |
| Tank Dataset | short_test_visual_direct_scaled_check | 0.1710 / TBD | 0.1768 / TBD | TBD / TBD | 0.3282 / TBD | 300 / TBD | 14.95 / TBD | none |

## Readout

- Best numeric row: `aqua_dvl_prior_visual` on `short_test` at 0.0323 m RMSE.
- Gap to `AQUA-SLAM` there: 1.66x (39.9% RMSE reduction to tie).
- It is not a superiority claim because: current row is diagnostic; held-out validation not established.
- No claimable win is available under the configured gates.

## GitHub-Safe Summary

| Status | Best current | Current RMSE m | AQUA-SLAM RMSE m | Gap | Claim |
|--------|--------------|---------------:|-----------------:|----:|-------|
| behind | aqua_dvl_prior_visual / short_test | 0.0323 | 0.0194 | 1.66x | not claimable yet |

## Practical PR Order

1. Produce a held-out Tank Medium AQUA-SLAM row and matching current row with enough samples.
2. Add P95 to benchmark rows when raw error vectors are available.
3. Only after the held-out table exists, tune the visual/DVL frontend against the largest remaining metric gap.

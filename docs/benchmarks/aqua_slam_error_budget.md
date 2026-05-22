# AQUA-SLAM Error Budget

- Sources: `docs/benchmarks/tank_aqua_slam.md`
- Baseline: `AQUA-SLAM` `short_test` 0.0194 m RMSE
- Best target: `aqua_visual_frontend` `short_test` 0.0947 m RMSE
- Gap to tie: 0.0753 m (4.88x baseline, 79.5% reduction needed)

## Budget

| Bucket | Evidence | Effect m | Next action |
|--------|----------|---------:|-------------|
| Best target gap | aqua_visual_frontend 0.0947 m vs AQUA-SLAM 0.0194 m | 0.0753 | Reduce this residual before any accuracy win claim. |
| Anchor improvement already banked | aqua_localization 0.4291 m -> aqua_visual_frontend 0.0947 m | 0.3344 | Keep this as the measured progress floor; do not regress below it. |
| Fusion regression budget | aqua_localization+visual is 0.1228 m worse than standalone visual | 0.1228 | Tune visual covariance, timing, and coupling before claiming fused-system progress. |

## Development Readout

- Standalone visual is currently the accuracy leader; fused visual+DVL is still a regression relative to that row.
- Add `--drift-report` and `--motion-report` from the best visual run to split scale, extrinsic, and drift terms.


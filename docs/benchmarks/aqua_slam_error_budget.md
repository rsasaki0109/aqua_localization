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

## Next Sweep

Use `run_tank_visual_fusion_sweep.py` to attack the `0.1228 m` fusion
regression directly. The first sweep should vary
`imu.visual.position_variance_floor` and `imu.visual.max_age_s` while keeping
the current best visual frontend settings fixed:

```bash
ros2 run aqua_localization run_tank_visual_fusion_sweep.py \
  --bag /tmp/short_test_ros2_visual \
  --reference /tmp/tank_short_test_gt.tum \
  --out-dir /tmp/aqua_tank_visual_fusion_covariance_sweep \
  --sequence short_test_visual_fusion_covariance \
  --translation-scale 0.169623465 \
  --base-from-camera-x-m -0.25 \
  --base-from-camera-y-m -0.45 \
  --max-stereo-descriptor-distance 64 \
  --max-temporal-descriptor-distance 64 \
  --orb-n-features 700 \
  --orb-fast-threshold 16 \
  --opencv-threads 2 \
  --expected-visual-frames 300 \
  --baseline-rmse-m 0.0194 \
  --standalone-visual-rmse-m 0.0947 \
  --pairs 0.005:0.1,0.01:0.25,0.02:0.5,0.04:1.0
```

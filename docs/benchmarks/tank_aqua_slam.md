# Tank Dataset AQUA-SLAM Head-to-Head

This file is the comparison table target for `aqua_localization` vs
[AQUA-SLAM](https://github.com/SenseRoboticsLab/AQUA-SLAM) on Tank Dataset
sequences.

An initial AQUA-SLAM trajectory has been recorded for `short_test.bag` with the
upstream Docker workflow and converted into the same TUM trajectory format used
by this repository. The current result is a first reproducibility anchor, not a
paper claim, because the two systems use different sensor frontends and
AQUA-SLAM only publishes after its visual-inertial initialization.

Upstream AQUA-SLAM publishes its main estimate on `/AQUA_SLAM/orb_odom`
(`nav_msgs/Odometry`). Record that ROS 1 topic with `rostopic echo -p`, then
convert it with `ros1_odometry_csv_to_tum.py`.

## Current Anchor Result

`aqua_localization` already has a public Tank Dataset anchor on `short_test`
using IMU + pressure + DVL and AprilTag SLAM ground truth:

| Dataset | Sequence | System | Inputs | Alignment | RMSE m | Source |
|---------|----------|--------|--------|-----------|-------:|--------|
| Tank Dataset | `short_test` | `aqua_localization` | IMU + pressure + DVL | SE(3) | 0.429 | [`datasets/tank_dataset_demo.md`](../../datasets/tank_dataset_demo.md) |

The first AQUA-SLAM measurement below uses `short_test.bag` and the upstream
`underwater_orbslam3_blue_gx5_short.yaml` configuration.

## Initial AQUA-SLAM Measurement

Recorded on 2026-05-20:

| Dataset | Sequence | System | Inputs | Alignment | Samples | Matched s | Mean m | Median m | RMSE m | Max m | Note |
|---------|----------|--------|--------|-----------|--------:|----------:|-------:|---------:|-------:|------:|------|
| Tank Dataset | `short_test` | AQUA-SLAM | stereo + IMU + DVL | SE(3) | 234 | 11.65 | 0.0173 | 0.0165 | 0.0194 | 0.0579 | Docker image `orb_dvl2_ros_noetic`, `/AQUA_SLAM/orb_odom` |

Raw-frame diagnostic for the same output, before SE(3) alignment:

| Dataset | Sequence | System | Alignment | Samples | RMSE m | Note |
|---------|----------|--------|-----------|--------:|-------:|------|
| Tank Dataset | `short_test` | AQUA-SLAM | none | 234 | 3.5186 | Confirms the odometry and AprilTag GT frames must be aligned before APE reporting. |

This short clip is a useful sanity check because AQUA-SLAM initializes, creates
keyframes, and publishes odometry, but it is only 11.65 matched seconds.

## Current `aqua_localization` Measurement

Recorded on 2026-05-20 with ROS 2 Humble, the converted `short_test` bag, and
the Tank Dataset IMU + pressure + DVL profile:

| Dataset | Sequence | System | Inputs | Alignment | Samples | Matched s | Mean m | Median m | RMSE m | Max m | Note |
|---------|----------|--------|--------|-----------|--------:|----------:|-------:|---------:|-------:|------:|------|
| Tank Dataset | `short_test` | `aqua_localization` | IMU + pressure + DVL | SE(3) | 5399 | 14.94 | 0.3796 | 0.4014 | 0.4291 | 0.7652 | same AprilTag GT export |

This is a fair current-stack result for the lighter ROS 2 localization mode,
but it is not sensor-equivalent to AQUA-SLAM's stereo + IMU + DVL frontend.
AQUA-SLAM is the accuracy target on this visual-DVL-IMU sequence; the immediate
`aqua_localization` win path is either validating and fusing the experimental
`stereo_visual_odometry.py` frontend on Tank or moving the claim to ROS 2
reproducibility, permissive licensing, and MBES/sonar datasets where this
repository has stronger tooling.

## Experimental Visual Frontend Measurement

Recorded on 2026-05-20 with the new `stereo_visual_odometry.py` frontend on the
camera-included `short_test` conversion:

| Dataset | Sequence | System | Inputs | Alignment | Samples | Matched s | Mean m | Median m | RMSE m | Max m | Note |
|---------|----------|--------|--------|-----------|--------:|----------:|-------:|---------:|-------:|------:|------|
| Tank Dataset | `short_test` | `aqua_visual_frontend` | stereo only | SE(3) | 200 | 11.35 | 1.2667 | 1.2280 | 1.3649 | 2.2035 | nominal stereo scale |
| Tank Dataset | `short_test` | `aqua_visual_frontend` | stereo only | Sim(3) | 200 | 11.35 | 0.0826 | 0.0786 | 0.0958 | 0.2458 | scale diagnostic, not paper-safe |
| Tank Dataset | `short_test` | `aqua_visual_frontend` | stereo only | SE(3) | 200 | 11.25 | 0.0815 | 0.0792 | 0.0947 | 0.2416 | `tracking.translation_scale=0.169623465`, same-sequence scale fit |
| Tank Dataset | `short_test` | `aqua_localization+visual` | IMU + pressure + DVL + stereo | SE(3) | 5399 | 14.94 | 0.3384 | 0.2928 | 0.3726 | 0.7497 | visual position update, same-sequence scale fit, variance floor 0.0025 |
| Tank Dataset | `short_test` | `aqua_localization+visual` | IMU + pressure + DVL + stereo | SE(3) | 5424 | 14.95 | 0.2579 | 0.2220 | 0.3228 | 1.2305 | visual position update, base-frame visual odometry, `base_from_camera=(-0.25,-0.45,0)` m, variance floor 0.01, replay rate 0.25, visual coverage 300/300 |
| Tank Dataset | `short_test` | `aqua_localization+visual` | IMU + pressure + DVL + stereo | SE(3) | 5400 | 14.95 | 0.1793 | 0.1394 | 0.2175 | 0.8564 | visual warmup, base-frame visual odometry, `orb.n_features=700`, `orb.fast_threshold=16`, OpenCV threads 2, replay rate 1.0, visual coverage 300/300 |

The frontend is already better than the IMU + pressure + DVL row once metric
scale is calibrated, but it still does not beat AQUA-SLAM's 0.0194 m RMSE and
the fused result only improves the current stack from 0.4291 m to 0.2175 m.
The next serious step is out-of-sequence scale calibration plus a calibrated
camera-to-base transform before claiming a visual-inertial-DVL result.

Generate the diagnostic visual scale with:

```bash
ros2 run aqua_localization calibrate_visual_scale.py \
  /tmp/tank_short_test_gt.tum \
  /tmp/tank_short_test_visual_frontend.tum \
  --ros-args
```

For paper-safe reporting, the calibration TUM and validation TUM must come from
different sequences.

Once full Tank Dataset access is available, use `validate_visual_scale.py` to
separate calibration and held-out validation:

```bash
ros2 run aqua_localization validate_visual_scale.py \
  --calibration-reference /tmp/tank_structure_easy_gt.tum \
  --calibration-estimate /tmp/tank_structure_easy_visual.tum \
  --validation-reference /tmp/tank_medium_gt.tum \
  --validation-estimate /tmp/tank_medium_visual.tum \
  --calibration-sequence Structure_Easy \
  --validation-sequence Medium \
  --markdown
```

The same calibrated values can be stored as a reusable YAML profile. Use one
calibration sequence to write the profile, then pass that profile to the fusion
benchmark on a held-out sequence:

```bash
ros2 run aqua_localization visual_calibration_profile.py \
  --out /tmp/tank_structure_easy_visual_profile.yaml \
  --name tank_structure_easy_visual \
  --calibration-reference /tmp/tank_structure_easy_gt.tum \
  --calibration-estimate /tmp/tank_structure_easy_visual.tum \
  --calibration-sequence Structure_Easy \
  --validation-sequence Medium \
  --base-from-camera-x-m -0.25 \
  --base-from-camera-y-m -0.45 \
  --base-from-camera-z-m 0.0 \
  --orb-n-features 700 \
  --orb-fast-threshold 16 \
  --opencv-threads 2 \
  --visual-position-variance-floor 0.01

ros2 run aqua_localization run_tank_visual_fusion_benchmark.py \
  --visual-calibration-profile /tmp/tank_structure_easy_visual_profile.yaml \
  --bag /tmp/tank_medium_ros2_visual \
  --reference /tmp/tank_medium_gt.tum \
  --out-dir /tmp/aqua_tank_medium_visual_fusion \
  --sequence Medium \
  --expected-visual-frames 300
```

CLI arguments passed to `run_tank_visual_fusion_benchmark.py` override profile
defaults, so a profile can be reused while still sweeping replay rate or feature
settings. Benchmark rows include the profile name in the note.

For single-sequence visual frontend experiments, `run_tank_visual_benchmark.py`
can replay a camera bag, record `/aqua_visual_frontend/odometry`, emit the scale
diagnostic, and save a Markdown benchmark row:

```bash
ros2 run aqua_localization run_tank_visual_benchmark.py \
  --bag /path/to/tank_sequence_ros2_with_cameras \
  --reference /tmp/tank_sequence_gt.tum \
  --out-dir /tmp/aqua_tank_visual_sequence \
  --sequence tank_sequence
```

When a visual TUM file has already been recorded, pass `--estimate` instead of
`--bag` to regenerate the scale report and benchmark row without replaying ROS.
The bag replay mode also saves `*_visual_frontend_status.csv`, which contains
per-frame feature counts, stereo match counts, triangulated point counts,
disparity/depth statistics, temporal match counts, PnP inliers, inlier ratio,
accepted/rejected state, and the reject reason. Use that CSV to decide whether
the next tuning pass should focus on image features, stereo geometry, temporal
matching, or PnP gates.
It also emits `*_visual_frontend_status.md` via
`summarize_visual_frontend_status.py`, so each visual benchmark run carries a
short tuning report next to the trajectory metrics. The same run writes
`*_visual_drift.md` with sliding-window SE(3), Sim(3), and scale estimates from
`analyze_visual_drift.py`; use that report to separate fixed-scale calibration
problems from true visual drift. It also writes
`*_visual_motion_segments.md` from `analyze_visual_motion_segments.py`, comparing
short visual/reference segment lengths and direction buckets to expose
frame-convention or extrinsic-dependent motion bias.

Descriptor-distance sweeps are handled by `run_tank_visual_matching_sweep.py`.
The runner assigns a unique odometry topic to every case so repeated bag replays
cannot contaminate the recorder with stale visual-odometry publishers.

```bash
ros2 run aqua_localization run_tank_visual_matching_sweep.py \
  --bag /tmp/short_test_ros2_visual \
  --reference /tmp/tank_short_test_gt.tum \
  --out-dir /tmp/aqua_tank_visual_matching_sweep_real_unique \
  --sequence short_test \
  --translation-scale 0.169623465 \
  --baseline-rmse-m 0.0194 \
  --pairs 64:64,80:80,96:96,112:112,disabled:disabled
```

Latest `short_test` sweep result, recorded on 2026-05-20:

| Stereo dist | Temporal dist | RMSE m | Gap to AQUA-SLAM | Matched s | Accepted | Median PnP inliers | Median temporal matches |
|-------------|---------------|-------:|-----------------:|----------:|---------:|-------------------:|------------------------:|
| 64 | 64 | 0.1699 | 8.76x | 14.95 | 100.0% | 195.0 | 219.0 |
| 80 | 80 | 0.1859 | 9.58x | 14.95 | 100.0% | 207.0 | 239.0 |
| 96 | 96 | 0.1864 | 9.61x | 14.95 | 100.0% | 208.0 | 241.0 |
| 112 | 112 | 0.1864 | 9.61x | 14.95 | 100.0% | 208.0 | 241.0 |
| disabled | disabled | 0.1864 | 9.61x | 14.95 | 100.0% | 208.0 | 241.0 |

This says descriptor filtering alone is not the win condition. The next useful
accuracy work is camera-to-base/extrinsic calibration and visual-inertial-DVL
coupling, not another small threshold-only sweep.

Camera-to-base lever-arm hypotheses can be tested without replaying the bag by
post-processing the visual TUM trajectory:

```bash
ros2 run aqua_localization sweep_visual_extrinsics.py \
  --reference /tmp/tank_short_test_gt.tum \
  --estimate /tmp/aqua_tank_visual_matching_sweep_real_unique/stereo_64__temporal_64/short_test_stereo_64__temporal_64_visual_frontend.tum \
  --out-dir /tmp/aqua_tank_visual_extrinsic_sweep_fine \
  --sequence short_test \
  --x-m=-0.45,-0.35,-0.25,-0.15,-0.05 \
  --y-m=-0.45,-0.35,-0.25,-0.15,-0.05 \
  --z-m=0 \
  --roll-deg=0 \
  --pitch-deg=0 \
  --yaw-deg=-10,0,10
```

Latest lever-arm readout on the best `64:64` visual trajectory:

| x m | y m | z m | roll deg | pitch deg | yaw deg | RMSE m | Matched s | Samples |
|----:|----:|----:|---------:|----------:|--------:|-------:|----------:|--------:|
| 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.1699 | 14.95 | 273 |
| -0.25 | -0.45 | 0.00 | 0.00 | 0.00 | 0.00 | 0.1417 | 14.95 | 273 |

The same lever arm was then replayed through `stereo_visual_odometry.py` itself:

```bash
ros2 run aqua_localization run_tank_visual_benchmark.py \
  --bag /tmp/short_test_ros2_visual \
  --reference /tmp/tank_short_test_gt.tum \
  --out-dir /tmp/aqua_tank_visual_base_extrinsic_benchmark \
  --sequence short_test_visual_base_extrinsic \
  --translation-scale 0.169623465 \
  --max-stereo-descriptor-distance 64 \
  --max-temporal-descriptor-distance 64 \
  --base-from-camera-x-m=-0.25 \
  --base-from-camera-y-m=-0.45 \
  --base-from-camera-z-m=0.0
```

That direct node run also produced 273 matched samples over 14.95 s with
0.1417 m SE(3) RMSE. The visual odometry publisher switches its child frame to
`base_link` when a non-zero `extrinsics.base_from_camera.*` parameter is set.

This is still a same-sequence diagnostic, not a paper-safe calibration. It does
show that extrinsics are a real error source: the tested lever arm improves the
visual frontend RMSE by about 16.6%, while the descriptor threshold sweep only
changed the best run within roughly 0.02 m.

The same base-frame visual odometry can be fed into `aqua_imu_loc` with the
reproducible fusion runner. The visual node now warms up ORB before subscribing,
which removes the first-frame OpenCV initialization spike from bag replay. A
lighter 700-feature profile keeps `short_test` at full 1.0x replay coverage:

```bash
ros2 run aqua_localization run_tank_visual_fusion_benchmark.py \
  --bag /tmp/short_test_ros2_visual \
  --reference /tmp/tank_short_test_gt.tum \
  --out-dir /tmp/aqua_tank_visual_fusion_profile_1x_warm_fast700 \
  --sequence short_test_visual_1x_warm_fast700 \
  --translation-scale 0.169623465 \
  --max-stereo-descriptor-distance 64 \
  --max-temporal-descriptor-distance 64 \
  --base-from-camera-x-m -0.25 \
  --base-from-camera-y-m -0.45 \
  --base-from-camera-z-m 0.0 \
  --visual-position-variance-floor 0.01 \
  --orb-n-features 700 \
  --orb-fast-threshold 16 \
  --opencv-threads 2 \
  --play-rate 1.0 \
  --expected-visual-frames 300
```

This 1.0x run produced 5400 fused samples over 14.95 s with 0.2175 m SE(3) RMSE
and 300/300 visual frame coverage. Before the warmup path, the same 1.0x runner
processed only 268-273 visual frames on this machine and the fused RMSE
regressed to about 0.38-0.42 m. The runner writes a `*_visual_coverage.md`
report with decode/stereo/tracking/total processing time and warns when
processed/expected visual frames fall below the configured gate.

The public Tank Dataset page currently exposes `short_test` as sample data and
requires the download form for the full sequence set, so this table keeps
Structure_Easy and Medium rows as targets until those bags are available.

## Head-to-Head Table

Rows are generated with `trajectory_benchmark_row.py` so both systems use the
same APE implementation.

| Dataset | Sequence | System | Alignment | Samples | Matched s | Mean m | Median m | RMSE m | Max m | Note |
|---------|----------|--------|-----------|--------:|----------:|-------:|---------:|-------:|------:|------|
| Tank Dataset | short_test | AQUA-SLAM | SE(3) | 234 | 11.65 | 0.0173 | 0.0165 | 0.0194 | 0.0579 | AQUA-SLAM Docker, short_test, /AQUA_SLAM/orb_odom |
| Tank Dataset | short_test | aqua_localization | SE(3) | 5399 | 14.94 | 0.3796 | 0.4014 | 0.4291 | 0.7652 | ROS 2 Humble, IMU+pressure+DVL, same AprilTag GT export |
| Tank Dataset | short_test | aqua_visual_frontend | SE(3) | 200 | 11.25 | 0.0815 | 0.0792 | 0.0947 | 0.2416 | stereo ORB+PnP, same-sequence scale fit from calibrate_visual_scale.py |
| Tank Dataset | short_test | aqua_localization+visual | SE(3) | 5399 | 14.94 | 0.3384 | 0.2928 | 0.3726 | 0.7497 | visual position update, same-sequence scale fit |
| Tank Dataset | short_test | aqua_localization+visual | SE(3) | 5424 | 14.95 | 0.2579 | 0.2220 | 0.3228 | 1.2305 | base-frame visual odometry, same-sequence scale/extrinsic diagnostics, replay rate 0.25, visual coverage 300/300 |
| Tank Dataset | short_test | aqua_localization+visual | SE(3) | 5400 | 14.95 | 0.1793 | 0.1394 | 0.2175 | 0.8564 | visual warmup, base-frame visual odometry, 700 ORB features, replay rate 1.0, visual coverage 300/300 |
| Tank Dataset | Medium | aqua_visual_frontend | TBD | TBD | TBD | TBD | TBD | TBD | TBD | held-out validation after scale calibration on Structure_Easy |
| Tank Dataset | Structure_Easy | AQUA-SLAM | TBD | TBD | TBD | TBD | TBD | TBD | TBD | record AQUA-SLAM output topic to TUM |
| Tank Dataset | Structure_Easy | aqua_localization | TBD | TBD | TBD | TBD | TBD | TBD | TBD | run closest available input mode |

## Gap Report

Use `benchmark_gap_report.py` to turn the table above into an explicit "how far
from winning" readout:

```bash
ros2 run aqua_localization benchmark_gap_report.py \
  docs/benchmarks/tank_aqua_slam.md \
  --target-system aqua_visual_frontend \
  --baseline-system AQUA-SLAM
```

Current output for the experimental visual frontend:

| Dataset | Sequence | Alignment | Target RMSE m | Baseline RMSE m | Gap x | Improvement to tie | Target samples | Baseline samples |
|---------|----------|-----------|--------------:|----------------:|------:|-------------------:|---------------:|-----------------:|
| Tank Dataset | short_test | SE(3) | 0.0947 | 0.0194 | 4.88 | 79.5% | 200 | 234 |

Read this as an engineering target, not as a paper result: the current visual
frontend needs roughly an 80% SE(3) RMSE reduction on the matched `short_test`
window to tie AQUA-SLAM. The most useful next benchmark rows are held-out Tank
sequences with one sequence used only for visual scale/extrinsic calibration and
another sequence used only for validation.

The same command can act as a regression gate. This passes for the current
experimental visual frontend:

```bash
ros2 run aqua_localization benchmark_gap_report.py \
  docs/benchmarks/tank_aqua_slam.md \
  --target-system aqua_visual_frontend \
  --baseline-system AQUA-SLAM \
  --max-gap-x 5.0 \
  --max-improvement-to-tie-percent 80.0
```

Tighten those numbers after each real accuracy improvement. For example,
`--max-gap-x 4.0` intentionally fails today because the current gap is `4.88x`.

For a broader progress ladder across all measured `aqua_*` rows, generate
[`aqua_slam_progress.md`](aqua_slam_progress.md):

```bash
ros2 run aqua_localization aqua_slam_progress_report.py \
  docs/benchmarks/tank_aqua_slam.md \
  --out docs/benchmarks/aqua_slam_progress.md
```

The current best row is `aqua_visual_frontend` at `0.0947 m` RMSE, which is
`4.88x` the AQUA-SLAM `short_test` RMSE and a `77.9%` improvement over the
IMU+pressure+DVL `aqua_localization` anchor. The fused
`aqua_localization+visual` row improves that anchor by `49.3%`, but still needs
a `91.1%` RMSE reduction to tie AQUA-SLAM.

Before updating the head-to-head table after a matching change, run the visual
matching sweep so the selected ORB descriptor-distance gates are evidence-based:

```bash
ros2 run aqua_localization run_tank_visual_matching_sweep.py \
  --bag /path/to/tank_sequence_ros2_with_cameras \
  --reference /tmp/tank_sequence_gt.tum \
  --out-dir /tmp/aqua_tank_visual_matching_sweep \
  --sequence tank_sequence \
  --translation-scale <calibrated-scale> \
  --baseline-rmse-m 0.0194 \
  --pairs 64:64,80:80,96:96,112:112,disabled:disabled
```

Use the best held-out setting from `visual_matching_sweep.md` when generating
the next `trajectory_benchmark_row.py` result. The baseline RMSE column makes
the sweep table directly show how far each setting is from the current
AQUA-SLAM `short_test` anchor.

## Generate Rows

Use the same reference TUM for both systems:

```bash
# In the AQUA-SLAM ROS 1 Docker container while the Tank bag is playing.
rostopic echo -p /AQUA_SLAM/orb_odom > /tmp/aqua_slam_orb_odom.csv

# In this ROS 2 workspace after making the CSV visible on the host.
ros2 run aqua_localization ros1_odometry_csv_to_tum.py \
  --csv /tmp/aqua_slam_orb_odom.csv \
  --out /tmp/tank_short_test_aqua_slam.tum \
  --time-unit auto
```

```bash
ros2 run aqua_localization trajectory_benchmark_row.py \
  --reference /tmp/tank_short_test_gt.tum \
  --estimate /tmp/tank_short_test_aqua_slam.tum \
  --dataset "Tank Dataset" \
  --sequence short_test \
  --system AQUA-SLAM \
  --note "AQUA-SLAM Docker, short_test, /AQUA_SLAM/orb_odom" \
  --header
```

```bash
ros2 run aqua_localization trajectory_benchmark_row.py \
  --reference /tmp/tank_short_test_gt.tum \
  --estimate /tmp/tank_short_test_aqua_localization.tum \
  --dataset "Tank Dataset" \
  --sequence short_test \
  --system aqua_localization \
  --note "ROS 2, same short_test reference"
```

Use rigid SE(3) alignment by default. Use `--scale` only if the paper section
explicitly reports Sim(3), and use `--no-align` only for raw-frame diagnostics.

## What Counts as Winning

A paper-safe win should include at least one of:

- lower APE RMSE on the same sequence and same reference trajectory
- similar APE with easier ROS 2 reproduction and fewer manual calibration edits
- better stability across repeated runs, especially if the AQUA-SLAM known
  long-sequence crash appears
- better permissive reuse story because `aqua_localization` is Apache-2.0
- better MBES/sonar replay diagnostics on non-Tank datasets

A claim that does **not** count yet:

- "beats AQUA-SLAM" based only on README features
- comparing `short_test` against AQUA-SLAM's `Structure_Easy`
- denying AQUA-SLAM stereo images while claiming full SLAM superiority

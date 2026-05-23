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

To compare against the direct replay path, run the ROS replay benchmark on the
same time window. `--start-offset-s` is passed to `ros2 bag play --start-offset`,
and `--duration-s` stops replay after that many bag-time seconds:

```bash
ros2 run aqua_localization run_tank_visual_benchmark.py \
  --bag /tmp/short_test_ros2_visual \
  --reference /tmp/tank_short_test_gt.tum \
  --out-dir /tmp/aqua_tank_visual_ros_1125_sync_check \
  --sequence short_test_visual_ros_1125_sync_check \
  --start-offset-s 0.0 \
  --duration-s 11.25 \
  --translation-scale 0.105024091 \
  --base-from-camera-x-m -0.25 \
  --base-from-camera-y-m -0.45 \
  --max-stereo-descriptor-distance 64 \
  --max-temporal-descriptor-distance 64 \
  --orb-n-features 700 \
  --orb-fast-threshold 16 \
  --opencv-threads 2
```

On 2026-05-22 this ROS replay window produced `220` matched samples over
`10.95` seconds with `0.1132 m` SE(3) RMSE at
`tracking.translation_scale=0.105024091`. Comparing that trajectory against the
direct `11.25` second replay with `compare_visual_trajectories.py` gave
`0.0000 m` SE(3) RMSE over the shared `219` samples, and
`compare_visual_status_timing.py` showed median timestamp delta `0.000 ms`,
median stereo sync delta difference `0.000 ms`, and only the final five direct
frames outside the ROS replay window. This means the earlier ROS/direct gap was
primarily a window mismatch, not a different visual frontend trajectory.

Once replay and direct trajectories match on the same window, use the direct
path for faster calibration sweeps:

```bash
ros2 run aqua_localization run_tank_visual_calibration_sweep.py \
  --bag /tmp/short_test_ros2_visual \
  --reference /tmp/tank_short_test_gt.tum \
  --out-dir /tmp/aqua_tank_visual_calibration_sweep_1125 \
  --sequence short_test_visual_calibration_1125 \
  --start-offset-s 0.0 \
  --duration-s 11.25 \
  --translation-scales 0.08,0.095,0.105024091,0.115,0.13 \
  --camera-bf-scales 0.95,1.0,1.05 \
  --camera-f-scales 0.98,1.0,1.02 \
  --base-from-camera-x-m -0.25 \
  --base-from-camera-y-m -0.45 \
  --baseline-rmse-m 0.0194
```

The sweep writes a CSV and Markdown summary with the best RMSE, Sim(3) scale
diagnostic, accepted-frame ratio, median PnP inliers, median temporal matches,
and the remaining gap to the AQUA-SLAM `0.0194 m` reference row. Keep this
window fixed while deciding whether the remaining error is scale-only, stereo
geometry, or base-frame extrinsics.

Initial calibration sweeps on 2026-05-22 showed that these simple calibration
levers do not close the AQUA-SLAM gap on the `11.25` second window:

| Sweep | Best setting | RMSE m | Gap to AQUA-SLAM | Readout |
|-------|--------------|-------:|-----------------:|---------|
| scale only | `translation_scale=0.095` | 0.1177 | 6.07x | scale is not the main remaining error |
| bf/f geometry | `translation_scale=0.095`, `camera_bf_scale=1.05`, `camera_f_scale=0.98` | 0.1164 | 6.00x | small improvement only |
| base x/y | `translation_scale=0.095`, `camera_bf_scale=1.05`, `camera_f_scale=0.98`, `base_from_camera=(-0.15,-0.55,0)` | 0.1139 | 5.87x | comparable to the ROS replay window row |

The best sweep row still needs about an `83%` RMSE reduction to tie
AQUA-SLAM's `0.0194 m` row. The next accuracy work should therefore target the
visual motion estimator itself: temporal feature stability, PnP outlier
handling, motion prior use, or visual-inertial/DVL coupling, rather than only
static scale or intrinsics.

PnP/RANSAC quality gates can be swept on the same fixed window:

```bash
ros2 run aqua_localization run_tank_visual_pnp_sweep.py \
  --bag /tmp/short_test_ros2_visual \
  --reference /tmp/tank_short_test_gt.tum \
  --out-dir /tmp/aqua_tank_visual_pnp_sweep_1125 \
  --sequence short_test_visual_pnp_1125 \
  --start-offset-s 0.0 \
  --duration-s 11.25 \
  --translation-scale 0.095 \
  --reprojection-errors-px 2,3,4,6 \
  --min-inlier-ratios 0.25,0.5,0.65,0.8 \
  --max-step-translation-m 0.02,0.05,0.10,2.0 \
  --baseline-rmse-m 0.0194
```

The PnP sweep reports RMSE, accepted ratio, rejected-frame count, dominant
reject reason, median PnP inliers, median inlier ratio, and median temporal
matches. A useful result here is not only a lower RMSE: if stricter gates create
rejections without reducing RMSE, the next target is motion prediction or
multi-sensor coupling rather than more aggressive PnP filtering.

Initial PnP sweeps on 2026-05-22 used the best static calibration candidate
from the previous sweep (`translation_scale=0.095`, `camera_bf_scale=1.05`,
`camera_f_scale=0.98`, `base_from_camera=(-0.15,-0.55,0)`). A moderate sweep
over reprojection error `2/3/4 px`, min inlier ratio `0.25/0.65`, and max step
`0.02/2.0 m` did not improve beyond `0.1139 m`. A stricter ratio sweep found a
small improvement:

| reproj px | min ratio | max step m | Accepted | Rejected | RMSE m | Gap to AQUA-SLAM |
|----------:|----------:|-----------:|---------:|---------:|-------:|-----------------:|
| 4.0 | 0.85 | 0.02 | 97.3% | 6 | 0.1128 | 5.81x |
| 3.0 | 0.85 | 0.02 | 87.5% | 28 | 0.1214 | 6.26x |
| 4.0 | 0.90 | 0.02 | 74.6% | 57 | 0.1433 | 7.39x |

This says PnP gate tightening can remove a few bad updates, but it is not the
main gap to AQUA-SLAM. Very strict inlier-ratio gates reject too many frames
and make the trajectory worse. The next likely win is adding a motion prior or
multi-sensor coupling so rejected visual steps can be bridged instead of simply
dropped.

Use per-step diagnostics to identify whether the remaining error is caused by a
few bad visual updates or by broad per-frame motion bias:

```bash
ros2 run aqua_localization analyze_visual_step_errors.py \
  /tmp/tank_short_test_gt.tum \
  /tmp/aqua_tank_visual_pnp_sweep_1125_strict_ratio/repr_4__ratio_0p85__step_0p02__inl_12__iter_100__conf_0p99/short_test_visual_pnp_1125_strict_ratio_repr_4__ratio_0p85__step_0p02__inl_12__iter_100__conf_0p99_visual_frontend.tum \
  --out /tmp/aqua_tank_visual_pnp_sweep_1125_strict_ratio/visual_step_errors.md \
  --csv /tmp/aqua_tank_visual_pnp_sweep_1125_strict_ratio/visual_step_errors.csv \
  --top-k 12
```

This aligns the visual trajectory to the reference, compares each consecutive
visual update with the reference update over the same timestamps, and reports
visual/reference step-length ratio, direction cosine, heading error, cumulative
distance ratio, and the worst local motion updates. Use the worst-step CSV as
the input for a future IMU/DVL motion-prior gate.

The first step-error run on 2026-05-23 used the best strict PnP row
(`0.1128 m` RMSE). It found `217` valid steps with visual cumulative distance
`0.942 m` against `1.152 m` reference distance, a cumulative visual/reference
ratio of `0.818`, median step-length ratio `0.817`, median direction cosine
`0.670`, and median absolute heading error `24.1 deg`. The worst local updates
clustered near the end of the window and around offsets `5.65`, `1.35`,
`7.75`, and `8.45` seconds. This suggests the remaining error is not a single
gross PnP outlier; the visual frontend is often under-estimating motion and has
meaningful direction jitter. A useful motion prior should therefore constrain
both step magnitude and direction, and should bridge rejected/noisy steps rather
than only tightening PnP gates.

Before implementing a production IMU/DVL prior, use the reference as an oracle
motion prior to estimate the possible upside:

```bash
ros2 run aqua_localization simulate_visual_motion_prior.py \
  /tmp/tank_short_test_gt.tum \
  /tmp/aqua_tank_visual_pnp_sweep_1125_strict_ratio/repr_4__ratio_0p85__step_0p02__inl_12__iter_100__conf_0p99/short_test_visual_pnp_1125_strict_ratio_repr_4__ratio_0p85__step_0p02__inl_12__iter_100__conf_0p99_visual_frontend.tum \
  --out-dir /tmp/aqua_tank_visual_motion_prior_oracle \
  --mode replace-outliers \
  --min-length-ratio 0.5 \
  --max-length-ratio 1.5 \
  --min-direction-cosine 0.5
```

This is not a benchmark claim because it uses the reference trajectory as the
prior. It answers a narrower engineering question: if an IMU/DVL prior could
detect and replace visual steps with bad magnitude or direction, how much of
the current visual error could it remove?

The first oracle-prior simulation on 2026-05-23 used the best strict PnP row
(`0.1128 m` RMSE). Replacing visual steps outside the length-ratio window
`[0.5, 1.5]` or below direction cosine `0.5` replaced `140/217` steps and
reduced RMSE to `0.0280 m` (`75.2%` reduction). A softer `blend-all` run with
`alpha=0.5` reduced RMSE to `0.0564 m`. This is still oracle-only, but it is
strong evidence that motion-prior work can plausibly close most of the
AQUA-SLAM gap if the prior supplies useful step magnitude and direction.

The next check uses real Tank DVL velocity instead of the oracle reference.
This diagnostic integrates `/dvl/twist` over each visual frontend step and
compares the DVL step against the same-timestamp reference displacement:

```bash
ros2 run aqua_localization analyze_tank_dvl_motion_prior.py \
  --bag /tmp/short_test_ros2_visual \
  --reference /tmp/tank_short_test_gt.tum \
  --visual /tmp/aqua_tank_visual_pnp_sweep_1125_strict_ratio/repr_4__ratio_0p85__step_0p02__inl_12__iter_100__conf_0p99/short_test_visual_pnp_1125_strict_ratio_repr_4__ratio_0p85__step_0p02__inl_12__iter_100__conf_0p99_visual_frontend.tum \
  --mode gt_yaw \
  --out /tmp/aqua_tank_dvl_prior_gt_yaw/dvl_prior.md \
  --csv /tmp/aqua_tank_dvl_prior_gt_yaw/dvl_prior.csv
```

`--mode gt_yaw` rotates body-frame DVL velocity with reference yaw, so it is
still a diagnostic rather than a deployable prior. It answers whether the DVL
velocity magnitude and frame convention are plausible before replacing GT yaw
with IMU yaw. `--mode body_raw` intentionally skips that rotation and should be
used as a frame sanity check; if raw body velocity looks better than yaw-rotated
velocity, the DVL frame or yaw convention is probably wrong. Use
`--dvl-frame-yaw-offset-deg` to test whether the DVL horizontal axes need a
fixed yaw rotation before they can serve as a visual motion prior.

The first real-DVL diagnostic on 2026-05-23 used the same best strict PnP row
as the oracle simulation. `/dvl/twist` covered `216/217` visual steps. The DVL
cumulative distance was `0.919 m` against `1.152 m` reference distance, a
cumulative ratio of `0.798`. That magnitude is close to the visual frontend's
`0.818` cumulative ratio, but still needs scale/bias calibration before it is
used as a hard step-length prior. Direction only became useful after applying a
`-90 deg` DVL frame yaw offset:

| Mode | DVL yaw offset deg | Covered | Cumulative ratio | Median length ratio | Median direction cosine | Median abs heading error deg | Readout |
|------|-------------------:|--------:|-----------------:|--------------------:|------------------------:|-----------------------------:|---------|
| `gt_yaw` | 0 | 216/217 | 0.798 | 0.791 | 0.023 | 88.9 | nearly perpendicular; frame axes are wrong |
| `gt_yaw` | -90 | 216/217 | 0.798 | 0.791 | 0.878 | 18.7 | plausible DVL direction prior |
| `gt_yaw` | +90 | 216/217 | 0.798 | 0.791 | -0.861 | 161.3 | mostly reversed |
| `body_raw` | 0 | 216/217 | 0.798 | 0.791 | -0.033 | 92.3 | raw body-frame velocity is not directly comparable |

The next deployable check is therefore not another visual threshold sweep. It
is replacing the diagnostic `gt_yaw` rotation with IMU yaw, keeping the `-90 deg`
DVL frame yaw offset, and testing whether a calibrated DVL step prior can bridge
the rejected/noisy visual steps that the oracle simulation identified.

The deployable-input version uses `/imu/data` orientation yaw instead of
reference yaw:

```bash
ros2 run aqua_localization analyze_tank_dvl_motion_prior.py \
  --bag /tmp/short_test_ros2_visual \
  --reference /tmp/tank_short_test_gt.tum \
  --visual /tmp/aqua_tank_visual_pnp_sweep_1125_strict_ratio/repr_4__ratio_0p85__step_0p02__inl_12__iter_100__conf_0p99/short_test_visual_pnp_1125_strict_ratio_repr_4__ratio_0p85__step_0p02__inl_12__iter_100__conf_0p99_visual_frontend.tum \
  --mode imu_yaw \
  --dvl-frame-yaw-offset-deg -90 \
  --imu-yaw-offset-deg 115 \
  --out /tmp/aqua_tank_dvl_prior_imu_yaw_m90_imu115/dvl_prior.md \
  --csv /tmp/aqua_tank_dvl_prior_imu_yaw_m90_imu115/dvl_prior.csv
```

On 2026-05-23 the same `short_test` diagnostic read `4991` IMU yaw samples and
kept DVL coverage at `216/217` visual steps. With DVL frame yaw fixed at
`-90 deg`, sweeping a constant IMU yaw offset gave:

| Mode | DVL yaw offset deg | IMU yaw offset deg | Covered | Cumulative ratio | Median length ratio | Median direction cosine | Median abs heading error deg | Readout |
|------|-------------------:|-------------------:|--------:|-----------------:|--------------------:|------------------------:|-----------------------------:|---------|
| `imu_yaw` | -90 | 0 | 216/217 | 0.798 | 0.791 | -0.383 | 115.5 | wrong global yaw convention |
| `imu_yaw` | -90 | 90 | 216/217 | 0.798 | 0.791 | 0.795 | 32.6 | usable, but worse than GT-yaw diagnostic |
| `imu_yaw` | -90 | 115 | 216/217 | 0.798 | 0.791 | 0.881 | 20.3 | closest to GT-yaw diagnostic |
| `imu_yaw` | -90 | 120 | 216/217 | 0.798 | 0.791 | 0.873 | 21.2 | similar plateau |

This makes the DVL prior path more concrete: real IMU yaw can recover almost
the same DVL direction quality as the GT-yaw diagnostic once a constant yaw
offset is calibrated. The `115 deg` offset above is same-sequence diagnostic
tuning, not a paper-safe calibration. For a benchmark claim, calibrate that
constant on one Tank sequence and validate the DVL-prior fusion on another.

The next diagnostic applies that real DVL/IMU prior to the visual trajectory
instead of only comparing per-step direction. Store the calibrated constants in
a profile so calibration and validation sequences are explicit:

```bash
ros2 run aqua_localization tank_dvl_prior_profile.py \
  --out /tmp/aqua_tank_dvl_prior_profile_short_to_medium.yaml \
  --name tank_short_dvl_prior_diag \
  --calibration-sequence short_test \
  --validation-sequence Medium \
  --calibration-bag /tmp/short_test_ros2_visual \
  --calibration-reference /tmp/tank_short_test_gt.tum \
  --calibration-visual /tmp/aqua_tank_visual_pnp_sweep_1125_strict_ratio/repr_4__ratio_0p85__step_0p02__inl_12__iter_100__conf_0p99/short_test_visual_pnp_1125_strict_ratio_repr_4__ratio_0p85__step_0p02__inl_12__iter_100__conf_0p99_visual_frontend.tum \
  --dvl-frame-yaw-offset-deg -90 \
  --imu-yaw-offset-deg 115 \
  --prior-scale 1.25375 \
  --mode replace-outliers \
  --note "same-sequence diagnostic profile; validate on held-out Tank sequence"

ros2 run aqua_localization apply_tank_dvl_motion_prior.py \
  --profile /tmp/aqua_tank_dvl_prior_profile_short_to_medium.yaml \
  --bag /tmp/short_test_ros2_visual \
  --reference /tmp/tank_short_test_gt.tum \
  --visual /tmp/aqua_tank_visual_pnp_sweep_1125_strict_ratio/repr_4__ratio_0p85__step_0p02__inl_12__iter_100__conf_0p99/short_test_visual_pnp_1125_strict_ratio_repr_4__ratio_0p85__step_0p02__inl_12__iter_100__conf_0p99_visual_frontend.tum \
  --out-dir /tmp/aqua_tank_dvl_motion_prior_apply_profile_check
```

For benchmark claims, run the profile through the validation wrapper instead
of calling the application tool directly. The wrapper records the declared
calibration/validation split, writes a markdown summary, and refuses to run on
the calibration sequence unless the diagnostic override is explicit. It also
writes a benchmark table row that can be fed directly to the gap report:

```bash
ros2 run aqua_localization run_tank_dvl_prior_validation.py \
  --profile /tmp/aqua_tank_dvl_prior_profile_short_to_medium.yaml \
  --sequence Medium \
  --bag /tmp/medium_ros2_visual \
  --reference /tmp/tank_medium_gt.tum \
  --visual /tmp/tank_medium_visual_frontend.tum \
  --max-corrected-rmse-m 0.04 \
  --min-improvement-percent 70 \
  --fail-on-gate-failure \
  --out-dir /tmp/aqua_tank_dvl_prior_validation_medium
```

Compare the generated row against the checked-in AQUA-SLAM baseline rows:

```bash
python3 aqua_localization/scripts/benchmark_gap_report.py \
  docs/benchmarks/tank_aqua_slam.md \
  /tmp/aqua_tank_dvl_prior_validation_medium/tank_dvl_prior_benchmark_row.md \
  --target-system aqua_dvl_prior_visual \
  --baseline-system AQUA-SLAM
```

The same-sequence smoke check remains useful while wiring the prior into the
pipeline, but it must stay labeled as diagnostic:

```bash
ros2 run aqua_localization run_tank_dvl_prior_validation.py \
  --profile /tmp/aqua_tank_dvl_prior_profile_short_to_medium.yaml \
  --sequence short_test \
  --allow-same-sequence \
  --allow-profile-sequence-mismatch \
  --bag /tmp/short_test_ros2_visual \
  --reference /tmp/tank_short_test_gt.tum \
  --visual /tmp/aqua_tank_visual_pnp_sweep_1125_strict_ratio/repr_4__ratio_0p85__step_0p02__inl_12__iter_100__conf_0p99/short_test_visual_pnp_1125_strict_ratio_repr_4__ratio_0p85__step_0p02__inl_12__iter_100__conf_0p99_visual_frontend.tum \
  --max-corrected-rmse-m 0.04 \
  --min-improvement-percent 70 \
  --out-dir /tmp/aqua_tank_dvl_prior_validation_smoke_check
```

That diagnostic smoke currently produces this row on `short_test`:

| Dataset | Sequence | System | Alignment | Samples | Matched s | Mean m | Median m | RMSE m | Max m | Note |
|---------|----------|--------|-----------|--------:|----------:|-------:|---------:|-------:|------:|------|
| Tank Dataset | short_test | aqua_dvl_prior_visual | SE(3) | 218 | 11.10 | 0.0297 | 0.0268 | 0.0323 | 0.0675 | profile=tank_short_dvl_prior_diag; prior=127/217; status=PASS; diagnostic override |

Against the checked-in AQUA-SLAM `short_test` row (`0.0194 m` SE(3) RMSE), that
is a `1.66x` gap and still needs a `39.9%` RMSE reduction to tie. Because the
row uses same-sequence overrides, it is evidence that the prior path can help,
not a benchmark claim.

On 2026-05-23 this profile-based real-prior application reduced the best strict
PnP visual row from `0.1128 m` to `0.0323 m` SE(3) RMSE, a `71.4%` reduction,
while using the DVL/IMU prior on `127/217` visual steps. That is close to the
oracle-prior upper-bound result of `0.0280 m`, but the profile still records
`short_test` as the calibration sequence. Without the DVL scale correction
(`--prior-scale 1.0`), `replace-outliers` improved the row to `0.0597 m`.

| Application | Prior scale | Blend alpha | Corrected RMSE m | Improvement | Prior-applied steps | Readout |
|-------------|------------:|------------:|-----------------:|------------:|--------------------:|---------|
| `replace-outliers` | 1.00000 | n/a | 0.0597 | 47.1% | 128/217 | real prior helps without scale calibration |
| `replace-outliers` | 1.25375 | n/a | 0.0323 | 71.4% | 127/217 | near oracle, same-sequence scale diagnostic |
| `blend-outliers` | 1.25375 | 0.75 | 0.0486 | 56.9% | 127/217 | softer but less accurate on this sequence |

The practical next step is to run the wrapper on the declared held-out sequence.
Only a validation row that passes without `--allow-same-sequence` or
`--allow-profile-sequence-mismatch` should be treated as a benchmark candidate.

When a visual TUM file has already been recorded, pass `--estimate` instead of
`--bag` to regenerate the scale report and benchmark row without replaying ROS.
The bag replay mode also saves `*_visual_frontend_status.csv`, which contains
per-frame feature counts, stereo match counts, triangulated point counts,
disparity/depth statistics, temporal match counts, PnP inliers, inlier ratio,
accepted/rejected state, and the reject reason. Use that CSV to decide whether
the next tuning pass should focus on image features, stereo geometry, temporal
matching, or PnP gates.

When `ros2 bag play` itself is suspected of dropping or corrupting camera
delivery, bypass ROS replay for the visual frontend and read the compressed
images directly from the rosbag2 sqlite file:

```bash
ros2 run aqua_localization run_tank_visual_direct_benchmark.py \
  --bag /tmp/short_test_ros2_visual \
  --reference /tmp/tank_short_test_gt.tum \
  --out-dir /tmp/aqua_tank_visual_direct_scaled_check \
  --sequence short_test_visual_direct_scaled_check \
  --translation-scale 0.151788798 \
  --base-from-camera-x-m -0.25 \
  --base-from-camera-y-m -0.45 \
  --max-stereo-descriptor-distance 64 \
  --max-temporal-descriptor-distance 64 \
  --orb-n-features 700 \
  --orb-fast-threshold 16 \
  --opencv-threads 2
```

Use a direct replay time window when comparing against an AQUA-SLAM row that
only overlaps part of the sequence. The window is applied before stereo pairing,
using `[start, end)` bounds, so the reported pair count, status CSV, TUM file,
and metric table all describe the same slice:

```bash
ros2 run aqua_localization run_tank_visual_direct_benchmark.py \
  --bag /tmp/short_test_ros2_visual \
  --reference /tmp/tank_short_test_gt.tum \
  --out-dir /tmp/aqua_tank_visual_direct_11s_check \
  --sequence short_test_visual_direct_11s_check \
  --start-offset-s 0.0 \
  --duration-s 11.65 \
  --translation-scale 0.151788798 \
  --base-from-camera-x-m -0.25 \
  --base-from-camera-y-m -0.45 \
  --max-stereo-descriptor-distance 64 \
  --max-temporal-descriptor-distance 64 \
  --orb-n-features 700 \
  --orb-fast-threshold 16 \
  --opencv-threads 2
```

On 2026-05-22 this direct replay path read `300` stereo pairs, processed
`300/300` frames, accepted `299` frame-to-frame updates, and had `0` decode
failures on `short_test`. The resulting SE(3) RMSE was `0.1907 m`, so it is not
an accuracy win over the best visual row; it is a replay-isolation baseline that
proves the bag's camera messages are decodable and keeps visual frontend tuning
independent from ROS bag playback delivery issues. Windowed direct replay is
now the preferred way to decide whether a full-sequence RMSE increase is caused
by later visual drift or by comparing against a shorter AQUA-SLAM overlap.
The first windowed check on 2026-05-22 used the first `11.25` seconds of
`short_test` and processed `224/224` stereo pairs. With the previous
same-sequence scale `0.169623465`, direct replay reported `0.1594 m` SE(3)
RMSE; retuning the same window to `tracking.translation_scale=0.105024091`
improved it to `0.1182 m`. That narrows the replay-isolation gap but still does
not reproduce the older `0.0947 m` ROS replay visual row, so the next accuracy
work should compare the direct and ROS visual trajectories on the same window
before adding new frontend logic.

When both ROS replay and direct-replay TUM files are available, compare them
directly before interpreting either one as a frontend accuracy result:

```bash
ros2 run aqua_localization compare_visual_trajectories.py \
  --baseline /tmp/tank_short_test_visual_frontend.tum \
  --target /tmp/aqua_tank_visual_direct_1125_scale0105_check/short_test_visual_direct_1125_scale0105_check_visual_frontend.tum \
  --out /tmp/aqua_tank_visual_direct_1125_scale0105_check/ros_vs_direct_visual_comparison.md \
  --csv /tmp/aqua_tank_visual_direct_1125_scale0105_check/ros_vs_direct_visual_errors.csv \
  --drift-threshold-m 0.05 \
  --drift-consecutive-samples 5
```

The report aligns the target trajectory onto the baseline, writes per-sample
errors, reports raw and aligned path-length ratios, and marks the first
timestamp where the aligned error stays above the drift threshold. Use it to
decide whether the gap comes from timestamp pairing/replay behavior, stale ROS
publishers, or true visual motion-estimation differences.

For a lower-level replay check, compare the visual frontend status CSVs. New
status logs include the left timestamp, right timestamp, and stereo sync delta,
while older status logs still compare by left timestamp and frame index:

```bash
ros2 run aqua_localization compare_visual_status_timing.py \
  --baseline-status /tmp/aqua_tank_visual_base_extrinsic_benchmark/short_test_visual_base_extrinsic_visual_frontend_status.csv \
  --target-status /tmp/aqua_tank_visual_direct_1125_scale0105_check/short_test_visual_direct_1125_scale0105_check_visual_frontend_status.csv \
  --out /tmp/aqua_tank_visual_direct_1125_scale0105_check/ros_vs_direct_status_timing.md \
  --csv /tmp/aqua_tank_visual_direct_1125_scale0105_check/ros_vs_direct_status_timing.csv \
  --timestamp-slop-ms 1.0
```

This writes a correspondence table with frame-index deltas, left timestamp
deltas, stereo sync delta differences, acceptance/status mismatches, and feature
or PnP count deltas. Use it before changing ORB/PnP logic: if the first
timestamp or frame-index mismatch appears early, the benchmark path still needs
replay/pairing cleanup.
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
| Tank Dataset | short_test_visual_direct_scaled_check | aqua_visual_frontend_direct | SE(3) | 300 | 14.95 | 0.1710 | 0.1768 | 0.1907 | 0.3282 | direct rosbag2 sqlite image replay, 300/300 visual frames, scale 0.151788798 |
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

Use [`aqua_slam_error_budget.md`](aqua_slam_error_budget.md) to turn that gap
into the next development budget:

```bash
ros2 run aqua_localization aqua_slam_error_budget.py \
  docs/benchmarks/tank_aqua_slam.md \
  --out docs/benchmarks/aqua_slam_error_budget.md
```

When the best visual run's diagnostics are available, add:

```bash
  --drift-report /tmp/.../short_test_visual_drift.md \
  --motion-report /tmp/.../short_test_visual_motion_segments.md
```

Current readout: the standalone visual row is the accuracy leader, while the
best fused `aqua_localization+visual` row is `0.1228 m` worse than standalone
visual. The next accuracy PR should therefore attack visual covariance, timing,
or coupling rather than another descriptor-only sweep.

Sweep the visual fusion covariance and timing knobs directly:

```bash
ros2 run aqua_localization run_tank_visual_fusion_sweep.py \
  --bag /tmp/short_test_ros2_visual \
  --reference /tmp/tank_short_test_gt.tum \
  --out-dir /tmp/aqua_tank_visual_fusion_covariance_sweep \
  --summary-out /tmp/aqua_tank_visual_fusion_covariance_sweep.md \
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
  --visual-ready-timeout 20 \
  --baseline-rmse-m 0.0194 \
  --standalone-visual-rmse-m 0.0947 \
  --pairs 0.005:0.1,0.01:0.25,0.02:0.5,0.04:1.0
```

The sweep assigns unique visual and fused odometry topics per case, so repeated
bag replays do not mix stale publishers into the recorder. The fusion runner
also waits for the visual frontend status CSV/header before starting bag replay,
which avoids zero-frame runs caused by OpenCV/ORB warmup racing the bag start.
Use the `Delta vs standalone` column as the regression target: the fused result
should move toward `0.0000 m` before claiming visual+DVL progress toward
AQUA-SLAM.

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

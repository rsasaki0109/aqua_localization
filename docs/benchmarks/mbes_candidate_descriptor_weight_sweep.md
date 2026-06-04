# MBES Candidate Descriptor Weight Sweep

Status: `diagnostic, not claimable`

This run checks whether the MBES pre-registration candidate ranking added by
`candidates.descriptor_weight` can improve the loop-closure replay on
MBES-SLAM `beach_pond`. It uses the same 120 s source window for all weights,
with the source MCAP bag copied into a Humble-readable sqlite3 window because
the local ROS 2 Humble environment does not have `rosbag2_storage_mcap`.

The result is useful but not a trajectory claim. The pose graph is worse than
its own input odometry for every row, and the input-odometry RMSE drifts between
replays despite matched-time coverage staying near 119 s. Treat this as a
tuning direction, then rerun with tighter replay determinism and loop audit.

## Command

```bash
OUT_ROOT=/tmp/aqua_mbes_candidate_descriptor_weight_sweep_real \
WEIGHTS=0,0.25,0.5,1.0,2.0 \
MBES_DURATION=120 \
MIN_DURATION_S=60 \
RECORD_READY_TIMEOUT_S=60 \
SWEEP_ROS_DOMAIN_ID_START=90 \
MBES_SRC_PLAY=/tmp/aqua_mbes_candidate_descriptor_weight_sweep_real/mbes_source_humble_sqlite_180s \
ROS_SETUP=/opt/ros/humble/setup.bash \
LOCAL_SETUP=/tmp/aqua_weight_sweep_install/setup.bash \
RECORD_STORAGE=sqlite3 \
IMU_PROFILE=/tmp/aqua_weight_sweep_install/aqua_imu_loc/share/aqua_imu_loc/config/mbes_slam.yaml \
SONAR_PROFILE=/tmp/aqua_weight_sweep_install/aqua_sonar_loc/share/aqua_sonar_loc/config/mbes_slam.yaml \
POSE_GRAPH_PROFILE=/tmp/aqua_weight_sweep_install/aqua_pose_graph/share/aqua_pose_graph/config/params.yaml \
MBES_LOOP_PROFILE=/tmp/aqua_weight_sweep_install/aqua_sonar_loc/share/aqua_sonar_loc/config/mbes_loop_closure.yaml \
MBES_LOOP_MIN_POINTS=120 \
MBES_LOOP_VOXEL_LEAF_M=0.25 \
MBES_LOOP_MIN_KEYFRAME_SEPARATION=40 \
MBES_LOOP_MAX_CORRECTION_ROTATION_RAD=0.2 \
POSE_GRAPH_LOOP_ROBUST_KERNEL_TYPE=dcs \
POSE_GRAPH_LOOP_ROBUST_KERNEL_DELTA=0.1 \
./aqua_localization/scripts/run_mbes_candidate_descriptor_weight_sweep.sh
```

## Summary

Generated summary:
`/tmp/aqua_mbes_candidate_descriptor_weight_sweep_real/mbes_candidate_descriptor_weight_sweep.md`

| Weight | Status | Input RMSE m | Pose graph RMSE m | Graph vs input m | Graph vs baseline m | Graph median m | Matched s | Accepted | Rejected | No candidate |
|-------:|--------|-------------:|------------------:|-----------------:|--------------------:|---------------:|----------:|---------:|---------:|-------------:|
| 0.0000 | baseline | 56.7244 | 67.7353 | -11.0109 | 0.0000 | 52.1436 | 119.38 | 6 | 242 | 122 |
| 0.2500 | check coverage | 62.4073 | 79.9237 | -17.5164 | -12.1884 | 57.0858 | 119.38 | 10 | 157 | 181 |
| 0.5000 | ok | 56.7472 | 72.4731 | -15.7259 | -4.7378 | 58.7766 | 119.35 | 3 | 126 | 100 |
| 1.0000 | check coverage | 51.1297 | 60.6998 | -9.5701 | +7.0355 | 41.3645 | 119.28 | 17 | 339 | 120 |
| 2.0000 | best, check coverage | 46.0288 | 52.8241 | -6.7953 | +14.9112 | 39.3112 | 119.32 | 10 | 203 | 77 |

## Readout

`candidates.descriptor_weight=2.0` produced the lowest pose-graph RMSE in this
diagnostic sweep, improving the pose graph by 14.9112 m versus the `0.0`
baseline replay. The row is still marked `check coverage` because input
odometry RMSE changed by more than 1 m across cases. The next repeat should
pin replay startup more tightly or compare against a source-odometry-driven
pose-graph run before promoting the weight.

## Repeat Sweep

Status: `diagnostic, replay variance dominates`

After adding duplicate-weight support to
`run_mbes_candidate_descriptor_weight_sweep.sh`, a repeated baseline run was
recorded at:
`/tmp/aqua_mbes_candidate_descriptor_weight_repeat_ae57502`

```bash
OUT_ROOT=/tmp/aqua_mbes_candidate_descriptor_weight_repeat_ae57502 \
WEIGHTS=0,0,1.0,2.0 \
MBES_DURATION=120 \
MIN_DURATION_S=60 \
RECORD_READY_TIMEOUT_S=60 \
SWEEP_ROS_DOMAIN_ID_START=100 \
MBES_SRC_PLAY=/tmp/aqua_mbes_candidate_descriptor_weight_sweep_real/mbes_source_humble_sqlite_180s \
ROS_SETUP=/opt/ros/humble/setup.bash \
LOCAL_SETUP=/tmp/aqua_weight_sweep_install/setup.bash \
RECORD_STORAGE=sqlite3 \
IMU_PROFILE=/tmp/aqua_weight_sweep_install/aqua_imu_loc/share/aqua_imu_loc/config/mbes_slam.yaml \
SONAR_PROFILE=/tmp/aqua_weight_sweep_install/aqua_sonar_loc/share/aqua_sonar_loc/config/mbes_slam.yaml \
POSE_GRAPH_PROFILE=/tmp/aqua_weight_sweep_install/aqua_pose_graph/share/aqua_pose_graph/config/params.yaml \
MBES_LOOP_PROFILE=/tmp/aqua_weight_sweep_install/aqua_sonar_loc/share/aqua_sonar_loc/config/mbes_loop_closure.yaml \
MBES_LOOP_MIN_POINTS=120 \
MBES_LOOP_VOXEL_LEAF_M=0.25 \
MBES_LOOP_MIN_KEYFRAME_SEPARATION=40 \
MBES_LOOP_MAX_CORRECTION_ROTATION_RAD=0.2 \
POSE_GRAPH_LOOP_ROBUST_KERNEL_TYPE=dcs \
POSE_GRAPH_LOOP_ROBUST_KERNEL_DELTA=0.1 \
./aqua_localization/scripts/run_mbes_candidate_descriptor_weight_sweep.sh
```

Generated summary:
`/tmp/aqua_mbes_candidate_descriptor_weight_repeat_ae57502/mbes_candidate_descriptor_weight_sweep.md`

| Weight | Status | Input RMSE m | Pose graph RMSE m | Graph vs input m | Graph vs baseline m | Matched s | Accepted | Rejected | No candidate |
|-------:|--------|-------------:|------------------:|-----------------:|--------------------:|----------:|---------:|---------:|-------------:|
| 0.0000 | baseline, best | 57.8057 | 69.4313 | -11.6256 | 0.0000 | 119.39 | 15 | 259 | 112 |
| 0.0000 | baseline repeat, check coverage | 68.8071 | 80.5182 | -11.7111 | -11.0869 | 119.34 | 40 | 370 | 137 |
| 1.0000 | check coverage | 62.0203 | 71.3160 | -9.2957 | -1.8847 | 119.30 | 6 | 172 | 170 |
| 2.0000 | check coverage | 68.7107 | 88.6944 | -19.9837 | -19.2631 | 119.36 | 14 | 318 | 96 |

Baseline repeat spread:

- Input RMSE: `57.8057..68.8071` m, spread `11.0014` m.
- Pose graph RMSE: `69.4313..80.5182` m, spread `11.0869` m.
- Accepted loops: `15..40`, spread `25`.

This invalidates the earlier apparent `2.0` improvement as a descriptor-weight
claim. The next useful work is to reduce replay/pose-graph nondeterminism and
then rerun the same repeated-baseline protocol.

### Slower Playback Probe

After exposing playback controls, the repeated baseline was rerun with
`PLAY_RATE=0.5`, `PLAY_READ_AHEAD_QUEUE_SIZE=10000`, and exact timeout margin
`PLAY_TIMEOUT_MARGIN_S=0`:

`/tmp/aqua_mbes_candidate_descriptor_weight_repeat_slow_exact_a27db0c`

| Weight | Status | Input RMSE m | Pose graph RMSE m | Graph vs input m | Graph vs baseline m | Matched s | Accepted | Rejected | No candidate |
|-------:|--------|-------------:|------------------:|-----------------:|--------------------:|----------:|---------:|---------:|-------------:|
| 0.0000 | baseline, best | 62.7736 | 75.8263 | -13.0527 | 0.0000 | 119.65 | 16 | 595 | 259 |
| 0.0000 | baseline repeat, check coverage | 58.6803 | 75.9117 | -17.2314 | -0.0854 | 119.60 | 10 | 408 | 261 |

Baseline repeat spread:

- Input RMSE: `58.6803..62.7736` m, spread `4.0933` m.
- Pose graph RMSE: `75.8263..75.9117` m, spread `0.0854` m.
- Accepted loops: `10..16`, spread `6`.

Slower playback largely stabilizes pose-graph RMSE, so future descriptor sweeps
should use these playback controls. Input odometry and loop-status counts still
drift enough to keep rows marked `check coverage`; the next target is
node-side input processing determinism and loop identity stability.

### Queue-Depth Probe

After adding `qos.sensor_depth` overrides, the repeated baseline was rerun with
the same slow playback controls but with both MBES profiles using
`qos.sensor_depth=1000`:

`/tmp/aqua_mbes_candidate_descriptor_weight_repeat_qos_55ebc9e`

| Weight | Status | Input RMSE m | Pose graph RMSE m | Graph vs input m | Graph vs baseline m | Matched s | Accepted | Rejected | No candidate |
|-------:|--------|-------------:|------------------:|-----------------:|--------------------:|----------:|---------:|---------:|-------------:|
| 0.0000 | baseline, best | 1848.3796 | 1684.8987 | +163.4809 | 0.0000 | 119.40 | 6 | 87 | 270 |
| 0.0000 | baseline repeat, check coverage | 1899.0143 | 1795.0178 | +103.9965 | -110.1191 | 119.66 | 1 | 13 | 316 |

Baseline repeat spread:

- Input RMSE: `1848.3796..1899.0143` m, spread `50.6347` m.
- Pose graph RMSE: `1684.8987..1795.0178` m, spread `110.1191` m.
- Accepted loops: `1..6`, spread `5`.

This rejects a deep sensor queue as the default replay setting. Keep
`qos.sensor_depth=5` in MBES profiles and use the environment overrides only as
diagnostics when testing callback pressure.

### Endpoint-Stamp Replay Probe

After adding endpoint keyframe stamps to `/mbes_loop_closure/status`, making
benchmark recorder readiness strict, and resolving recorder profiles from the
sourced install prefix, the repeated baseline was rerun with the same shallow
queue and slow playback controls:

`/tmp/aqua_mbes_candidate_descriptor_weight_repeat_endpoint_ef9e097`

| Weight | Status | Input RMSE m | Pose graph RMSE m | Graph vs input m | Graph vs baseline m | Matched s | Accepted | Rejected | No candidate |
|-------:|--------|-------------:|------------------:|-----------------:|--------------------:|----------:|---------:|---------:|-------------:|
| 0.0000 | baseline, best | 53.3113 | 60.9428 | -7.6315 | 0.0000 | 119.61 | 6 | 345 | 258 |
| 0.0000 | baseline repeat, check coverage | 60.7759 | 83.5053 | -22.7294 | -22.5625 | 119.61 | 15 | 297 | 173 |

Baseline repeat spread:

- Input RMSE: `53.3113..60.7759` m, spread `7.4646` m.
- Pose graph RMSE: `60.9428..83.5053` m, spread `22.5625` m.
- Accepted loops: `6..15`, spread `9`.

The profile/recorder fixes are working: strict readiness passed in both runs,
status samples were recorded, and accepted rows now carry finite
`current_keyframe_timestamp` and `candidate_keyframe_timestamp` values directly
from the status message. This does not solve replay variance by itself. The
remaining spread points at candidate evaluation and registration scheduling
rather than missing output-topic recording or missing endpoint identity.

### Timestamp-Buffered Submap Probe

After changing MBES loop-closure submap assembly to buffer point clouds by
message stamp and finalize each MBES-profile submap one keyframe late
(`submaps.finalize_delay_keyframes=1`), the repeated baseline was rerun with
the same shallow queue and slow playback controls:

`/tmp/aqua_mbes_candidate_descriptor_weight_repeat_timestamp_delay_wip_115937`

| Weight | Status | Input RMSE m | Pose graph RMSE m | Graph vs input m | Graph vs baseline m | Matched s | Accepted | Rejected | No candidate |
|-------:|--------|-------------:|------------------:|-----------------:|--------------------:|----------:|---------:|---------:|-------------:|
| 0.0000 | baseline, best | 59.0207 | 64.9449 | -5.9242 | 0.0000 | 119.64 | 13 | 237 | 103 |
| 0.0000 | baseline repeat | 59.0437 | 69.5286 | -10.4849 | -4.5837 | 119.66 | 9 | 292 | 172 |

Baseline repeat spread:

- Input RMSE: `59.0207..59.0437` m, spread `0.0230` m.
- Pose graph RMSE: `64.9449..69.5286` m, spread `4.5837` m.
- Accepted loops: `9..13`, spread `4`.

This is a useful determinism improvement versus the endpoint-stamp probe
(`7.4646` m input RMSE spread, `22.5625` m pose-graph RMSE spread, accepted-loop
spread `9`). It does not make the loop set deterministic: the two accepted-loop
endpoint timestamp sets had zero shared pairs. Keep the timestamp-buffered
submaps, then target candidate/registration ordering before descriptor-weight
tuning.

## Follow-Up

- Use strict recorder readiness, endpoint-stamp status messages, and
  `PLAY_RATE=0.5 PLAY_READ_AHEAD_QUEUE_SIZE=10000` for the next descriptor
  sweep, with repeated baselines included.
- Pin node-side candidate evaluation and registration scheduling determinism
  before tuning descriptor weights again; timestamp-buffered submaps reduce
  RMSE variance but do not align accepted-loop identity yet.
- Keep repeated `0.0` baseline cases in every weight sweep; duplicate weights
  keep the first output name and add `_run2`, for example `weight_0` and
  `weight_0_run2`.
- Audit accepted-loop geometry before accepting the RMSE improvement as useful.
- Add consistency thresholds after loop geometry is reviewed; this sweep has
  no positive consistency guard.

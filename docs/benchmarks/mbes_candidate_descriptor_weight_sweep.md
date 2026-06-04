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

## Follow-Up

- Re-run the best weights `1.0,2.0` with identical startup coverage and a
  repeated `0.0` baseline to estimate replay variance.
- Audit accepted-loop geometry before accepting the RMSE improvement as useful.
- Add consistency thresholds after loop geometry is reviewed; this sweep has
  no positive consistency guard.

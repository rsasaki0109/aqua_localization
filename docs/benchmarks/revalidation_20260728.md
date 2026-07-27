# Tank and MBES Revalidation — 2026-07-28

This note records the first public-data revalidation after the 2026-06-11
work-loss reconstruction. It separates a passing Tank regression result from
the still-failing MBES end-to-end repeatability gate.

## Environment and data

- ROS 2: Jazzy
- Repository: `aqua_localization`
- Tank/MBES replay code: commit `8c9e051`
- MBES comparator executable fix: commit `98a117c`
- Full workspace test result: 703 tests, 0 errors, 0 failures, 0 skipped
- Data and generated artifacts are stored on the external SSD under
  `/media/sasaki/aiueo/datasets/aqua_localization`; the repository paths under
  `datasets/public/` are symlinks, not duplicate data.

| Input | SHA256 |
|-------|--------|
| Tank `short_test.bag` | `e302b75677b4bd3d80c6570831e7d5e4f8283f7193d53bda11237c3ba3540fe8` |
| MBES-SLAM `beach_pond.tar.gz` | `c68d9448502aab9d436e4784453bf07f11663f6719678b01751e4992bfe111c8` |

The converted Tank bag contains 300 left images, 300 right images, 300 ground
truth poses, 4,991 IMU messages, 450 pressure messages, and 104 DVL messages.
The converted MBES bag is 9,474.477 seconds long with 4,102,283 messages and
passes `check_mbes_benchmark_ready.py`.

## Tank `short_test`

The replay used the reconstructed same-run visual regression gate at realtime
1.0x. Calibration remains same-sequence: visual scale `0.169623465` and
base-from-camera translation `(-0.25, -0.45, 0)` m. This is a reconstruction
regression result, not held-out evidence.

| Metric | Result |
|--------|-------:|
| Fused APE RMSE (SE(3)) | **0.1843 m** |
| Mean / median / max | 0.1629 / 0.1445 / 0.7817 m |
| Matched samples / duration | 5,424 / 14.95 s |
| Same-run visual APE RMSE | 0.1959 m |
| Fused minus visual | -0.0116 m |
| Visual coverage | 300/300 (100%) |
| Regression gate | **PASS** |

The current reconstruction improves on the historical 0.2140 m row by
0.0297 m. It does not change the requirement for held-out Tank sequences
before making a broader accuracy or SOTA claim.

Artifacts:

```text
/media/sasaki/aiueo/datasets/aqua_localization/benchmarks/
  tank_short_test_revalidation_20260728/
```

The directory includes the generated benchmark row, coverage report, TUM
trajectories, replay script, and component logs.

## MBES-SLAM `beach_pond`

Two consecutive 120-second replays used the same source, workspace,
`ROS_DOMAIN_ID=42`, and shallow default sensor queues. The preflight process
guard and post-run check were clean. Both trajectories start at the same source
stamp and contain no duplicate timestamps, so the result is not explained by
the leaked-node contamination found in the historical probes.

| Metric | Run 1 | Run 2 |
|--------|------:|------:|
| Input odometry samples | 11,964 | 11,955 |
| Input APE RMSE (SE(3)) | 120.0266 m | 204.7904 m |
| Pose graph APE RMSE (SE(3)) | 145.2342 m | 263.0150 m |
| Loop-status samples | 1,102 | 1,064 |
| Accepted loops | 0 | 0 |
| Sonar feedback received / applied | 25 / 24 | 28 / 24 |
| Stale sonar feedback | 1 | 4 |

The raw common-window trajectory comparison covers 119.5 seconds and reports a
mean difference of **184.4686 m** and a maximum difference of **967.2658 m**.
The required mean-difference gate of 1 m therefore **FAILS**. All loop attempts
were rejected before registration because the downsampled submaps had fewer
than the configured 300-point minimum; the matching zero accepted-loop sets
are vacuous and are not loop-closure evidence.

Artifacts:

```text
/media/sasaki/aiueo/datasets/aqua_localization/benchmarks/
  mbes_repeat_20260728/
    mbes_repeat_probe_compare.md
    run1/
    run2/
```

Reproduce the two-run probe after sourcing the workspace:

```bash
export WORKSPACE="$PWD"
export MBES_SRC="$PWD/datasets/public/mbes_slam/beach_pond_ros2"
export PROBE_OUT=/path/on/external/ssd/mbes_repeat
export MBES_DURATION=120
export ROS_DOMAIN_ID=42
export GEOMETRY_AUDIT_REQUIRE_COMPLETE=0
bash aqua_localization/scripts/probe_sonar_feedback_repeatability.sh
```

The command is expected to exit nonzero while the repeatability defect remains;
the recorded bags and reports are still complete and usable for diagnosis.

## Current interpretation

- Tank reconstruction and its same-run regression gate are validated.
- Dataset acquisition, conversion, status export, and the two-run MBES driver
  are operational.
- MBES Phase 1 remains open: the sonar-registration-to-UKF feedback path is
  still timing/order sensitive, even with stamp buffering and clean process
  hygiene.
- No MBES accuracy, repeatability, loop-closure, or SOTA claim is supported by
  this run.

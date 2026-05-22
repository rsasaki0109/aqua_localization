# Real-Bag Evaluation Run Sheet

This report is generated from the benchmark manifest. Use it to keep
real-data comparisons reproducible before turning results into README or
paper claims.

Manifest: `docs/benchmarks/real_bag_evaluation_manifest.json`

| Case | Status | Target | Baselines | Metrics | Next step |
|------|--------|--------|-----------|---------|-----------|
| Tank Dataset `short_test` | measured | aqua_localization+visual | AQUA-SLAM | translation APE RMSE, matched seconds, visual coverage, replay rate | Beat or narrow the AQUA-SLAM SE(3) RMSE gap on the same short_test window before claiming accuracy superiority. |
| MBES-SLAM `beach_pond` | measured | aqua_localization MBES frontend | dataset reference odometry, no-sonar fusion | translation APE RMSE, registration residual, loop accepted/rejected/no-candidate counts, descriptor gate pass rate | Audit the 35 accepted loops from the 120 s tuned beach_pond replay in RViz/rerun before using the row as accuracy evidence. |
| NTNU `subset-fjord/fjord_1` | measured | aqua_imu_loc | dataset baseline trajectory, future robot_localization IMU-only config | translation APE RMSE, depth RMSE, yaw-frame diagnostic | Add robot_localization as a generic fusion baseline for the same inputs. |
| AQUALOC `harbor_07` | planned | aqua_localization visual frontend | RTAB-Map, dataset reference trajectory if usable | visual odometry availability, translation APE RMSE, depth RMSE, tracking dropout count | Confirm camera calibration and reference trajectory before adding a visual SLAM baseline. |

## Case Details

### Tank Dataset `short_test`

- Status: `measured`
- Comparison group: visual-dvl-imu SLAM
- Target system: `aqua_localization+visual`
- Baselines: `AQUA-SLAM`
- Inputs: stereo, IMU, DVL, pressure
- Reference: AprilTag ground truth TUM trajectory
- Artifacts: `docs/benchmarks/tank_aqua_slam.md`, `visual frontend status CSV`, `TUM trajectories`

```bash
ros2 run aqua_localization run_tank_visual_fusion_benchmark.py --profile tank_short_test_light --replay-rate 1.0
```

Fairness notes:
- AQUA-SLAM uses the full stereo + IMU + DVL frontend.
- Report SE(3) alignment and sample count with every row.

### MBES-SLAM `beach_pond`

- Status: `measured`
- Comparison group: MBES registration and loop diagnostics
- Target system: `aqua_localization MBES frontend`
- Baselines: `dataset reference odometry`, `no-sonar fusion`
- Inputs: MBES point returns, IMU, reference odometry
- Reference: dataset reference odometry when available
- Artifacts: `datasets/mbes_slam_beach_pond_acquisition.md`, `docs/benchmarks/mbes_beach_pond_loop_status.md`, `run_mbes_loop_benchmark.sh`, `readiness report`, `rerun export`, `RViz loop markers`, `mbes_loop_status.csv`, `descriptor sweep report`, `accepted-loop audit report`

```bash
MBES_LOOP_MIN_POINTS=120 MBES_LOOP_VOXEL_LEAF_M=0.25 ros2 run aqua_localization run_mbes_loop_benchmark.sh
```

Fairness notes:
- Do not compare this as a full visual SLAM task.
- Keep MBES loop closure claims separate from generic fusion baselines.
- The measured row is a loop-status tuning result, not a validated full-trajectory SLAM win.

### NTNU `subset-fjord/fjord_1`

- Status: `measured`
- Comparison group: IMU and pressure control case
- Target system: `aqua_imu_loc`
- Baselines: `dataset baseline trajectory`, `future robot_localization IMU-only config`
- Inputs: IMU, pressure
- Reference: dataset baseline trajectory
- Artifacts: `docs/benchmarks/fjord_1.md`, `docs/benchmarks/fjord_1_yaw_frame.md`

```bash
ros2 run aqua_localization bench_fjord_1.sh
```

Fairness notes:
- This is a control case, not a strong SLAM win condition.
- Use it to show why aiding sensors matter underwater.

### AQUALOC `harbor_07`

- Status: `planned`
- Comparison group: future visual localization
- Target system: `aqua_localization visual frontend`
- Baselines: `RTAB-Map`, `dataset reference trajectory if usable`
- Inputs: stereo, IMU, pressure
- Reference: TBD
- Artifacts: `camera calibration notes`, `TUM trajectories`, `tracking status CSV`

_No replay command is pinned yet._

Fairness notes:
- Only compare RTAB-Map after valid visual inputs are configured.
- Do not use sonar-only results as a visual SLAM superiority claim.

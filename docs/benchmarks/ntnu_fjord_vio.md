# NTNU fjord_1: aqua_graph Stereo VIO (2026-10-11)

Cross-dataset check of the Tank-developed stack on a different vehicle,
sensor suite and environment: the NTNU multi-camera underwater
visual-inertial dataset
([huggingface.co/datasets/ntnu-arl/underwater-datasets](https://huggingface.co/datasets/ntnu-arl/underwater-datasets),
BSD-3-Clause), Trondheim fjord sequence `fjord_1` (142 m, 312 s, BlueROV2
Heavy-class *Ariel*, manually piloted).

**What the reference is.** `fjord_1_baseline.tum` is ReAqROVIO (refractive
ROVIO) run with **four cameras** and the IMU, not ground truth. The numbers
below measure agreement with that published VIO, using one stereo pair.

| Method | Inputs | RMSE m | Max m | Final m | drift 10 m % | drift 50 m % | drift 100 m % |
|--------|--------|------:|------:|------:|------:|------:|------:|
| aqua_graph (online frontend) | cam1/cam0 stereo + Alphasense IMU | **0.375** | 0.793 | 0.674 | 2.93 | 1.05 | 0.73 |
| aqua_imu_loc UKF (history, [`fjord_1.md`](fjord_1.md)) | MAVROS IMU + "pressure" | ~800-1300 | | | | | |

SE(3)-aligned APE over the whole 312 s; drift is the world-frame segment
error in % of path length (`evaluate_replay_trajectories.py
--segment-lengths`). Deterministic: two replays give the same fingerprint
(`6c81ee5b2c1284db` with the landmark prior; `19c999b16ecfe3b0` before it). 312 s of data replay in ~200 s on 4 vCPU,
single-threaded, including image decoding and tracking.

## All six fjord sequences

Same settings, nothing tuned per sequence; every bag is streamed from the
dataset repository into the converter (`--src-url`, no local raw copy),
replayed deterministically and scored against its own ReAqROVIO reference:

```bash
ros2 run aqua_replay run_ntnu_benchmark.py --calib-dir <calib> \
  --work-dir <dir> --sequences fjord_1 fjord_2 fjord_3 fjord_4 fjord_5 fjord_6 --cleanup
```

<!-- generated:begin -->
| Sequence | Path m | Duration s | RMSE m | Max m | Final m | drift 10 m % | drift 50 m % | drift 100 m % | Fingerprint |
|----------|------:|------:|------:|------:|------:|------:|------:|------:|------|
| `fjord_1` | 142 | 312 | 0.375 | 0.793 | 0.674 | 2.93 | 1.05 | 0.73 | `6c81ee5b2c1284db` |
| `fjord_2` | 207 | 500 | 0.731 | 1.353 | 1.214 | 4.87 | 1.98 | 0.96 | `38bcec08cc5c830e` |
| `fjord_3` | 123 | 272 | 0.358 | 0.569 | 0.436 | 3.11 | 1.03 | 0.70 | `4e69fcec7edfafc5` |
| `fjord_4` | 166 | 411 | 0.576 | 1.167 | 1.149 | 3.92 | 1.36 | 1.08 | `6f8609b8cbf39ca6` |
| `fjord_5` | 235 | 441 | 2.332 | 5.694 | 5.601 | 12.79 | 5.50 | 2.32 | `7d7156a6a29acc77` |
| `fjord_6` | 306 | 639 | 4.139 | 10.976 | 10.717 | 13.74 | 7.02 | 3.44 | `993bbf4b5799c0b9` |
<!-- generated:end -->

1.9 km of trajectory in total; ~55 min wall for all six on 4 vCPU,
including streaming ~115 GB of raw bags.

- **fjord_1-4: 0.36-0.73 m RMSE, 0.70-1.08 % drift per 100 m.**
- **fjord_5 / fjord_6: 2.3 m / 4.1 m (2.3 % / 3.4 % per 100 m).** The first
  run of these two re-anchored the fixed-lag smoother 10+ times; logging the
  cause showed `IndeterminantLinearSystem` at a landmark: GTSAM's stereo
  factor returns zero Jacobians for points behind the camera, leaving such
  a landmark unconstrained. A weak 10 m prior on each new landmark
  (`visual.landmark_prior_sigma_m`, now the default) removed every reset and
  improved fjord_5 2.44 -> 2.33 m and fjord_6 4.79 -> 4.14 m (fjord_1-4 within
  +-0.02 m; Tank `short_test` unchanged). The remaining error on these two
  is drift between resets, not a failure, and is still open.

## Reproduce

```bash
# fjord_1.bag (12.5 GB), fjord_1_baseline.tum and the cam0_cam1_stereo
# calibration from the dataset repository, then:
ros2 run aqua_localization convert_ntnu_stereo_bag.py \
  --src fjord_1.bag --reference fjord_1_baseline.tum \
  --intrinsics camchain-stereo-intrinsics-underwater.yaml \
  --imu-extrinsics camchain-imucam-stereo-extrinsics-air.yaml \
  --dst fjord_1_stereo_ros2 --params-out fjord_1_geometry.yaml
ros2 run aqua_replay offline_replay \
  --scenario $(ros2 pkg prefix aqua_replay)/share/aqua_replay/scenarios/ntnu_fjord_vio.yaml \
  --bag fjord_1_stereo_ros2 --out /tmp/fjord_vio
ros2 run aqua_replay evaluate_replay_trajectories.py --bag /tmp/fjord_vio \
  --reference /reference/odometry --estimate /aqua_graph/odometry \
  --segment-lengths 10 50 100
```

The converter rectifies the equidistant (fisheye) pair with the in-water
intrinsics (cam1 is the left camera: Kalibr's `T_cn_cnm1` has +0.110 m in
x) and writes the rectified geometry overlay
`aqua_graph/config/ntnu_fjord_stereo_geometry.yaml` (fx 418.1 px, baseline
0.1097 m, body_T_camera from the Kalibr IMU-camera calibration).

## Settings (nothing tuned on fjord_1)

- Frontend: same tracker and defaults as Tank, 150 features, every 2nd
  frame published (10 Hz).
- IMU noise: the dataset's published `imu0_alphasense_noise.yaml`
  (Kalibr values). A 10x tighter bias random walk was tried and was worse.
- No DVL (none on the vehicle) and **no depth**, see below.

## Finding: the "pressure" topic is not water depth

`/mavros/imu/static_pressure` moves by only +-0.16 m-equivalent while the
reference trajectory climbs ~7 m at the end of `fjord_1`: it is the hull's
internal barometer, not the water-depth sensor. Fused as depth
(sigma 0.05 m) it pins the vertical and the estimate diverges in the last
seconds (RMSE 3.06 m, final 49.7 m, accelerometer bias pulled to 2-3 m/s^2).
`datasets/ntnu_demo.md` and the UKF's `aqua_imu_loc/config/ntnu_fjord.yaml`
both treat it as water pressure; the UKF history in `fjord_1.md` was
measured with it.

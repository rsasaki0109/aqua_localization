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
| aqua_graph (online frontend) | cam1/cam0 stereo + Alphasense IMU | **0.379** | 0.806 | 0.685 | 2.93 | 1.07 | 0.73 |
| aqua_imu_loc UKF (history, [`fjord_1.md`](fjord_1.md)) | MAVROS IMU + "pressure" | ~800-1300 | | | | | |

SE(3)-aligned APE over the whole 312 s; drift is the world-frame segment
error in % of path length (`evaluate_replay_trajectories.py
--segment-lengths`). Deterministic: two replays give fingerprint
`19c999b16ecfe3b0`. 312 s of data replay in ~200 s on 4 vCPU,
single-threaded, including image decoding and tracking.

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

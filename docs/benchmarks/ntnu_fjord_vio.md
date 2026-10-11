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
| aqua_graph (online frontend) | cam1/cam0 stereo + Alphasense IMU | **0.320** | 0.630 | 0.506 | 3.81 | 0.87 | 0.54 |
| aqua_graph, frontend rectifying online | raw fisheye cam1/cam0 + IMU | 0.330 | 0.661 | 0.384 | 3.84 | 0.91 | 0.54 |
| aqua_imu_loc UKF (history, [`fjord_1.md`](fjord_1.md)) | MAVROS IMU + "pressure" | ~800-1300 | | | | | |

SE(3)-aligned APE over the whole 312 s; drift is the world-frame segment
error in % of path length (`evaluate_replay_trajectories.py
--segment-lengths`). Deterministic: two replays give the same fingerprint
(`786a451e00bdbec0`; before CLAHE + glare mask 0.375 m,
`6c81ee5b2c1284db`). 312 s of data replay in ~200 s on 4 vCPU,
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
| `fjord_1` | 142 | 312 | 0.320 | 0.630 | 0.506 | 3.81 | 0.87 | 0.54 | `786a451e00bdbec0` |
| `fjord_2` | 207 | 500 | 0.689 | 1.526 | 0.592 | 4.87 | 2.18 | 0.81 | `89dd83b3463fcbd4` |
| `fjord_3` | 123 | 272 | 0.286 | 0.595 | 0.310 | 3.69 | 0.89 | 0.37 | `7d9150143a5ebe8c` |
| `fjord_4` | 166 | 411 | 0.559 | 1.213 | 0.970 | 4.34 | 1.33 | 0.83 | `8a1d37852bb3d7e5` |
| `fjord_5` | 235 | 441 | 0.637 | 1.325 | 0.581 | 3.86 | 1.70 | 0.82 | `a20833f882f5143e` |
| `fjord_6` | 306 | 639 | 0.904 | 1.748 | 0.625 | 4.54 | 2.40 | 0.93 | `ced91d0ae4c2eb91` |
<!-- generated:end -->

1.9 km of trajectory in total; ~55 min wall for all six on 4 vCPU,
including streaming ~115 GB of raw bags.

- **All six: 0.29-0.90 m RMSE, 0.37-0.93 % drift per 100 m** (before
  the CLAHE + glare mask fix below: 0.36-4.14 m, 0.70-3.44 %). Every
  sequence improved in RMSE and 100 m drift; 10 m drift got slightly worse
  on fjord_1/3/4 (e.g. fjord_1 2.93 -> 3.81 %), CLAHE amplifies image
  noise at short range.
- History: before the drift fix below, fjord_5 / 6 were at 2.33 m / 4.14 m
  (2.3 % / 3.4 % per 100 m), and before that 2.44 m / 4.79 m with 10+
  fixed-lag smoother re-anchors each: GTSAM's stereo factor returns zero
  Jacobians for points behind the camera, leaving such a landmark
  unconstrained (`IndeterminantLinearSystem`). A weak 10 m prior on each new
  landmark (`visual.landmark_prior_sigma_m`, default) removed every reset.

## fjord_5 / 6 drift: hazy images and sun glare

The remaining fjord_5 / 6 error was almost all yaw: about -30 deg / -38 deg
at the end, against a reference that turns over 1000 deg. What it was not:

- *Not the smoother*: no resets, and a 10x tighter gyro bias random walk
  only took fjord_5 from 2.33 to 2.09 m.
- *Not static features*: no track sits on the rectification border or
  stays fixed in the image.
- *Not the focal length*: scaling the backend's fx by 0.97 helps fjord_5 / 6
  (1.53 / 2.72 m) but ruins fjord_1 (0.375 -> 1.06 m, +18 deg yaw). An
  air-calibration + Snell check also agrees with the in-water calibration to
  about 1 %.

Where it was: the yaw error grows in steps of 2-3 deg exactly where
tracking collapses, with 30-45 features and more than 1000 new track ids/s
in frames that are hazy and have sun glare at the top. Stereo PnP between
consecutive frames finds clearly wrong rotations in a handful of those
frames. Equalizing contrast (CLAHE, clip 2.0) and keeping features out of
saturated pixels (`saturation_level` 245) fixes it:

| fjord_5 | RMSE m | Final m | drift 100 m % | final yaw err deg |
|---------|------:|------:|------:|------:|
| before | 2.332 | 5.601 | 2.32 | -29.1 |
| glare mask only | 2.349 | 6.057 | 2.33 | -29.3 |
| CLAHE only | 0.756 | 0.852 | 0.96 | -7.0 |
| CLAHE + glare mask (now the NTNU default) | **0.637** | **0.581** | **0.82** | -4.5 |

Both are `aqua_visual_frontend` parameters (`clahe_clip_limit`,
`saturation_level`), off by default. The NTNU scenarios turn them on. The
Tank scenario leaves them off: on Tank `short_test` (dehazed images) they
change nothing (0.0099 m either way). They were chosen once on fjord_5,
not swept, and the six-sequence table above is with them on.

## Online fisheye rectification

`aqua_visual_frontend` can rectify the raw equidistant pair itself, so a live
vehicle needs no offline converter. It takes `input_format: raw`
(sensor_msgs/Image: mono8/16, bgr/rgb(a)8, Bayer 8-bit) or `compressed`, and
`rectify.camchain` / `left_camera` / `right_camera` / `balance`. The same
cv::fisheye rectification as the converter yields the same geometry on this
calibration (fx 418.125, baseline 0.10968 m, logged at startup). The
backend still needs the matching `ntnu_fjord_stereo_geometry.yaml` overlay.

```bash
ros2 run aqua_replay run_ntnu_benchmark.py --calib-dir <calib> \
  --work-dir <dir> --sequences fjord_1 --online-rectify
```

This converts with `--no-rectify` (raw fisheye JPEGs on
`/camera/{left,right}/image_raw/compressed`) and replays
`ntnu_fjord_vio_online_rectify.yaml`. On fjord_1 it scores **0.330 m** RMSE
(offline-rectified: 0.320 m; the JPEG step sits before rather than after
rectification). Two runs give the same fingerprint
(`000a45e0772913ce`).

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

## Settings

- Frontend: same tracker as Tank, 150 features, every 2nd frame published
  (10 Hz), plus CLAHE (clip 2.0) and the glare mask (level 245), chosen on
  fjord_5 as described above.
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

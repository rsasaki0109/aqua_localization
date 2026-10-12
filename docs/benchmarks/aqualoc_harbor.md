# AQUALOC Harbor: Monocular Visual-Inertial-Pressure Odometry (2026-10-12)

[AQUALOC](http://www.lirmm.fr/aqualoc/) (Ferrera et al., IJRR 2019) is a
public underwater visual-inertial-pressure dataset: an ROV with a monocular
uEye camera (640x512, 20 Hz, pinhole-equidistant), an MPU-9250 IMU
(200 Hz, no orientation output) and a barometer (~5 Hz), no DVL. Ground
truth is COLMAP camera poses for every 5th image, rescaled in 2021
(`new_harbor_colmap_traj_sequence_0N.txt`). This is the stack's first
benchmark with published results from other systems on several sequences;
Tank (AQUA-SLAM) has only one public sequence.

## Pipeline

`aqualoc_harbor_vio.yaml`: `stereo_frontend` in `mono: true` mode
(undistorts the equidistant camera, KLT tracking, CLAHE, every 2nd frame,
`stamp_offset_s` = Kalibr `timeshift_cam_imu`) -> `aqua_graph` with
monocular landmarks (triangulated after 3 sightings with >= 0.03 rad
parallax inside the 4 s window, projection factors), pressure depth
(first sample as reference, Huber against a -267 m glitch) and a weak
zero-velocity prior until 30 landmarks exist. Kalibr calibration from the
dataset. Evaluation: `evaluate_aqualoc.py` maps the ground-truth image
indices to the bag's image stamps (+ time shift) and reports position ATE
after SE(3) and Sim(3) alignment. No loop closure, no map reuse.

## Results (all 7 harbor sequences, ATE RMSE m)

| Seq | Length m | aqua_graph SE3 | aqua_graph Sim3 | Sim3 scale | ORB-SLAM3-VI | ORB-SLAM3-VIP |
|----:|------:|------:|------:|------:|------:|------:|
| 01 | 36.7 | 0.330 | 0.242 | 1.053 | 0.437 | 0.113 |
| 02 | 68.9 | 0.331 | 0.327 | 1.011 | 0.081 | 0.054 |
| 03 | 23.0 | 0.485 | 0.162 | 0.789 | 0.108 | 0.051 |
| 04 | 47.7 | 0.511 | 0.313 | 0.924 | 0.140 / 0.237 | 0.098 / 0.151 |
| 05 | 27.1 | 0.370 | 0.361 | 1.024 | 0.076 | 0.047 |
| 06 | 11.7 | 0.324 | 0.272 | 0.869 | 0.057 | 0.026 |
| 07 | 23.9 | 0.271 | 0.236 | 0.958 | 0.081 / 0.203 | 0.044 / 0.108 |

ORB-SLAM3-VI / -VIP: Lu et al., "A Visual-Inertial Pressure Fusion-Based
Underwater SLAM System", Sensors 24(10) 3207, 2024, Table 2 (two values
where tracking was lost and restarted). They do not state the alignment
(their scale-error table suggests Sim3) or which ground-truth version they
used, so the comparison is indicative.

Every sequence tracks to the end without a smoother reset and the replays
are deterministic, but **aqua_graph is 3-7x behind ORB-SLAM3-VIP and
behind ORB-SLAM3-VI everywhere except harbor 01. This is not a SOTA
result.**

## Why: where the gap comes from (measured 2026-10-12)

- **Not map reuse.** COLMAP's image-overlap matrices give how often a
  sequence looks again at a place seen more than 10 s earlier: harbor 01
  41 %, 02 65 %, 03 79 %, 04 13 %, **05 1 %**, 06 20 %, 07 41 %. Harbor 05
  never revisits and is as far behind ORB-SLAM3-VIP (0.36 vs 0.047 m) as
  the others.
- **The start-up.** Harbor 05, aligned on its first 20 s: the error grows
  to ~1 m by 42 s and then stays flat to the end. Excluding the first 60 s
  the ATE is 0.023 m Sim3 (0.161 m SE3): the monocular scale is
  underestimated by ~25 % for the first ~40 s (estimated speed 0.10 vs
  0.14 m/s), converges once the ROV moves more, and the 4 s fixed-lag
  window cannot go back and correct the start.
- **Smoothing helps but does not close it.** The smoother's own final
  estimates (`topics.smoothed_odometry`) are barely better than the
  real-time output (harbor 05 0.350 vs 0.361 m). Keeping every state
  (`graph.smoother_lag_s` longer than the run, `topics.window_path`): harbor
  06 0.153 m SE3 / Sim3 (scale 0.998), vs 0.27 m fixed-lag and 0.057 /
  0.026 m for ORB-SLAM3-VI / -VIP, at ~7x slower than real time. A longer
  window during start-up only (`graph.initial_lag_s`) is inconsistent:
  harbor 06 0.17 m, harbor 05 0.27-0.92 m.
- **What is left** is the quality of the visual constraints: KLT tracks
  live a few seconds, so each landmark is seen from few, close viewpoints.
  ORB-SLAM3 bundle-adjusts keyframes with long-lived map points. Closing
  the gap means a keyframe-based visual SLAM back end, not incremental
  changes to this one.
- **Map reuse was tried** (branch `claude/map-reuse`, not merged): ORB
  descriptors per track, re-identification of lost tracks, persistent
  landmarks, camera-centre consensus, search by projection. On harbor 06
  448 of 3000 new tracks had a map point projecting within 40 px, but the
  best ORB Hamming distance was mostly 80-100 (near random) and only 4
  were <= 50: ORB at KLT corners does not re-identify points on this
  imagery.
- Also: the Kalibr `timeshift_cam_imu` sign is uncertain on this data
  (harbor 05: -0.04 s 0.36 m, 0 s 0.29 m, +0.04 s 0.27 m Sim3); left at
  the Kalibr convention.
- Initialization: the ROV is already moving and the IMU gives no
  orientation; without the bootstrap velocity prior some sequences diverged
  before the first landmarks existed (0.5 s attitude window: harbor 06/07,
  2 s: harbor 02/05).

## Reproduce

```bash
# from the AQUALOC share (Harbor_sites_sequences/harbor_sequence_0N_bag.tar.gz,
# harbor_groundtruth_files/new_harbor_colmap_traj_sequence_0N.txt)
rosbags-convert --src harbor_sequence_1.bag --dst harbor_01_ros2 \
  --dst-storage mcap --src-typestore ros1_noetic --dst-typestore ros2_jazzy \
  --include-topic /rtimulib_node/imu /camera/image_raw /barometer_node/pressure
ros2 run aqua_replay offline_replay \
  --scenario $(ros2 pkg prefix aqua_replay)/share/aqua_replay/scenarios/aqualoc_harbor_vio.yaml \
  --bag harbor_01_ros2 --out /tmp/aqualoc_harbor_01
ros2 run aqua_replay evaluate_aqualoc.py --replay /tmp/aqualoc_harbor_01 \
  --source harbor_01_ros2 --gt new_harbor_colmap_traj_sequence_01.txt
```

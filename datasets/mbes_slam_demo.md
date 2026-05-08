# MBES-SLAM beach_pond — Public Demo Note (`aqua_sonar_loc`)

This note records the public-data bringup of `aqua_sonar_loc` against the **MBES-SLAM**
public bathymetry dataset, which ships a real multibeam sonar (Norbit iWBMSh) point cloud
recorded from a small surface vessel surveying a coastal pond. It replaces the previous
synthetic-bag demo as the primary `aqua_sonar_loc` bringup story.

## Source

- Dataset: MBES-SLAM (POS group, IJRR 2021)
- Paper: Hammond, Rowley, Roman, _Bathymetric SLAM with Multibeam Echosounders Using Submap-Based Loop Closure_, IJRR 2021.
- Download (single tar):
  - `https://seaward.science/files/pos-datasets/bag/beach_pond.tar.gz` (≈ 2.5 GB compressed, 6.8 GB after extract)
- Sequence: `beach_pond` (≈ 158 min, surface vessel above a coastal pond)
- Platform: surface vessel with Norbit iWBMSh multibeam, Microstrain 3DM-GX5-25 IMU, Nortek DVL, ublox GNSS, Wassp CTD.
- License: research dataset, see the seaward.science landing page for redistribution terms.

```bash
mkdir -p aqua_localization/datasets/public/mbes_slam
cd aqua_localization/datasets/public/mbes_slam
wget --no-check-certificate https://seaward.science/files/pos-datasets/bag/beach_pond.tar.gz
tar xzf beach_pond.tar.gz
```

After extraction:

```text
aqua_localization/datasets/public/mbes_slam/beach_pond/beach_pond.bag             # 6.8 GB ROS 1
aqua_localization/datasets/public/mbes_slam/beach_pond/20200717_beach_pond.surv   # site metadata
aqua_localization/datasets/public/mbes_slam/beach_pond/ctd/                       # CTD profiles (CSV)
```

The site-metadata file pins the dataset origin:

```text
utm_zone: 19
origin_z: 61.82
origin_lon: -71.78644
origin_lat: 41.574459
```

## Convert ROS 1 → ROS 2 (subset only)

The bag carries 14.6 M messages across 70+ topics (≈ 9474 s). Most are vendor-specific
custom messages (`pyublox_msgs`, `ds_sensor_msgs`, `norbit_msgs`, `mscl_msgs`,
`pos_msgs`, `ds_core_msgs`) that have no upstream ROS 2 typestore. We include only
the standard-typed topics needed by the localization stack:

```bash
rosbags-convert \
  --src aqua_localization/datasets/public/mbes_slam/beach_pond/beach_pond.bag \
  --dst aqua_localization/datasets/public/mbes_slam/beach_pond_ros2 \
  --dst-storage mcap \
  --src-typestore ros1_noetic \
  --dst-typestore ros2_jazzy \
  --include-topic /norbit/detections \
                  /nav/sensors/microstrain/imu/raw \
                  /nav/processed/microstrain/imu/madgwick \
                  /nav/sensors/microstrain/mag/raw \
                  /nav/processed/odometry \
                  /nav/sensors/navsat/ubx_pos/fix \
                  /tf /tf_static
```

## Detected topics (after conversion)

| Topic                                       | Type                              | Messages | Rate (Hz) |
|---------------------------------------------|-----------------------------------|---------:|----------:|
| `/norbit/detections`                        | `sensor_msgs/msg/PointCloud2`     |  125 488 |    13.2   |
| `/nav/sensors/microstrain/imu/raw`          | `sensor_msgs/msg/Imu`             |  947 459 |   100.0   |
| `/nav/processed/microstrain/imu/madgwick`   | `sensor_msgs/msg/Imu` (AHRS quat) |  947 356 |   100.0   |
| `/nav/sensors/microstrain/mag/raw`          | `sensor_msgs/msg/MagneticField`   |  947 458 |   100.0   |
| `/nav/processed/odometry`                   | `nav_msgs/msg/Odometry`           |  472 530 |    50.0   |
| `/nav/sensors/navsat/ubx_pos/fix`           | `sensor_msgs/msg/NavSatFix`       |   47 369 |     5.0   |
| `/tf`, `/tf_static`                         | `tf2_msgs/msg/TFMessage`          | 614 622 + 1 | — |

Roles for `aqua_sonar_loc`:

- **Sonar (multibeam fan)**: `/norbit/detections` → `aqua_sonar_loc` PointCloud2 input.
- **Ground-truth odometry**: `/nav/processed/odometry` (full ROS 1 EKF that fuses
  IMU + GPS + DVL). Use as the reference for `compare_trajectories.py`.

## Sequence-specific config

`aqua_sonar_loc/config/mbes_slam.yaml` is an MBES-SLAM-specific starter:

- `topics.points: /norbit/detections`
- `frames.sonar: norbit`
- `scan_matching.backend: gicp` (more robust than ICP on the noisy multibeam fans)
- `preprocessing.max_range_m: 60.0`, `min_points: 60` (Norbit fans are dense, hundreds
  of beams per ping)
- Quality gates `max_fitness_score: 1.5`, `max_translation_step_m: 0.6`,
  `max_rotation_step_rad: 0.4` reject ICP runaway without clipping legitimate
  ~1 m/s, low-yaw-rate surface motion.

It is installed automatically by `aqua_sonar_loc` (the whole `config/` directory is
exported in `CMakeLists.txt`).

## Replay command (sonar-only, GICP backend)

The bag is long (≈ 158 min). For a quick demo, `ros2 bag play --duration` slices a window:

```bash
ros2 launch aqua_localization replay.launch.py \
  start_bag:=true \
  bag_path:=aqua_localization/datasets/public/mbes_slam/beach_pond_ros2 \
  bag_sonar_points_topic:=/norbit/detections \
  use_sim_time:=true \
  enable_imu_loc:=false \
  enable_fusion:=false \
  sonar_params_file:=$(ros2 pkg prefix aqua_sonar_loc)/share/aqua_sonar_loc/config/mbes_slam.yaml
```

To trim to a 120 s window (instead of replaying the full 9474 s), drive `ros2 bag play`
manually with `--duration` and disable `start_bag`:

```bash
# Terminal A: localization stack (no bag).
ros2 launch aqua_localization replay.launch.py \
  start_bag:=false \
  use_sim_time:=true \
  enable_imu_loc:=false \
  enable_fusion:=false \
  sonar_params_file:=$(ros2 pkg prefix aqua_sonar_loc)/share/aqua_sonar_loc/config/mbes_slam.yaml \
  bag_sonar_points_topic:=/norbit/detections

# Terminal B: bag (jazzy uses --playback-duration, not --duration).
ros2 bag play aqua_localization/datasets/public/mbes_slam/beach_pond_ros2 \
  --clock --start-offset 60 --playback-duration 120
```

## Verification topics

```bash
ros2 topic echo /aqua_sonar_loc/status --once
ros2 topic echo /aqua_sonar_loc/odometry --once | head -40
ros2 topic hz /aqua_sonar_loc/odometry
```

A representative `/aqua_sonar_loc/status` sample on this bag (CSV via
`ros2 topic echo --csv`):

```text
1595011683,719077000,norbit,gicp,True,True,512,512,512,0.0025,gicp converged
1595011713,350625000,norbit,gicp,True,True,512,512,512,0.0069,gicp converged
1595011743,388410000,norbit,gicp,True,True,512,512,512,0.0054,gicp converged
```

Columns: `sec,nsec,frame_id,backend,success,converged,input_points,preprocessed_points,aligned_points,fitness_score,status`.

GICP finds ~512 correspondences per fan and converges with fitness < 0.01 m² on
nearly every ping. The fan is very dense (Norbit iWBMSh outputs ~512 beams each
ping) and the surface vessel motion between pings is small, so correspondences
are easy to find — but the fan-to-fan geometry is degenerate along the heading,
which is why the recovered transform stays small relative to the 1.7 m/s vessel
motion (see _Measured numbers_ below).

## Record trajectory for offline comparison

```bash
# Estimate (sonar-only ICP/GICP).
ros2 run aqua_localization record_odometry.py \
  --topic /aqua_sonar_loc/odometry \
  --out /tmp/beach_pond_aqua_sonar_loc.tum \
  --format tum

# Reference (ROS 1 EKF baseline already in the bag).
ros2 run aqua_localization record_odometry.py \
  --topic /nav/processed/odometry \
  --out /tmp/beach_pond_baseline.tum \
  --format tum

# Quantitative comparison.
ros2 run aqua_localization compare_trajectories.py \
  /tmp/beach_pond_baseline.tum \
  /tmp/beach_pond_aqua_sonar_loc.tum
```

## Measured numbers (60 s slice, GICP backend)

A 60 s window starting at `--start-offset 60 --rate 1.0 --playback-duration 60`
captures 222 `/aqua_sonar_loc/odometry` samples (≈ 3.7 Hz, throttled by GICP runtime
on dense ~hundred-beam multibeam fans) and 2988 `/nav/processed/odometry` reference
samples. The vessel travels ≈ 125 m of ground track during the window.

| metric                                     | value (m) |
|--------------------------------------------|----------:|
| matched samples                            | 222 |
| matched duration                           | 59.56 s |
| APE mean — rigid SE(3) Umeyama             | 31.88 |
| APE median — rigid SE(3) Umeyama           | 31.26 |
| APE RMSE — rigid SE(3) Umeyama             | 37.28 |
| APE max — rigid SE(3) Umeyama              | 69.25 |
| APE mean — Sim(3) Umeyama (with scale)     | 9.70 |
| APE RMSE — Sim(3) Umeyama (with scale)     | 11.66 |
| Sim(3) alignment scale                     | 211.7× |

A Sim(3) scale of ~210× means the scan-matching estimate barely moves while the boat
covers ~125 m of ground track — the scan-to-scan registration converges into a tiny
near-static envelope around the origin. Plot in
`aqua_localization/docs/media/mbes_slam_demo_thumbnail.png`.

### Tried: submap-mode and submap-mode + motion prior

`aqua_sonar_loc` ships a submap front end (`scan_matching.submap_size > 1`) that
matches each new fan against the concatenation of the last K aligned fans, plus a
constant-velocity initial-guess option (`scan_matching.use_motion_prior`). On
`beach_pond` neither helps:

| variant                             | matched | APE mean rigid (m) | Sim(3) scale |
|-------------------------------------|--------:|-------------------:|-------------:|
| scan-to-scan (`submap_size=1`)      |     222 |              31.88 |       211.7× |
| submap_size=8, no prior             |     100 |              36.02 |       299.7× |
| submap_size=8, prior on             |      87 |              ~31.9 |   (collapsed)|

Why submap alone doesn't help: GICP converges to ~identity on a single multibeam
fan, so the submap rolls forward by ~0 m each step and stacks K copies of the same
sheet on top of itself — no along-track structure is gained. Adding a constant-velocity
motion prior cannot bootstrap from a pure sonar signal because the first registration
also returns ~identity, the prior captures ~identity, and the next registration starts
from ~identity again. **The geometric degeneracy is broken only by an external motion
prior** (IMU dead reckoning, DVL twist, or the bag's `/nav/processed/odometry`). The
submap infrastructure stays in the codebase for the IMU-prior wiring (next step) and
is intentionally left at `submap_size=1` in `mbes_slam.yaml` until then.

## Honest status

- **Scan-to-scan ICP/GICP between successive multibeam fans is geometrically marginal.**
  Each fan is essentially a cross-track line on the seafloor; consecutive pings overlap
  along the heading direction with very little along-track structure. Real bathymetric
  SLAM (the IJRR 2021 paper) accumulates submaps and matches them, not single fans.
  `aqua_sonar_loc` runs scan-to-scan only, so the 30+ m rigid APE on this dataset is
  expected. The demo demonstrates that the pipeline ingests real public multibeam data
  end-to-end, not that the pipeline matches the IJRR 2021 SLAM result.
- The platform is a surface vessel; depth state is essentially zero. The bag does not
  carry a `FluidPressure` topic, which is fine — `enable_imu_loc:=false` skips the UKF
  that would consume one.
- `compare_trajectories.py` runs Umeyama SE(3) alignment by default, so the rigid-aligned
  APE is a fair comparison with the ROS 1 EKF baseline. Sim(3) (`--scale`) tells you
  whether the *shape* of the estimate matches even when its scale is wrong — the 211×
  alignment scale here is the scan-to-scan saturation signature.

## Next steps

- Wire an external motion prior into `aqua_sonar_loc`. Either subscribe to
  `aqua_imu_loc/odometry` (IMU dead reckoning) or to a coarse Microstrain-IMU velocity
  channel, and use the inter-fan delta as the registration's initial guess. Once the
  prior is non-zero, re-enable `submap_size: 8` and `use_motion_prior: true` in
  `mbes_slam.yaml` — the submap infrastructure is already in place.
- Add a Microstrain-IMU-only profile to `aqua_imu_loc` (`/nav/sensors/microstrain/imu/raw`,
  no pressure, depth fixed at 0) so the dead-reckoning prior above can be produced from
  this same bag.
- Try `aqua_fusion` once IMU + sonar both have plausible odometry on this bag.

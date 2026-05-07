# Public Dataset Candidates

This page tracks candidate datasets for the first public `aqua_localization` demo video.

## Recommended First Demo

### NTNU Multi-Camera Underwater Visual-Inertial Dataset

- URL: https://huggingface.co/datasets/ntnu-arl/underwater-datasets
- License: BSD-3-Clause according to the dataset card.
- Platform: Ariel, a custom underwater robot based on BlueROV2 Heavy.
- Sensors relevant to this MVP:
  - IMU
  - barometer/pressure-derived depth
  - ROS bag files
  - reference trajectory files
- Why this is the first target:
  - public hosting is clear
  - download instructions are documented
  - ROS bag structure is documented
  - the platform is close to the BlueROV2 target class

Dataset card notes:

- `snapshot_download(repo_id="ntnu-arl/underwater-datasets", repo_type="dataset", ...)` can download the dataset.
- The dataset is large, so choose one short trajectory first.
- The card documents a barometer calibration:
  `depth = - (barometer_measurement - baro_pressure_offset_) / baro_pressure_scale`

First demo strategy:

1. Download one Marine Cybernetics Lab trajectory.
2. Run `inspect_bag_topics.py` on the bag directory.
3. If the bag exposes pressure/barometer as a scalar topic, use `scalar_to_pressure_node`
   with `scalar_to_pressure_ntnu.yaml` and fill the sequence calibration before recording.
4. Record IMU/depth dead reckoning first with sonar disabled.

Command shape:

```bash
ros2 run aqua_localization inspect_bag_topics.py /path/to/ntnu_bag
ros2 launch aqua_localization replay.launch.py \
  start_bag:=true \
  bag_path:=/path/to/ntnu_bag \
  enable_sonar_loc:=false \
  enable_fusion:=false \
  enable_rviz:=true
```

## Second Demo Candidate

### AQUALOC

- URL: http://www.lirmm.fr/aqualoc/
- Paper page: https://huggingface.co/papers/1910.14532
- Data format: ROS bags and raw data according to the paper page.
- Sensors relevant to this MVP:
  - low-cost IMU
  - pressure sensor
  - monocular camera
- Why this is useful:
  - directly aligned with visual-inertial-pressure underwater localization
  - good candidate for IMU + pressure dead reckoning demo

Risk:

- Availability and exact download layout should be checked immediately before recording.

## Sonar Track

### MBES-SLAM beach_pond (chosen first sonar demo)

- URL: <https://seaward.science/files/pos-datasets/bag/beach_pond.tar.gz>
- Paper: Hammond, Rowley, Roman, IJRR 2021. Bathymetric SLAM with Multibeam Echosounders Using Submap-Based Loop Closure.
- Platform: surface vessel with Norbit iWBMSh multibeam, Microstrain IMU, Nortek DVL, ublox GNSS.
- Bag size: 6.8 GB ROS 1 (≈ 158 min, ≈ 14.6 M messages).
- Sensors relevant to `aqua_localization`:
  - `/norbit/detections` `sensor_msgs/msg/PointCloud2` 13 Hz multibeam fan
  - `/nav/sensors/microstrain/imu/raw` `sensor_msgs/msg/Imu` 100 Hz
  - `/nav/processed/odometry` `nav_msgs/msg/Odometry` 50 Hz (ROS 1 EKF baseline)
- Why chosen:
  - first public dataset where the standard-typed point cloud is already in metric `xyz`
  - real multibeam, not a simulated FLS or a synthetic generator
  - the bag also includes a ground-truth EKF odometry track for offline APE comparison
- Limitation: scan-to-scan ICP/GICP between consecutive multibeam fans is geometrically
  marginal (the IJRR 2021 paper accumulates submaps and matches them). The demo shows
  end-to-end ingestion and convergence on real data, not parity with the paper's SLAM.

See [datasets/mbes_slam_demo.md](../datasets/mbes_slam_demo.md) for the conversion
workflow, replay command, and measured APE.

### OpenSonarDatasets (backup catalog)

- URL: https://github.com/remaro-network/OpenSonarDatasets
- Purpose: curated list of open-source sonar datasets.
- Use for: finding additional FLS/MBES sequences after MBES-SLAM beach_pond is exhausted.

Risk:

- Many sonar datasets are images, raw sonar, or task-specific annotations rather than metric `PointCloud2`.
- A conversion step may be needed before `aqua_sonar_loc` can run ICP/GICP.

## Future Candidate

### Tank Dataset

- Article: https://journals.sagepub.com/doi/full/10.1177/02783649251364904
- Data format described in article: ROS bags and raw data.
- Relevant topics listed in the article:
  - `/imu/data`: `sensor_msgs/Imu`
  - `/depth/data`: `nav_msgs/Odometry`
  - `/DVL/data`: DVL driver message
  - camera and ground-truth topics
- Why it matters:
  - has IMU/depth/DVL/GT and is valuable for later DVL and benchmark work.

Risk:

- Depth is not `FluidPressure`; it needs an adapter before direct MVP replay.

## Decision

NTNU `subset-fjord/fjord_1` is the first IMU + pressure demo and is fully wired through
[`datasets/ntnu_demo.md`](../datasets/ntnu_demo.md). MBES-SLAM `beach_pond` is the first
sonar demo and is fully wired through
[`datasets/mbes_slam_demo.md`](../datasets/mbes_slam_demo.md). AQUALOC is held as a
backup IMU + pressure target. The Tank Dataset is parked for the future DVL demo.

# Public Dataset Demo Plan

The next product milestone is a short public demo video that shows `aqua_localization` replaying open underwater data.

## Target Outcome

Record a 60-120 second video with:

- terminal launch command
- RViz view from `aqua_localization/rviz/demo.rviz`
- TF tree for `map -> odom -> base_link`
- odometry path from `/aqua_fusion/odometry` or `/aqua_imu_loc/odometry`
- estimator, scan matching, and fusion status topics
- sonar point cloud or filtered cloud when the dataset contains suitable sonar data

The README will embed the final video as:

```markdown
[![aqua_localization public dataset demo](docs/media/public_demo_thumbnail.png)](https://youtu.be/REPLACE_WITH_DEMO_VIDEO_ID)
```

## Candidate Public Data

The maintained shortlist is in [public_dataset_candidates.md](public_dataset_candidates.md).
Use NTNU first unless download size or topic format blocks replay; use AQUALOC as the IMU + pressure backup.

### Track A: IMU + Pressure First

Use this track to make the first reliable demo because it matches the current strongest MVP path.

Candidate datasets:

- AQUALOC: visual-inertial-pressure underwater localization data, available as ROS bags and raw data.
- NTNU underwater datasets: underwater robot data with IMU and pressure/barometer streams.

Expected demo:

- replay IMU and pressure
- run `aqua_imu_loc`
- optionally run `aqua_fusion` with sonar disabled
- show depth-constrained dead reckoning and estimator status

Example command shape:

```bash
ros2 run aqua_localization inspect_bag_topics.py /path/to/public_bag
ros2 launch aqua_localization replay.launch.py \
  start_bag:=true \
  bag_path:=/path/to/public_bag \
  enable_rviz:=true \
  enable_sonar_loc:=false \
  enable_fusion:=false
```

### Track B: Public Sonar (chosen: MBES-SLAM beach_pond)

The first sonar demo runs `aqua_sonar_loc` GICP against the MBES-SLAM `beach_pond`
public bathymetry bag (Norbit iWBMSh multibeam, real surface vessel survey).
Full workflow in [`datasets/mbes_slam_demo.md`](../datasets/mbes_slam_demo.md).

Candidate fallback sources if a different sonar sequence is needed later:

- additional MBES-SLAM bags (`pond`, `pond_run2`, etc. on the same seaward.science host)
- public sonar datasets listed in OpenSonarDatasets after conversion to `sensor_msgs/msg/PointCloud2`
- `uuv_simulator`/rexrov-style bags as a regression baseline

Expected demo (current):

- replay 60–120 s slice of `/norbit/detections` into `aqua_sonar_loc`
- GICP backend with `mbes_slam.yaml` quality gates
- record `/aqua_sonar_loc/odometry` and the bag's `/nav/processed/odometry` baseline
- compare offline with `compare_trajectories.py`

Example command shape:

```bash
ros2 launch aqua_localization replay.launch.py \
  start_bag:=false \
  use_sim_time:=true \
  enable_imu_loc:=false \
  enable_fusion:=false \
  sonar_params_file:=$(ros2 pkg prefix aqua_sonar_loc)/share/aqua_sonar_loc/config/mbes_slam.yaml \
  bag_sonar_points_topic:=/norbit/detections

ros2 bag play aqua_localization/datasets/public/mbes_slam/beach_pond_ros2 \
  --clock --start-offset 60 --playback-duration 60
```

## Demo Readiness Checklist

- Choose dataset and sequence.
- Confirm license allows public demo video.
- Run `inspect_bag_topics.py` and save the suggested command.
- Add any adapter command needed for depth-to-pressure or sonar point cloud conversion.
- Save final replay command in `datasets/README.md`.
- Create an RViz config for the demo.
- Record video.
- Add `docs/media/public_demo_thumbnail.png`.
- Replace the README YouTube placeholder URL.

## Honest Current Status

The codebase is now at MVP bringup level:

- all main nodes build and run
- all main nodes have runtime tests
- replay launch can start the stack and bag playback
- vehicle starter configs exist for BlueROV2 and `uuv_simulator`
- a bag inspection helper can suggest launch arguments

The project is not yet at validated localization-performance level on public data. The demo goal is to produce a reproducible bringup/localization pipeline first, then improve accuracy and dataset-specific tuning.

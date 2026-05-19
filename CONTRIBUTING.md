# Contributing

Thanks for helping improve `aqua_localization`. The most useful
contributions are reproducible dataset bring-up notes, focused bug fixes,
benchmark results, and small localization improvements with tests.

## Development Setup

Use a normal ROS 2 colcon workspace:

```bash
git clone https://github.com/rsasaki0109/aqua_localization.git
cd aqua_localization
rosdep install --from-paths . --ignore-src -r -y
cd ..
colcon build --symlink-install
source install/setup.bash
```

## Test Before Opening A Pull Request

```bash
colcon test --packages-select \
  aqua_imu_loc aqua_sonar_loc aqua_fusion aqua_pose_graph aqua_localization \
  --event-handlers console_direct+
```

For dataset-specific changes, include the exact bag, time window,
configuration file, and comparison command you used. If the change affects
accuracy, include before/after numbers from `compare_trajectories.py` or
the relevant benchmark script.

## Issue Quality

Good issue reports include:

- ROS 2 distribution and OS.
- Package or launch file involved.
- Exact command that failed.
- Relevant log output.
- Bag source, topic list, and time window if the issue is dataset-related.

For new public datasets, open a dataset request and include licensing,
download size, sensor topics, and the expected localization path
(IMU/pressure, DVL, sonar, visual, or acoustic).

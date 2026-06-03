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

## Useful Small Contributions

- Reproduce one documented dataset demo and report whether the commands still
  work on Humble or Jazzy.
- Add a public dataset request with license, sensor topics, download size, and
  the expected localization path.
- Submit a benchmark result with exact bag, topic remaps, parameters, metric,
  and before/after numbers.
- Improve a README or dataset note where a new user would otherwise need local
  knowledge to continue.

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

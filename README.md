# aqua_localization

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache--2.0-green.svg)](LICENSE)
[![Latest release](https://img.shields.io/github/v/release/rsasaki0109/aqua_localization)](https://github.com/rsasaki0109/aqua_localization/releases)

ROS 2 underwater localization for real AUV/ROV data: a 15-state additive
UKF for IMU + pressure + DVL + sonar updates, sonar point-cloud registration
with PCL ICP/GICP/NDT, and a g2o SE(3) pose graph backend.

![MBES-SLAM beach_pond multibeam sonar replay in rerun.io](docs/media/mbes_slam_beach_pond.gif)

The headline path is intentionally public-data first: four underwater
datasets, four [rerun.io](https://rerun.io) renderings, no synthetic bag
or simulator-only demo in the main story. Targets BlueROV2-class ROVs,
custom AUVs, and `uuv_simulator`. ROS 2 Humble and Jazzy are supported.

Latest release: **[v0.2](https://github.com/rsasaki0109/aqua_localization/releases/tag/v0.2)**.

## Why It Exists

Underwater localization usually needs more than a generic planar robot
stack: pressure/depth is a first-class measurement, DVL velocity arrives
in the vehicle frame, sonar geometry can be degenerate, and public
benchmarks often use ROS bags with dataset-specific topics. This repository
keeps those underwater-specific paths explicit and testable.

## Public-Data Results

Each row has a one-shot recorder and rerun export script. The screenshots
regenerate from the recorded demo bag without a manual RViz session.

| Dataset | What it shows | rerun screenshot |
|---------|---------------|------------------|
| **Tank Dataset `short_test`** | DVL fusion vs AprilTag GT, **0.43 m APE RMSE** on 15 s | [`tank_dataset_rerun.png`](docs/media/tank_dataset_rerun.png) |
| **MBES-SLAM `beach_pond`** | Multibeam fans accumulated into a depth-coloured bathymetric scan | [`mbes_slam_rerun.png`](docs/media/mbes_slam_rerun.png) |
| **NTNU `subset-fjord/fjord_1`** | Dataset SLAM baseline through a 7 m fjord dive | [`ntnu_fjord_1_rerun.png`](docs/media/ntnu_fjord_1_rerun.png) |
| **AQUALOC `harbor_07`** | LIRMM "Dumbo" ROV underwater camera + pressure depth track | [`aqualoc_harbor_07_rerun.png`](docs/media/aqualoc_harbor_07_rerun.png) |

<p>
  <img src="docs/media/tank_dataset_rerun.png" alt="aqua_localization on Tank Dataset short_test in rerun.io" width="49%">
  <img src="docs/media/mbes_slam_rerun.png" alt="aqua_localization on MBES-SLAM beach_pond in rerun.io" width="49%">
</p>

## Packages

| Package | Role |
|---------|------|
| [`aqua_imu_loc`](aqua_imu_loc) | 15-state additive UKF (IMU + pressure + DVL + sonar position) |
| [`aqua_sonar_loc`](aqua_sonar_loc) | PointCloud2 preprocessing + PCL ICP / GICP / NDT registration with submap front end |
| [`aqua_fusion`](aqua_fusion) | Loose-coupling fusion of IMU/depth and sonar odometry |
| [`aqua_pose_graph`](aqua_pose_graph) | g2o SE(3) keyframe graph (loop closure detection is the v0.3 milestone) |
| [`aqua_msgs`](aqua_msgs) | Diagnostic and fusion-input message types |
| [`aqua_localization`](aqua_localization) | Metapackage + top-level launches |

Detailed architecture per package: [`docs/architecture.md`](docs/architecture.md).

## Quick Start

```bash
# Clone into a colcon workspace, install dependencies, build, and source.
git clone https://github.com/rsasaki0109/aqua_localization.git
cd aqua_localization && rosdep install --from-paths . --ignore-src -r -y
cd ..
colcon build --symlink-install
source install/setup.bash

# Launch the full stack with default parameters.
ros2 launch aqua_localization aqua_localization.launch.py
```

## Try a Public Demo

The smallest validated path is the Tank Dataset `short_test` sequence:
about 15 seconds of real underwater motion with IMU, pressure-derived
depth, DVL, and AprilTag ground truth.

```bash
# After following datasets/tank_dataset_demo.md to download and convert
# short_test.bag, record the estimator outputs into a self-contained demo bag.
ros2 run aqua_localization record_tank_demo.sh

# Export to rerun.io for a browser-friendly 3D + plots replay.
ros2 run aqua_localization rerun_export.py \
  --bag aqua_localization/datasets/public/tank_dataset/demo_with_estimate \
  --out /tmp/tank.rrd
rerun /tmp/tank.rrd
```

Per-dataset bring-up notes include download size, conversion steps,
dataset-specific calibration, and replay commands:

- [`datasets/tank_dataset_demo.md`](datasets/tank_dataset_demo.md)
- [`datasets/mbes_slam_demo.md`](datasets/mbes_slam_demo.md)
- [`datasets/ntnu_demo.md`](datasets/ntnu_demo.md)
- [`datasets/aqualoc_demo.md`](datasets/aqualoc_demo.md)

## Web Replay

Two browser-friendly paths run on the same self-contained demo bag:

- **rerun.io** — the recommended default. Use
  [`rerun_export*.py`](aqua_localization/scripts) to write a `.rrd` with
  a curated 3D + plots blueprint, then open it locally with
  `rerun some.rrd`. Headless `--screenshot-to` produces the README
  thumbnails.
- **Lichtblick** (Apache-2.0 fork of Foxglove Studio) — drag the
  `.mcap` onto <https://lichtblick-suite.github.io/lichtblick/> and
  import [`docs/foxglove/aqua_tank_demo.json`](docs/foxglove/aqua_tank_demo.json).
  The accompanying [`lichtblick_screenshot.py`](aqua_localization/scripts/lichtblick_screenshot.py)
  drives the same flow headlessly via Playwright.

Bag-recording recipe: [`docs/foxglove/README.md`](docs/foxglove/README.md).

## Roadmap

The headline v0.3 milestone is **loop closure detection** on top of
`aqua_pose_graph`. Bathymetric submap-vs-submap matching for the MBES
path and visual loop closure for AQUALOC are the two natural front
ends. ESKF backend, magnetometer fusion, and acoustic positioning are
also on the list.

Plan and state of the stack: [`PLAN.md`](PLAN.md).
Verified-feature checklist: [`docs/mvp_checklist.md`](docs/mvp_checklist.md).
Per-platform benchmarks: [`docs/benchmarks/`](docs/benchmarks).
Public launch checklist: [`docs/public_launch_checklist.md`](docs/public_launch_checklist.md).
MBES loop closure plan: [`docs/mbes_loop_closure.md`](docs/mbes_loop_closure.md).

## Honest Limitations

- IMU-only dead reckoning drifts roughly an order of magnitude on bags
  without DVL/visual aiding (NTNU `fjord_1`, AQUALOC `harbor_07` show
  hundreds of meters of XY drift; depth `z(t)` tracks well via the
  pressure update).
- Single-fan multibeam registration is geometrically degenerate.
  Tightly-coupled sonar feedback narrows MBES-SLAM `beach_pond` fusion
  drift from ±40 m to ~17 m, but per-fan residuals are still ~10 m
  magnitude. The pose graph backend ships, but loop closure detection
  (the front end that *generates* loop constraints) is the v0.3 work.
- `aqua_fusion` has unit + runtime tests but no per-platform benchmark
  history yet.

## Testing

```bash
colcon test --packages-select \
  aqua_imu_loc aqua_sonar_loc aqua_fusion aqua_pose_graph aqua_localization \
  --event-handlers console_direct+
```

134 + 5 unit / runtime tests on v0.2 (zero failures).

## Contributing

Bug reports, dataset bring-up notes, benchmark results, and focused pull
requests are welcome. Start with [`CONTRIBUTING.md`](CONTRIBUTING.md)
for the expected build/test commands and issue templates.

## License

Apache-2.0. See [`LICENSE`](LICENSE) and individual `package.xml` files
for per-package maintainer info.

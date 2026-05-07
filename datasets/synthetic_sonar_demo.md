# Synthetic sonar bag — `aqua_sonar_loc` end-to-end demo

This note exercises the `aqua_sonar_loc` package without a real sonar dataset. It uses
`scripts/make_synthetic_sonar_bag.py` to generate a deterministic `sensor_msgs/PointCloud2`
rosbag2 of a robot moving at constant speed along world +x through a fixed point field, then
plays the bag through `replay.launch.py` with the IMU and fusion nodes disabled.

## Generate the bag

```bash
ros2 run aqua_localization make_synthetic_sonar_bag.py \
  --dst /tmp/synthetic_sonar_bag \
  --topic /sonar/points \
  --num-points 400 \
  --num-steps 120 \
  --dt 0.1 \
  --speed-m-s 0.5 \
  --max-range-m 40.0 \
  --overwrite
```

Each step writes one `sensor_msgs/PointCloud2` with `header.frame_id = sonar_link`. The world
points are sampled once with a fixed RNG seed (`--seed 7`) so the bag is reproducible.

## Replay

```bash
ros2 launch aqua_localization replay.launch.py \
  start_bag:=true \
  bag_path:=/tmp/synthetic_sonar_bag \
  bag_sonar_points_topic:=/sonar/points \
  use_sim_time:=true \
  enable_imu_loc:=false \
  enable_fusion:=false
```

Watch the topics:

```bash
ros2 topic list
ros2 topic echo /aqua_sonar_loc/status --once
ros2 topic echo /aqua_sonar_loc/odometry --once
```

## Expected output

`aqua_sonar_loc` starts the ICP backend (`backend=icp`) and reports each scan as
`accepted` (the synthesizer drops scans that fall below `--min-points`).

Recording `/aqua_sonar_loc/odometry` to a TUM file with `record_odometry.py` shows the
scan-to-scan ICP composing into a clean linear track. A short example over 1.5 s:

| t [s]  | x [m]  | y [m]  | z [m]  |
|-------:|-------:|-------:|-------:|
| 10.40  | -5.194 | -0.005 | -0.007 |
| 11.90  | -5.944 | -0.006 | -0.008 |

`Δx = -0.75 m` over 1.5 s gives **-0.5 m/s in body frame**, which matches the synthesized
**+0.5 m/s world frame** speed (ICP reports the inverse: scene appears to move in -x as the
robot moves in +x). y and z stay near zero, as expected for a planar straight-line trajectory.

## Quality gating (`scan_matching.max_*`)

`aqua_sonar_loc` exposes three post-ICP gates:

- `max_fitness_score` (PCL ICP fitness, mean squared correspondence distance)
- `max_translation_step_m` (per-step translation magnitude)
- `max_rotation_step_rad` (per-step rotation magnitude)

A non-positive value disables the gate (default). When a gate fires, the scan is rejected:
`/aqua_sonar_loc/status.success = false`, `status.status` is one of:

```text
icp rejected: fitness_score above max_fitness_score
icp rejected: translation step above max_translation_step_m
icp rejected: rotation step above max_rotation_step_rad
```

`/aqua_sonar_loc/odometry` is **not** published on rejection and the accumulated
transform is not advanced — this prevents bad ICP solutions from poisoning downstream
fusion or trajectory dead reckoning.

To exercise the gates without writing a custom config, generate a noisy variant and
override params at runtime:

```bash
ros2 run aqua_localization make_synthetic_sonar_bag.py \
  --dst /tmp/synthetic_sonar_bag_noisy --topic /sonar/points \
  --xy-noise-stddev 0.3 --overwrite

# In one terminal, play the noisy bag:
ros2 bag play /tmp/synthetic_sonar_bag_noisy --clock

# In another, run the sonar node with a tight fitness gate:
ros2 run aqua_sonar_loc sonar_loc_node --ros-args \
  -p use_sim_time:=true \
  -p scan_matching.max_fitness_score:=0.05 \
  -p scan_matching.max_translation_step_m:=0.2
```

`/aqua_sonar_loc/status` will show some scans flipping to `success: false` once the
correspondences become noisy enough to push the fitness score above the gate.

## Switching the scan-matching backend (icp / gicp / noop)

`aqua_sonar_loc` ships three backends and a factory selected via
`scan_matching.backend`. To run the synthetic bag through GICP instead of ICP:

```bash
ros2 run aqua_sonar_loc sonar_loc_node --ros-args \
  -p use_sim_time:=true \
  -p scan_matching.backend:=gicp
```

On the clean synthetic bag both ICP and GICP recover the same -0.5 m/s in body x with
residuals at the 1e-7 m level. GICP needs at least ~20 points per scan to compute
local point covariances, so generate dense bags (default `--num-points 400` is fine)
when comparing the two backends.

## Why this is useful

- Validates the full pipeline: PointCloud2 ingestion → preprocessor → PCL ICP → odometry/status.
- Provides a known-answer regression input that does not require Gazebo/`uuv_simulator` or any
  large public sonar dataset.
- Good base for future variants: add Gaussian noise (`--xy-noise-stddev`), change the trajectory,
  or switch the ICP backend to test sonar-side improvements.

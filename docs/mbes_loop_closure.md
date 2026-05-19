# MBES Loop Closure Front-End Plan

The pose graph backend can now accept external loop closures on
`/aqua_pose_graph/loop_constraint`. This document pins the intended MBES
front end so the next implementation can stay small and measurable.

## Current Building Blocks

- `aqua_pose_graph` builds a g2o SE(3) keyframe chain from upstream odometry.
- `aqua_msgs/PoseGraphKeyframe` exposes the keyframe ID and pose assigned by
  the graph backend.
- `aqua_msgs/PoseGraphLoopConstraint` carries a relative SE(3) constraint
  between two existing keyframes.
- `aqua_sonar_loc` already has PCL ICP/GICP/NDT backends and MBES-specific
  quality gates.
- `aqua_sonar_loc/mbes_loop_closure_node` is the first experimental MBES
  front end. It accumulates short submaps between pose-graph keyframes,
  searches older odometry-near submaps, registers them with ICP/GICP/NDT, and
  publishes accepted constraints to `/aqua_pose_graph/loop_constraint`.
- The same node publishes `aqua_msgs/LoopClosureStatus` on
  `/mbes_loop_closure/status` for each tested candidate, including rejection
  reason, convergence, fitness score, and correction magnitude.
- `aqua_localization/scripts/pose_graph_loop_demo.py` publishes a synthetic
  odometry chain plus one loop constraint for smoke-testing the graph input.

## Target MBES Pipeline

1. Subscribe to accepted MBES fans and pose-graph keyframes.
2. Accumulate a local bathymetric submap for each keyframe interval.
3. Keep a searchable index of older submaps.
4. Reject candidates inside a temporal/keyframe exclusion window.
5. Run submap-vs-submap GICP, ICP, or NDT with the odometry relative
   transform as the initial guess.
6. Gate on convergence, fitness, correction magnitude, and transform
   consistency.
7. Publish `aqua_msgs/PoseGraphLoopConstraint` with a conservative
   information matrix.
8. Publish `aqua_msgs/LoopClosureStatus` so tuning can distinguish "no
   candidates" from rejected or accepted registration results.
9. Re-export the rerun.io demo with before/after pose-graph paths and loop
   edges.

## Smoke Demo

Terminal A:

```bash
ros2 launch aqua_pose_graph pose_graph.launch.py
```

Terminal B:

```bash
ros2 run aqua_localization pose_graph_loop_demo.py
```

Expected behavior:

- `/aqua_pose_graph/keyframe_count` reaches 5,
- `/aqua_pose_graph/loop_constraint_count` reaches 1,
- `/aqua_pose_graph/path` updates after the loop constraint is inserted and
  optimized.

## First Real-Data Target

Use MBES-SLAM `beach_pond` because the repository already has:

- dataset conversion notes,
- GICP config,
- rerun export,
- committed screenshots and GIFs.

The first real-data milestone does not need to find every loop closure. A
single accepted submap-vs-submap loop that visibly changes the optimized
path is enough for the next README-worthy demo.

Launch shape:

```bash
ros2 launch aqua_localization replay.launch.py \
  start_bag:=true \
  bag_path:=aqua_localization/datasets/public/mbes_slam/beach_pond_ros2 \
  bag_sonar_points_topic:=/norbit/detections \
  use_sim_time:=true \
  enable_imu_loc:=true \
  enable_sonar_loc:=true \
  enable_pose_graph:=true \
  enable_mbes_loop_closure:=true \
  imu_params_file:=$(ros2 pkg prefix aqua_imu_loc)/share/aqua_imu_loc/config/mbes_slam.yaml \
  sonar_params_file:=$(ros2 pkg prefix aqua_sonar_loc)/share/aqua_sonar_loc/config/mbes_slam.yaml \
  mbes_loop_closure_params_file:=$(ros2 pkg prefix aqua_sonar_loc)/share/aqua_sonar_loc/config/mbes_loop_closure.yaml
```

Tune in this order:

1. `submaps.voxel_leaf_m` and `submaps.min_points` until submaps are dense
   enough but not too slow.
2. `candidates.max_distance_m` and `candidates.min_keyframe_separation` until
   plausible revisits are tested.
3. `gates.max_fitness_score`, `gates.max_correction_translation_m`, and
   `gates.max_correction_rotation_rad` until false positives are rejected.
4. `loop.translation_sigma_m` and `loop.rotation_sigma_rad` after comparing
   optimized path changes against the MBES-SLAM reference odometry.

Useful live checks while tuning:

```bash
ros2 topic echo /mbes_loop_closure/status
ros2 topic echo /aqua_pose_graph/loop_constraint_count
```

`LoopClosureStatus.candidate_id` is `UINT32_MAX` when a keyframe has no
eligible historical submap. Rejections report the specific gate that failed,
so overly strict candidate, fitness, or correction thresholds are visible
without reading debug logs.

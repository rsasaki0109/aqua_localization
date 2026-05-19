# MBES Loop Closure Front-End Plan

The pose graph backend can now accept external loop closures on
`/aqua_pose_graph/loop_constraint`. This document pins the intended MBES
front end so the next implementation can stay small and measurable.

## Current Building Blocks

- `aqua_pose_graph` builds a g2o SE(3) keyframe chain from upstream odometry.
- `aqua_msgs/PoseGraphLoopConstraint` carries a relative SE(3) constraint
  between two existing keyframes.
- `aqua_sonar_loc` already has PCL ICP/GICP/NDT backends and MBES-specific
  quality gates.
- `aqua_localization/scripts/pose_graph_loop_demo.py` publishes a synthetic
  odometry chain plus one loop constraint for smoke-testing the graph input.

## Target MBES Pipeline

1. Subscribe to accepted MBES fans and pose-graph keyframes.
2. Accumulate a local bathymetric submap around each keyframe.
3. Keep a searchable index of older submaps.
4. Reject candidates inside a temporal/keyframe exclusion window.
5. Run submap-vs-submap GICP or NDT with the odometry relative transform as
   the initial guess.
6. Gate on convergence, fitness, inlier count, relative step size, and
   transform consistency.
7. Publish `aqua_msgs/PoseGraphLoopConstraint` with a conservative
   information matrix.
8. Re-export the rerun.io demo with before/after pose-graph paths and loop
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

# Changelog

## Unreleased

- Added `aqua_graph`, a GTSAM fixed-lag factor-graph backend fusing IMU
  preintegration, body-frame DVL velocity, pressure depth, and stereo
  reprojection factors with event-anchored states. Tank `short_test`:
  0.0092 m APE RMSE on AQUA-SLAM's 11.65 s window (AQUA-SLAM 0.0194 m);
  single public sequence, held-out validation pending.
- Added `aqua_replay`: deterministic offline replay (bit-identical repeat
  runs, verified on real beach_pond MBES data), multi-bag input, an offline
  stereo feature frontend, and `run_tank_benchmark.py`.
- Fixed the Tank IMU-DVL extrinsic (identity -> official calibration
  rotation); UKF Tank row 0.429 m -> 0.145 m.
- Registered all stack nodes as rclcpp components; added CI on Humble and
  Jazzy.
- Added a sensor-outage robustness sweep (`run_tank_robustness.py`,
  `offline_replay --drop-topic`, feature-track `--drop`): on Tank
  `short_test` aqua_graph stays at 0.0096 m through a 10 s DVL outage (UKF
  7.4 m) and degrades to 0.022 m through a 10 s camera outage.
- Track B dead reckoning on the full MBES-SLAM `beach_pond` survey (16.7 km,
  no GNSS position): aqua_graph with IMU + DVL + GNSS compass reaches 2.84 m
  APE RMSE vs 3.60 m for classical DVL dead reckoning
  (`docs/benchmarks/beach_pond_dead_reckoning.md`). New in aqua_graph: a
  heading factor, `TwistWithCovarianceStamped` DVL input, depth hold for
  surface vessels, `dvl.time_offset_s`, zero-bias init for moving starts,
  and smoother re-anchoring instead of crashing on singular / inconsistent
  fixed-lag updates (`GraphEstimatorStatus.smoother_resets`).
- Fixed MBES loop-closure submaps: `mbes_loop_closure` stacked raw
  sensor-frame pings (single across-track lines) into a submap; with
  `topics.odometry` each ping is now placed at its own odometry pose, and
  `sensor.max_across_track_ratio` drops refraction-limited outer beams.
  Earlier `mbes_beach_pond_loop_audit*` pages predate this fix. On
  beach_pond, bathymetric loops are not identifiable (flat basin at the
  crossings); see `docs/benchmarks/beach_pond_mbes_loop_feasibility.md`.

- Reduced pose-graph runtime work by skipping automatic g2o optimization on
  odometry-only chains until a loop constraint exists, with an opt-in parameter
  for the older chain-only auto-optimize behavior.
- Added pose-graph optimization diagnostics for optimize count and latest
  active chi^2 so real-bag tuning can confirm when g2o actually runs.
- Threaded those pose-graph optimization diagnostics into the MBES loop-status
  summary exporter and optional benchmark-row columns.
- Added `mbes_loop_benchmark_row.py --summary` so benchmark rows can pull the
  latest optimization diagnostics from the exported markdown summary.
- Added DVL-prior quality gates to the Tank held-out validation path so
  AQUA-SLAM comparison candidates can require coverage and confidence evidence,
  not only lower RMSE.
- Forwarded those DVL-prior quality gates through the Medium held-out verifier
  and AQUA-SLAM claim-readiness report.
- Regenerated the AQUA-SLAM claim-readiness page with the current Medium
  blockers and visible DVL-prior gate thresholds.
- Added optional Huber/DCS robust kernels on pose-graph loop constraints to
  reduce bad-loop influence during MBES loop-closure tuning.
- Added an optional accepted-loop consistency guard for MBES loop closures so
  new candidates can be rejected when their correction disagrees with trusted
  prior loop corrections.
- Rebuilt the GitHub Pages landing page around the full public-data demo wall,
  MBES hero replay, 3DGS viewer entry point, and GitHub star CTA.
- Recorded a readiness-only 3DGS training smoke report for the published Tank
  sample pack.
- Planned: 3DGS training smoke test from the published Tank sample pack.
- Planned: MBES loop-closure threshold sweep and status summary on a real bag.
- Planned: AQUA-SLAM comparison run with reproducible tables.

## v0.5 - 2026-05-22

This release makes the underwater 3DGS sample track easier to see, share, and
inspect from GitHub Pages. The downloadable sample pack remains attached to
v0.3.

- Simplified the GitHub Pages landing page around the underwater 3DGS sample.
- Reworked the sample pack inspector into a frame + 3D camera-path viewer.
- Added Play, Pause, and Reset controls for the 20-frame Tank sample.
- Refreshed the README/GitHub Pages screenshot to show the interactive viewer.
- Added Open Graph and Twitter card metadata for the 3DGS Pages entry points.
- Updated README entry points to highlight the playable sample viewer.
- Added a 3DGS training readiness checker for nerfstudio-style sample packs.
- Added the v0.3 sample pack readiness result to the static inspector.
- Added a public 3DGS training smoke-test guide.
- Added a 3DGS training smoke-test report template.

## v0.4 - 2026-05-21

This is a presentation and infrastructure release for the underwater 3DGS
sample pack track. The downloadable sample pack remains attached to v0.3.

- Added a static 3DGS sample pack inspector on GitHub Pages.
- Added a contact sheet generated from the 20 Tank Dataset sample frames.
- Added a compact viewer JSON with frame poses, intrinsics, topics, and counts.
- Added a README screenshot linking to the inspector.
- Updated the GitHub Pages workflow actions to Node 24-compatible major
  versions.

## v0.3 - 2026-05-21

- Published the first underwater 3DGS input sample pack artifact:
  `tank_short_test_3dgs_pack_20frames.zip`.
- Added manual camera intrinsics support for Tank camera conversions without
  `sensor_msgs/msg/CameraInfo`.
- Added direct README and GitHub Pages download links for the sample pack.

## v0.2 - 2026-05-09

- Added the pose-graph backend and sonar covariance calibration release
  snapshot.

## v0.1 - 2026-05-08

- Published the initial MVP snapshot.

# Changelog

## Unreleased

- Added a 3DGS training readiness checker for nerfstudio-style sample packs.
- Planned: 3DGS training smoke test from the published Tank sample pack.
- Planned: MBES loop-closure threshold sweep and status summary on a real bag.
- Planned: AQUA-SLAM comparison run with reproducible tables.

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

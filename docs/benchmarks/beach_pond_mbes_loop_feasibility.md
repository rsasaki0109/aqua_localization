# MBES Loop Closure on `beach_pond`: Feasibility Findings (2026-10-11)

Goal: correct the no-compass dead-reckoning drift of
[`beach_pond_dead_reckoning.md`](beach_pond_dead_reckoning.md) (hundreds of
metres over 2.4 h) with bathymetric submap loop closures. Status: **no
correct loop constraint is obtainable on this seafloor; the plumbing is
in place for a dataset with more relief.** Nothing here is a benchmark
claim.

## What changed

- **Motion-compensated submaps (correctness fix).** `mbes_loop_closure`
  used to build a submap by stacking the raw sensor-frame pings between two
  keyframes. A Norbit ping is a single across-track line (512 beams, x = 0),
  so those submaps were overlapping profiles at one spot, not a seafloor
  patch, and any loop "accepted" on them registered meaningless geometry.
  With `topics.odometry` set, every ping is now placed at its own odometry
  pose (`PoseInterpolator`, slerp) in the anchor keyframe's body frame via
  `sensor.translation_m` / `sensor.rotation_rpy_rad`. Empty `topics.odometry`
  keeps the old behaviour. Earlier loop-audit pages
  (`mbes_beach_pond_loop_audit*.md`) were produced on stacked-ping submaps
  and should be re-run before they are trusted.
- Scenario `aqua_replay/scenarios/beach_pond_mbes_slam.yaml`: aqua_graph
  (no compass) -> aqua_pose_graph (20 m keyframes) -> mbes_loop_closure
  (Norbit pings + aqua_graph odometry), 30 min replays in ~2 min.

## Findings (first 30 min, no compass)

1. **Candidates are real revisits, but the guess is far off.** All 33
   tested candidate pairs are true revisits (reference anchor distance
   1-43 m). The dead-reckoned initial guess is off by 12-39 m and a
   constant ~15.6 deg in yaw, which is the gyro-bias heading drift
   (~2.3e-4 rad/s) accumulated before the first revisit (~20 min). Local
   GICP (3 m correspondence radius) cannot converge from there; every pair
   failed the fitness gate.
2. **Outer beams are refraction-limited.** Gridded at 1 m from the
   reference INS poses, overlapping passes disagree by 0.17-1.6 m at the
   true pose, while each pass is internally smooth (points within 0.02 m
   of their cell median, p99 0.19 m). The disagreement depends on beam
   angle: median |dz| between passes is ~0.45 m with all beams, 0.06-0.13 m
   with beams within 45 deg (|y| <= depth), 0.03-0.06 m within ~31 deg.
   Ruled out on the way: sonar mount roll/pitch (+-1.5 deg sweep, < 5 %
   change), a port/starboard mirror, and sonar-vs-nav clock offsets
   (including the 18 s GPS-UTC leap). New
   `sensor.max_across_track_ratio` (1.0 in the scenario) applies the 45 deg
   limit.
3. **Crossings are not identifiable either.** Restricting loops to the 12
   near-perpendicular crossings (45-90 deg) on the reference track, with
   reference poses, inner beams and median gridding, a yaw (+-30 deg) x
   translation (+-30 m) search never selects the true pose: shifted or
   rotated placements match more cells within 0.15 m (up to ~90 % of the
   overlap) than the truth does (~60 %). The crossings lie in the ~21 m
   deep central basin, which is flat to ~0.1 m over tens of metres; the
   sloped banks, where registration could work, are only surveyed by
   parallel concentric passes.
## Next options

- Evaluate MBES loop closure on a dataset with real relief (the plumbing:
  motion-compensated submaps, beam-angle filter, pose-graph scenario, is
  ready). On beach_pond the honest result is that bathymetric loop closure
  cannot correct the no-compass drift.
- With the compass (the realistic configuration for this surface vessel)
  dead reckoning is already 2.84 m RMSE, so there is little left for loops
  to correct here.

# MBES Loop Closure on `beach_pond`: Feasibility Findings (2026-10-11)

Goal: correct the no-compass dead-reckoning drift of
[`beach_pond_dead_reckoning.md`](beach_pond_dead_reckoning.md) (hundreds of
metres over 2.4 h) with bathymetric submap loop closures. Status: **no
loop accepted yet; the blocker is registration observability on this
seafloor, not the plumbing.** Nothing here is a benchmark claim.

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
2. **Submaps disagree even at the true pose.** Gridded at 1 m and placed
   with the reference INS poses, overlapping submaps differ by 0.17-1.16 m
   RMS (mean offsets up to 0.7 m). A sonar mount roll/pitch sweep
   (+-1.5 deg) changes this by < 5 %, so it is not a simple mount bias;
   outer-beam refraction and steep-bank gridding are the likely sources.
   With aqua_graph attitude instead of the INS the disagreement grows a
   little (graph-vs-INS tilt: 0.43 deg mean, 0.5 deg std about the swath
   roll axis).
3. **The seafloor is too smooth to resolve that.** Median slope is 4 %
   (p90 15 %), so a 0.5 m vertical inconsistency equals ~12 m of
   horizontal ambiguity. A coarse yaw x translation search over DEMs never
   recovered the true pose: plain NCC is flat (0.998 everywhere, a tilted
   plane matches its shifted copies), high-pass NCC peaks are not unique,
   and absolute-height RMS (with a 50 % overlap requirement) lands
   6-35 m from the truth.

## Next options

- Larger, multi-pass submaps (100 m+ of track) so curvature, not slope,
  dominates; or restrict loops to crossing transits where the two swaths
  are near-perpendicular (along-slope ambiguity of one pass is constrained
  by the other).
- Use the compass case instead (the realistic configuration for this
  surface vessel): DR is already 2.84 m RMSE there, so loops would need
  sub-metre registration, which finding 2 rules out on this data.
- Evaluate on a dataset with more relief before investing further here.

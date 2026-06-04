# aqua_localization Plan and State of the Stack

This is the canonical "where we are, where we are going" document for
`aqua_localization`. Keep it practical: it should tell a returning contributor
what is already solid, what is still experimental, and what the next useful
engineering move is.

Current date: 2026-06-05.
Latest release: [v0.5](https://github.com/rsasaki0109/aqua_localization/releases/tag/v0.5).
Current public goal: make the project useful enough to earn 10 GitHub stars
through reproducible public-data demos, honest limits, and small contribution
paths.

## Project Goal

`aqua_localization` is a ROS 2 localization stack for underwater robots. The
core idea is that underwater localization is not just "mobile robot odometry
under water": GNSS disappears, pressure/depth is a primary measurement, DVL is
body-frame velocity, and sonar observations can be sparse, degenerate, delayed,
or hard to associate.

The stack is built around real public data first:

- Tank Dataset `short_test` for DVL + pressure + AprilTag ground truth.
- MBES-SLAM `beach_pond` for multibeam bathymetry and MBES loop-closure work.
- NTNU `subset-fjord/fjord_1` for underwater inertial/depth replay.
- AQUALOC `harbor_07` for underwater camera + pressure data.

The project identity remains:

- not a thin wrapper around `robot_localization`;
- a self-implemented 15-state additive UKF in `aqua_imu_loc`;
- pressure/depth updates are first-class;
- DVL and sonar feedback are first-class, not afterthoughts;
- PCL-based sonar registration is part of the localization path;
- g2o pose graph loop closure is present but still experimental on real MBES;
- public demos and benchmark artifacts must be reproducible from commands.

ROS 2 Humble and Jazzy are the supported targets. Humble is currently the main
local replay/benchmark environment for MBES determinism work.

## Current Snapshot

The repository is no longer at "MVP only" stage. It now has:

- a public README and GitHub Pages landing page;
- a 10-star launch checklist and repository topics/description aligned with
  the public positioning;
- public-data demo artifacts for Tank, MBES-SLAM, NTNU, and AQUALOC;
- a published v0.3 Tank Dataset 3DGS input pack and interactive viewer;
- releases through v0.5;
- g2o pose graph backend;
- experimental MBES loop-closure front end;
- status exports, audit plots, geometry review tables, and benchmark rows;
- SOTA gap analysis and OSS comparison docs.

The strongest claimable result is still the Tank Dataset localization row:
DVL + pressure replay on `short_test` with 0.43 m APE RMSE, plus an
experimental same-sequence visual-aided row around 0.37 m. The MBES path is a
working research track, not a SOTA claim.

## Release History

### v0.1 - MVP snapshot

- ROS 2 ament packages: `aqua_msgs`, `aqua_imu_loc`, `aqua_sonar_loc`,
  `aqua_fusion`, and `aqua_localization`.
- 15-state additive UKF: position, velocity, RPY, accel bias, gyro bias.
- Pressure/depth measurement update.
- DVL body-frame velocity update.
- Optional AHRS yaw / gyro-bias hooks.
- Static-bias initializer with motion detection.
- IMU mount rotation and surface-vessel pseudo-depth hooks.
- PCL ICP/GICP/NDT sonar registration path.
- Basic runtime tests and public-data bring-up scripts.

### v0.2 - Pose graph backend and sonar covariance calibration

- New `aqua_pose_graph` package with g2o SE(3) keyframe graph.
- External loop constraints through `/aqua_pose_graph/loop_constraint`.
- Pose graph path publication through `/aqua_pose_graph/path`.
- Sonar covariance calibration script for MBES residuals.
- Top-level launch wiring for pose graph mode.

### v0.3 - Public 3DGS input artifact

- Tank Dataset 20-frame 3DGS-style sample pack.
- `transforms.json`, matched frames, and pose metadata.
- Interactive GitHub Pages viewer for inspection before download.
- Training-readiness checker and documentation.

### v0.4 - Public comparison and benchmark documentation

- Stronger benchmark docs under `docs/benchmarks`.
- AQUA-SLAM comparison anchor and SOTA gap analysis.
- Public-data result tables organized for README and Pages.
- More explicit "no superiority claim yet" language.

### v0.5 - Public launch and MBES determinism push

- Public landing page and OGP metadata refreshed for the 10-star push.
- MBES loop-status messages carry endpoint keyframe stamps.
- Benchmark recorder readiness is strict by default.
- Recorder cleanup is hardened on failure.
- MBES profile resolution now uses the sourced install prefix.
- MBES replay sensor queues are kept shallow by default after a deep-queue
  probe caused kilometer-scale RMSE/backlog.
- MBES submaps are now assembled by message timestamp rather than callback
  arrival order.
- `IMU_SONAR_ODOMETRY_TOPIC` is exposed as a diagnostic override so replay
  runs can disable sonar feedback into the IMU UKF without editing YAML.

## Architecture State

```
raw IMU / AHRS / pressure / DVL / sonar
                  |
                  v
           aqua_imu_loc
       additive UKF + depth/DVL
       + optional sonar feedback
                  |
                  v
        /aqua_imu_loc/odometry
                  |
      +-----------+------------+
      |                        |
      v                        v
aqua_sonar_loc           aqua_pose_graph
PCL ICP/GICP/NDT         g2o SE(3) keyframes
submaps + covariance     external loop constraints
      |                        ^
      v                        |
/aqua_sonar_loc/odometry       |
      |                        |
      +---- optional feedback --+
```

Important ownership rules:

- `aqua_imu_loc` owns dead reckoning and publishes `/aqua_imu_loc/odometry`.
- `aqua_sonar_loc` publishes sonar registration output and filtered point
  clouds.
- `aqua_pose_graph` consumes upstream odometry and loop constraints.
- `mbes_loop_closure_node` consumes pose-graph keyframes and MBES point clouds,
  proposes loop constraints, and publishes loop-status diagnostics.
- `aqua_fusion` still exists as loose-coupling support, but the SOTA-facing
  track is increasingly pose-graph and tightly-coupled feedback oriented.

## Verified Public Paths

### Tank Dataset

Status: strongest public localization result.

- Dataset: Tank Dataset `short_test`.
- Sensors: IMU, depth/pressure-derived z, DVL, AprilTag ground truth.
- Current headline: 0.43 m APE RMSE for DVL + pressure fusion.
- Visual-aided row exists but is not a broad claim because it is same-sequence
  and still needs a stronger held-out protocol.

Next work:

- Convert the result into a stricter benchmark table with exact source bag,
  config, commit, alignment statement, and failure gates.
- Compare fairly with AQUA-SLAM under sensor-equivalent conditions.
- Add held-out Tank windows, not only `short_test`.

### MBES-SLAM beach_pond

Status: active research track, not claimable SOTA.

What works:

- MBES fans replay into `aqua_sonar_loc`.
- Filtered point clouds and sonar odometry are recorded.
- MBES submap loop closure runs and publishes status rows.
- Accepted/rejected/no-candidate rows can be exported to CSV and markdown.
- Audit plots and geometry reports exist.
- Endpoint keyframe timestamps are embedded in status messages.
- Strict recorder readiness prevents source-only false benchmark bags.
- Timestamp-buffered submaps reduce replay variance.

Most recent determinism results:

- Endpoint-stamp probe:
  - input RMSE spread: 7.4646 m;
  - pose graph RMSE spread: 22.5625 m;
  - accepted-loop spread: 9.
- Timestamp-buffered submap probe:
  - input RMSE spread: 0.0230 m;
  - pose graph RMSE spread: 4.5837 m;
  - accepted-loop spread: 4.
- Open-loop sonar-feedback diagnostic:
  - accepted-loop count spread: 0;
  - but input RMSE degraded to about 1 km;
  - conclusion: sonar feedback is performance-critical and order-sensitive.

The important finding is not "MBES loop closure is solved." It is:

- message-stamp submap assembly helps;
- shallow queues are necessary;
- strict recorder readiness is necessary;
- sonar feedback must stay enabled for performance;
- the remaining blocker is deterministic ordering around sonar feedback,
  candidate evaluation, registration scheduling, and pose-graph keyframe
  identity.

### NTNU and AQUALOC

Status: useful public replay demos, not high-accuracy claims.

- Depth/pressure behavior is useful and inspectable.
- IMU-only XY drift remains large without DVL/visual/acoustic aiding.
- These tracks are good for robustness, adapters, and future visual loop
  closure, but they should not be presented as current accuracy wins.

### Underwater 3DGS

Status: published input artifact, not a trained reconstruction claim.

- v0.3 pack contains 20 Tank frames, matched poses, intrinsics, and
  nerfstudio-style metadata.
- GitHub Pages viewer is live.
- Training-readiness checker passes.

Next work:

- Run a small training smoke test with exact command, environment, and output.
- Report failure modes honestly if underwater imagery or sparse frame count
  blocks useful reconstruction.
- Keep it separate from localization SOTA claims unless it feeds back into
  visual odometry or mapping.

## Why This Is Not SOTA Yet

The gap is not "missing one magic feature." The current blockers are evidence
and determinism.

1. Repeatability is still not strong enough on MBES loop closure.

   The same source bag and settings can still produce different accepted loop
   endpoint pairs. Timestamp-buffered submaps reduced RMSE variance, but the
   accepted-loop identity is not stable enough for descriptor tuning to be
   meaningful.

2. Pose graph loop closure does not consistently beat the input odometry.

   On the MBES repeat probes, pose graph RMSE is often worse than the upstream
   odometry. A SOTA claim needs the loop-closure layer to improve the trajectory
   under a fair alignment and source-bag protocol.

3. Sonar feedback is both necessary and order-sensitive.

   Disabling `/aqua_sonar_loc/odometry` feedback into `aqua_imu_loc` made loop
   counts more repeatable but destroyed trajectory quality. Keeping feedback is
   necessary, but the node-side timestamp/order handling must be made
   deterministic.

4. False-positive loop protection is still weak.

   Geometry audits exist, but accepted loops are not automatically proven as
   plausible revisits. Current gates use fitness, correction magnitude, plan
   separation, descriptor similarity, and duplicate suppression, but the stack
   still lacks a strong consistency guard that can be trusted as a paper-grade
   false-positive filter.

5. The benchmark set is too thin for a superiority claim.

   One public bag or one sequence is not enough. A fair SOTA statement needs
   held-out windows, multiple sequences, exact configs, a baseline row, and
   comparable sensors.

6. Existing underwater SLAM baselines are broader.

   AQUA-SLAM and related systems include visual/DVL/IMU graph optimization,
   calibration, and multi-sequence evidence. A narrower MBES-specific claim is
   possible, but it must be framed as such and backed by MBES-specific loop
   evidence.

## Main Engineering Plan

### Phase 1 - Deterministic MBES replay

Goal: repeated baseline runs on the same MBES source window produce the same
upstream odometry, same pose-graph keyframe chain, and same accepted loop
endpoint pairs.

Current status:

- strict recorder readiness: done;
- recorder cleanup: done;
- install-prefix profile resolution: done;
- shallow replay queues: done;
- endpoint stamps in status rows: done;
- timestamp-buffered MBES submaps: done;
- sonar feedback diagnostic override: done.

Next tasks:

- Buffer sonar feedback updates in `aqua_imu_loc` by message stamp and apply
  them in a deterministic order relative to IMU samples.
- Decide whether delayed sonar observations should update the UKF immediately,
  be replayed through a small time-sorted queue, or be rejected with an explicit
  status reason.
- Add status counters for accepted/skipped sonar feedback updates so replay
  differences are visible without inspecting bags manually.
- Add a repeatability checker that compares:
  - `/aqua_imu_loc/odometry` pose sequence;
  - `/aqua_pose_graph/keyframe` endpoint stamps;
  - `/mbes_loop_closure/status` accepted endpoint pairs;
  - loop-constraint count and final pose graph RMSE.
- Keep `IMU_SONAR_ODOMETRY_TOPIC=` as a diagnostic only, not a default.

Definition of done:

- two `WEIGHTS=0,0` baseline runs on `beach_pond` have near-identical input
  odometry coverage and stable accepted endpoint pairs;
- accepted-loop endpoint overlap is high enough that descriptor/gate changes
  can be evaluated against the same loop candidates;
- no kilometer-scale open-loop degradation is accepted as a "fix."

### Phase 2 - Candidate and registration determinism

Goal: once upstream odometry/keyframes are stable, the MBES loop front end
should evaluate candidates in a deterministic order and produce repeatable
accepted/rejected status rows.

Tasks:

- Sort candidate submaps by explicit score tuple, not incidental container or
  callback order.
- Make descriptor score, centroid distance, extent ratio, point-count ratio,
  and keyframe separation visible in summary tables.
- Ensure registration backend inputs are deterministically ordered by stamp and
  stable tie-breakers.
- Add a replay test fixture with synthetic keyframes and out-of-order point
  clouds so callback order cannot silently change submaps.
- Add a benchmark-side "accepted endpoint overlap" report.

Definition of done:

- repeated MBES runs produce the same accepted endpoint pairs or a documented
  small delta with the reason visible in status counters;
- descriptor-weight sweeps no longer show improvement dominated by replay
  variance.

### Phase 3 - False-positive guard

Goal: accepted loop constraints should be plausible enough that a trajectory
improvement is not just a lucky bad edge.

Tasks:

- Add batch consistency / clique checks as an enforced gate, not only an audit.
- Promote geometry audit warnings into machine-readable pass/fail categories.
- Add plan-view and depth-delta thresholds that are dataset-configurable and
  documented.
- Calibrate loop information matrices from observed registration residuals
  instead of relying on hand-tuned diagonal values.
- Re-run accepted-loop audit plots after every tuning change.

Definition of done:

- no accepted loop used in a claim lacks geometry coverage;
- high-risk loop rows are either rejected by gate or explicitly reviewed;
- loop constraints improve pose graph RMSE without creating obvious false
  positive shortcuts.

### Phase 4 - Accuracy tuning

Goal: after repeatability and false-positive control, tune for performance.

Tasks:

- Re-run descriptor weight sweeps with repeated baseline rows.
- Tune `candidates.descriptor_weight`, descriptor scales, max distance, minimum
  keyframe separation, fitness gate, correction translation/rotation gates,
  and robust kernel delta.
- Separate "diagnostic best row" from "claimable row."
- Keep public docs updated with root artifact paths and exact commit SHAs.

Definition of done:

- a tuned row improves pose graph RMSE over input odometry and repeated
  baseline;
- improvement survives at least one held-out MBES window or sequence;
- loop audit does not flag unexplained high-risk accepted constraints.

### Phase 5 - SOTA-readiness protocol

Goal: make it hard to accidentally overclaim.

Tasks:

- Add a generated SOTA-readiness report that fails if a claim row lacks:
  - exact source bag and duration;
  - config paths and commit SHA;
  - input odometry baseline;
  - pose graph metric row;
  - loop audit;
  - held-out sequence or window;
  - comparison baseline;
  - limitation note.
- Integrate the report into docs so README wording cannot drift ahead of
  evidence.
- Keep `docs/benchmarks/sota_gap_analysis.md` as the public blocker list.

Definition of done:

- any future "SOTA" wording has a matching evidence packet;
- benchmark tables are script-generated, not hand-edited marketing text.

## Recommended Next Command Sequence

Use this when resuming MBES work from a clean shell:

```bash
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/aqua_mbes_det_log build \
  --packages-up-to aqua_sonar_loc aqua_imu_loc aqua_pose_graph aqua_localization \
  --build-base /tmp/aqua_mbes_det_build \
  --install-base /tmp/aqua_mbes_det_install

source /tmp/aqua_mbes_det_install/setup.bash

pytest -q \
  aqua_localization/test/test_run_mbes_loop_benchmark.py \
  aqua_localization/test/test_record_mbes_demo.py
```

Then run repeated MBES baseline probes only after code-level tests pass:

```bash
OUT_ROOT=/tmp/aqua_mbes_repeat_next \
WEIGHTS=0,0 \
MBES_DURATION=120 \
MIN_DURATION_S=60 \
RECORD_READY_TIMEOUT_S=60 \
SWEEP_ROS_DOMAIN_ID_START=130 \
MBES_SRC_PLAY=/tmp/aqua_mbes_candidate_descriptor_weight_sweep_real/mbes_source_humble_sqlite_180s \
ROS_SETUP=/opt/ros/humble/setup.bash \
LOCAL_SETUP=/tmp/aqua_mbes_det_install/setup.bash \
RECORD_STORAGE=sqlite3 \
PLAY_RATE=0.5 \
PLAY_READ_AHEAD_QUEUE_SIZE=10000 \
PLAY_TIMEOUT_MARGIN_S=0 \
MBES_LOOP_MIN_POINTS=120 \
MBES_LOOP_VOXEL_LEAF_M=0.25 \
MBES_LOOP_MIN_KEYFRAME_SEPARATION=40 \
MBES_LOOP_MAX_CORRECTION_ROTATION_RAD=0.2 \
POSE_GRAPH_LOOP_ROBUST_KERNEL_TYPE=dcs \
POSE_GRAPH_LOOP_ROBUST_KERNEL_DELTA=0.1 \
./aqua_localization/scripts/run_mbes_candidate_descriptor_weight_sweep.sh
```

Do not set `IMU_SONAR_ODOMETRY_TOPIC=` for normal performance runs. Use it only
to isolate the sonar feedback path.

## Public Launch Plan

The public launch work is mostly complete:

- README first-visitor path exists.
- GitHub Pages overview exists.
- OGP title/description/image exist and return HTTP 200.
- Desktop/mobile screenshots were checked.
- Repository description matches `docs/public_launch_checklist.md`.
- Topics are set.
- Release links point to current artifacts.

Remaining public-facing tasks:

- Open small issues from this plan so contributors have obvious entry points.
- Keep the README "Results Snapshot" conservative until MBES loop closure has
  stable evidence.
- Add a short "How to reproduce the MBES determinism probe" issue or doc.
- Publish a 3DGS training smoke result or explicitly document why the small
  pack is only an input artifact.

## Contribution-Sized Issues To Open

Good first technical issues:

- Add a script that compares accepted endpoint pairs between two MBES status
  CSVs and reports overlap.
- Add a script that compares `/aqua_pose_graph/keyframe` endpoint timestamp
  sets between two recorded bags.
- Add `aqua_imu_loc` status counters for sonar feedback accepted/skipped.
- Add markdown generation for the open-loop sonar-feedback diagnostic table.
- Add a small synthetic test for out-of-order sonar feedback observations.

Medium issues:

- Implement timestamp-ordered feedback buffering in `aqua_imu_loc`.
- Enforce batch consistency as a loop acceptance gate.
- Generate SOTA-readiness reports from benchmark artifact directories.
- Add a held-out Tank Dataset benchmark row.
- Add a held-out MBES window benchmark row.

Large issues:

- Visual loop closure for AQUALOC or Tank.
- Error-state Kalman filter backend.
- Learned or data-driven MBES bathymetric loop proposal.
- Full AQUA-SLAM head-to-head with sensor-equivalent configuration.

## Current Non-Goals

- Do not claim SOTA on MBES loop closure yet.
- Do not use open-loop sonar-feedback-disabled RMSE as a performance result.
- Do not treat a single replay run as enough evidence for descriptor tuning.
- Do not hide rows where pose graph is worse than input odometry.
- Do not add unrelated refactors while determinism is being debugged.

## One-Sentence Handover

The stack is public-demo-ready and research-useful, but the next real step is
not more marketing or broad feature work: keep sonar feedback enabled, make
its timestamp/order handling deterministic, then re-run repeated MBES baselines
until accepted loop identity is stable enough for loop-candidate tuning to mean
something.

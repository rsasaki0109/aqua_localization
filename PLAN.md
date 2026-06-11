# aqua_localization Plan and State of the Stack

This is the canonical "where we are, where we are going" document for
`aqua_localization`. Keep it practical: it should tell a returning contributor
what is already solid, what is still experimental, and what the next useful
engineering move is.

Current date: 2026-06-11.
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

The strongest claimable result is the Tank Dataset visual-aided fusion row:
IMU + pressure + DVL + stereo-ORB visual position updates on `short_test` at
**0.2140 m APE RMSE** (realtime 1.0x replay, 300/300 visual coverage,
2026-06-11; supersedes the 0.2175 m row and the older 0.43 m DVL+pressure
headline). A 0.0154 m diagnostic row (`aqua_dvl_prior_visual`, beats
AQUA-SLAM's 0.0194 m) exists but is same-sequence-tuned and stays
non-claimable until held-out sequences arrive. The MBES path is a working
research track, not a SOTA claim.

## 2026-06-11 Work-Loss Incident and Reconstruction (READ FIRST)

**What happened.** A session on 2026-06-11 (afternoon) discovered that the
entire "Since v0.5 (in-tree)" code described below — the `shift_from` lag
compensation, the stamp-ordered sonar feedback buffer, the EstimatorStatus
counters, `compare_mbes_repeat_probe.py`, the recorder hardening, the tank
config changes — was **never committed to git**. Only this PLAN.md (commit
`bce0292`) made it into the repository. The implementation lived as
uncommitted working-tree state in the previous dev workspace at
`/media/sasaki/aiueo/ai_coding_ws/aqua_loc_ws`, and that volume is no longer
mounted (`/media/sasaki/` is empty; last mount activity March 2026 per
directory mtime). The current canonical checkout is
`/home/sasaki/workspace/aqua_loc_ws/aqua_localization`, synced with
`origin/main`, and it contained none of the described code. Every claim in
this document that says "done (in-tree)" was, at that point, documentation of
lost work.

Evidence trail, for the record:

- `grep -rln "PositionHistoryBuffer|buffer_by_stamp|sonar_feedback_received|
  snap_first_update"` over the repo matched **only PLAN.md**;
- `git log --all` had no commit touching `compare_mbes_repeat_probe.py`,
  `setup_nested_workspace_env.sh`, or `shift_from`;
- `aqua_msgs/msg/EstimatorStatus.msg` had no sonar feedback counters;
- `imu_loc_node.cpp` was at the pre-fix state the "Original Diagnosis"
  section describes (no history buffer, no snap, no caps, direct
  `update_position` in the subscription callback);
- the v0.5-era `record_mbes_demo.sh` has a `cleanup_processes()` and
  `resolve_profile()`, but the pre-hardening versions (no child-PID KILL
  escalation, no readability re-check, no overlay preference, no
  `ROS_DOMAIN_ID`/startup-delay defaults).

**Lesson (process change, mandatory).** Commit after every validated step.
The reconstruction below exists because PLAN.md recorded the design in
enough detail to rebuild from; artifacts (/tmp probe dirs, benchmark rows)
were NOT recoverable. "In-tree" is not a state — committed or lost.

### Reconstructed and verified (committed)

Commit `4cea97a` "Reconstruct lag-compensated position fusion and
stamp-ordered sonar feedback" rebuilds the C++ core from this document's
design notes, on ROS 2 Jazzy (`/opt/ros/jazzy`), with the nested-workspace
build inside the repo (`colcon build --packages-select aqua_msgs
aqua_imu_loc --symlink-install`; `build/`/`install/`/`log/` at repo root).
All 9 `aqua_imu_loc` ctest targets pass, including the new ones.

- `aqua_imu_loc/include/aqua_imu_loc/position_history_buffer.hpp` + `src/`:
  per-IMU-step `(stamp, position)` deque with `configure(horizon_s)`,
  interpolating `lookup()` (exact / interpolated / clamp-to-newest /
  nullopt-when-older-than-buffer), and `shift_from(stamp, delta)` folding
  applied corrections into entries at/after the measurement stamp.
- `aqua_imu_loc/include/aqua_imu_loc/feedback_odometry_buffer.hpp` + `src/`:
  stamp-ordered staging buffer (`push` keeps ascending stamp order, stable
  for equal stamps; `drain_through(stamp)` pops everything at/before).
- `imu_loc_node.cpp`: subscription callbacks reduced to
  `to_observation()`; one shared `apply_position_observation()` does
  nonfinite/stale gating, floor + cap on the covariance diagonal, lag
  compensation (`z' = x_now + (z - x(t_m))` when the history lookup
  succeeds, uncompensated fallback otherwise), the update, then
  `shift_from`. Sonar path: `imu.sonar.buffer_by_stamp` (default true)
  stages observations and the IMU step drains them after prediction +
  history push. Visual path: `imu.visual.snap_first_update` (default
  false) applies the first visual fix with ~1e-9 covariance.
- New parameters: `imu.position_history.horizon_s` (2.0),
  `imu.visual.max_position_variance` / `imu.sonar.max_position_variance`
  (0.0 = disabled), `imu.visual.snap_first_update` (false),
  `imu.sonar.buffer_by_stamp` (true).
- `EstimatorStatus.msg`: `sonar_feedback_received / applied /
  skipped_stale / skipped_nonfinite / pending` (received == applied +
  skipped_* + pending).
- `tank_dataset.yaml`: visual `max_position_variance: 0.005`,
  `snap_first_update: true` per the best validated row's config.
- Tests: `test_position_history_buffer.cpp` (8, incl.
  `ShiftFromAppliesCorrectionToTailOnly`,
  `ShiftFromStopsDelayedMeasurementReapplication`),
  `test_feedback_odometry_buffer.cpp` (5, incl. shuffled-arrival
  determinism), runtime additions
  `SnapFirstVisualUpdateJumpsToMeasurement` and
  `SonarFeedbackBufferAppliesByStampAndReportsCounters`.

Tooling reconstructed in the same session (this commit):

- `record_status.py`: CSV header + row extended with the five sonar
  feedback counters (getattr fallback keeps it usable against pre-counter
  nodes); `test_record_status_format.py` extended, 6/6 pass.
- `compare_mbes_repeat_probe.py`: rebuilt with input-odometry SHA256 /
  sample-count / duplicate-stamp diff (duplicate stamps trigger the
  leaked-node warning in the report), the "Windowed inter-run agreement"
  section (raw common-window diff, no alignment;
  `--window-agreement-mean-max-m` default 1.0, `--require-window-agreement`
  exit code), accepted endpoint-pair overlap from `mbes_loop_status.csv`,
  and estimator-status tail counters. Smoke-tested end to end (hash
  mismatch, duplicate-stamp flag, gate FAIL exit 1 all verified); its
  dedicated pytest file is still to be rewritten (below).

### Still lost — not yet reconstructed

Ordered roughly by next-session priority:

1. `test/test_compare_mbes_repeat_probe.py` (original had 5 tests).
2. `record_mbes_demo.sh` hardening v2: `cleanup_processes()` child-PID
   snapshot → INT → wait-for-death (8 s) → TERM → `pkill -KILL -P` → KILL;
   `resolve_profile()` readability re-check after `ros2 pkg prefix`;
   `prefer_workspace_install_overlays()` prepending `WORKSPACE/install/*`
   to `AMENT_PREFIX_PATH`/`LD_LIBRARY_PATH`; defaults `ROS_DOMAIN_ID=42`,
   `NODE_STARTUP_DELAY_S=5`, `POST_PLAY_SLEEP_S=3`.
3. `run_mbes_loop_benchmark.sh`: forward `ROS_DOMAIN_ID`,
   `NODE_STARTUP_DELAY_S`, `POST_PLAY_SLEEP_S` into the recorder.
4. `run_tank_visual_fusion_benchmark.py`: record the visual input topic to
   `*_visual_input.tum` in the same replay; default the regression gate to
   the same-run visual RMSE (`--standalone-visual-rmse-m` override;
   negative disables); explicit FAIL/warn row when fused > visual.
5. `acquire_mbes_beach_pond.py` (wget tarball + optional ROS 2 conversion),
   `probe_sonar_feedback_repeatability.py`,
   `scripts/setup_nested_workspace_env.sh`,
   `export_estimator_status.py` (was an open item even pre-loss).
6. Probe drivers (`run_probe.sh` with pre-flight stale-node guard) lived in
   /tmp and are gone; re-derive from the "Probe hygiene" notes below.

### Consequences for results and claims

- **All measured rows below are now historical.** The 0.2140 m headline,
  the probe #1–#5 numbers, and the A/B tables were produced by the lost
  binaries. The reconstruction follows this document's design faithfully,
  but it has NOT been re-validated against data — it must not be claimed
  until the Tank visual fusion benchmark is re-run and lands in the same
  neighborhood (fused ≈ 0.214 m, fused-vs-same-run-visual gap ≈ +0.02 m).
- **Datasets are absent on this machine.** `datasets/public/` is empty:
  Tank `short_test` (bag + ROS 2 conversion), `beach_pond_ros2` (mcap),
  and the GT TUM exports all need re-acquisition before any replay work.
- Findings that survive regardless (they are conclusions, not artifacts):
  leaked-node hygiene, shallow-QoS-for-accuracy, deep-queues-fix-counts-
  but-worsen-determinism, sonar-feedback-is-order-sensitive, and the
  per-axis error decomposition pointing at DVL x bias.

### Environment facts (current machine)

- Canonical checkout: `/home/sasaki/workspace/aqua_loc_ws/aqua_localization`
  (git, synced with origin). The `/media/sasaki/aiueo/...` paths in the
  command sequences below are STALE — substitute the new prefix.
- ROS 2: Jazzy only (`/opt/ros/jazzy`); no Humble install here.
- 8 cores; box was idle (load < 1) during reconstruction — good for the
  determinism A/Bs once datasets are back.

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

### Since v0.5 (2026-06-10 to 2026-06-11; lost, then partially reconstructed)

> NOTE: the work in this section was never committed and was lost with the
> previous workspace. See "2026-06-11 Work-Loss Incident and Reconstruction"
> above for what has been rebuilt and what is still missing. The text below
> is kept as the reconstruction spec.

**Sonar feedback determinism (C++ / messages):**

- `aqua_imu_loc` buffers sonar odometry by message stamp (`imu.sonar.buffer_by_stamp`,
  default true) and drains in stamp order on each IMU step.
- `EstimatorStatus` carries `sonar_feedback_received/applied/skipped_stale/
  skipped_nonfinite/pending`.
- Unit tests: `test_feedback_odometry_buffer`, updated `test_imu_loc_node_runtime`.
- `probe_sonar_feedback_repeatability.py` confirms buffer logic is deterministic
  at unit-test level.

**Benchmark / acquisition tooling:**

- `compare_mbes_repeat_probe.py` — artifact-directory diff (odometry TUM hash,
  accepted loop pairs, EstimatorStatus tail counters).
- `acquire_mbes_beach_pond.py` — wget tarball + optional ROS2 conversion.
- `record_status.py` CSV header extended for sonar feedback counters.

**Recorder / nested-workspace replay fixes (`record_mbes_demo.sh`):**

- `resolve_profile()` verifies readability after `ros2 pkg prefix` lookup so a
  stale outer overlay does not silently win over a valid nested config.
- `prefer_workspace_install_overlays()` prepends `WORKSPACE/install/*` to
  `AMENT_PREFIX_PATH` and `LD_LIBRARY_PATH` so nested `aqua_sonar_loc` installs
  (e.g. `mbes_loop_closure_node`, `mbes_loop_closure.yaml`) are found when an
  outer `aqua_loc_ws/install` overlay is also sourced.
- Defaults for repeat probes: `ROS_DOMAIN_ID=42` when unset, `NODE_STARTUP_DELAY_S=5`,
  `POST_PLAY_SLEEP_S=3` (overridable; benchmark wrapper forwards these).
- `run_mbes_loop_benchmark.sh` passes `ROS_DOMAIN_ID`, `NODE_STARTUP_DELAY_S`,
  `POST_PLAY_SLEEP_S` into the recorder subprocess.

**Visual fusion + Tank benchmark (C++ / config / tooling, 2026-06-11):**

- `PositionHistoryBuffer::shift_from()` — folds every applied position
  correction back into the lag-compensation history (visual and sonar paths);
  kills the positive-feedback failure (fused 0.4446 → 0.1979 m diagnostic).
- `imu.visual.snap_first_update` — first absolute visual update is trusted
  near-fully, absorbing the frontend-warmup dead-reckoning transient
  (verified live: 0.516 m first innovation absorbed in one step).
- `run_tank_visual_fusion_benchmark.py` records the visual input topic in the
  same replay (`*_visual_input.tum`) and gates fused-vs-visual on the
  same-run value by default.
- Tank config: `imu.visual.max_position_variance` capped at 0.005 (the
  frontend publishes pessimistic 0.04 m² covariance; the variance *floor* is
  inert — only the cap changes the weighting).
- New valid 1.0x rows; best fused 0.2140 m retires the 0.2175 m public row.

**Replay hygiene + determinism tooling (2026-06-11):**

- Leaked-node cross-talk discovered: nodes from interrupted runs survive on
  the same `ROS_DOMAIN_ID` and double-publish into later replays (observed
  2x `/aqua_imu_loc/odometry` samples). `record_mbes_demo.sh`
  `cleanup_processes()` hardened (child-PID snapshot → INT → wait → TERM →
  KILL); probe drivers add pre-flight stale-node guards.
- `compare_mbes_repeat_probe.py` gains a "Windowed inter-run agreement"
  section: raw common-window position diff (no alignment),
  `--window-agreement-mean-max-m` gate (default 1.0 m),
  `--require-window-agreement` exit code. Tests:
  `test/test_compare_mbes_repeat_probe.py`.
- QoS depth 5-vs-50 A/B: deep queues equalize sample counts but worsen both
  determinism and accuracy — keep `IMU_QOS_SENSOR_DEPTH=5`.

**Operational pitfall (documented):**

- The nested `aqua_localization` workspace must be built and sourced via
  `scripts/setup_nested_workspace_env.sh` with `AQUA_LOC_WS` pointing at the
  parent `aqua_loc_ws`. Using only the outer install tree can miss
  `mbes_loop_closure_node` or an outdated `mbes_loop_closure.yaml`.
- Post-probe comparison still requires extracting `EstimatorStatus` from
  recorded bags (no `export_estimator_status.py` yet); use bag replay or a
  short `rosbag2_py` script and symlink `input_odometry.tum` /
  `mbes_loop_status.csv` into the probe directory for `compare_mbes_repeat_probe.py`.
- Before ANY replay run: verify no stale aqua nodes are alive
  (`pgrep -af "imu_loc_node|sonar_loc_node|pose_graph_node|mbes_loop_closure_node"`).
  `pkill -f` from an agent shell can silently fail on orphaned nodes; use
  direct `kill -KILL <pid>` and re-verify.

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
- Sensors: IMU, depth/pressure-derived z, DVL, stereo camera (ORB frontend),
  AprilTag ground truth.
- Current headline: **0.2140 m APE RMSE** for IMU+pressure+DVL+visual fusion
  (1.0x replay, 300/300 coverage, shift fix + first-visual snap, 2026-06-11).
  DVL+pressure-only baseline remains 0.43 m.
- Per-axis state: fused wins z (0.037 vs visual 0.175, pressure anchor),
  matches y, loses only x (0.205 vs 0.082) — a DVL x scale/extrinsic bias is
  the dominant remaining error and the main lever toward AQUA-SLAM's 0.0194 m.
- Scale remains same-sequence-fit; broad claims still need the held-out
  protocol.

Next work:

- Convert the result into a stricter benchmark table with exact source bag,
  config, commit, alignment statement, and failure gates.
- Compare fairly with AQUA-SLAM under sensor-equivalent conditions.
- Add held-out Tank windows, not only `short_test`.
- **Blocked:** `Structure_Easy.bag` and `Medium.bag` requested via official form;
  awaiting email delivery before `acquire_tank_heldout_sequences.py --convert-all`
  and held-out calibration (`run_tank_sequence_heldout_calibration.py`).

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

Most recent determinism results (historical probes):

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

**E2E repeat probe #1** (`/tmp/aqua_mbes_repeat_probe`, 2026-06-11):

- Protocol: `WEIGHTS=0,0`, `MBES_DURATION=120`, nested Jazzy workspace, default-ish
  replay timing (no `PLAY_RATE=0.5`), `SWEEP_ROS_DOMAIN_ID_START` → run1=130,
  run2=131.
- Dataset: `datasets/public/mbes_slam/beach_pond_ros2` (mcap, ~120 s window).
- Compare report: `/tmp/aqua_mbes_repeat_probe/mbes_repeat_probe_compare.md`.

| Metric | run1 (`weight_0`) | run2 (`weight_0_run2`) | Match? |
|--------|-------------------|------------------------|--------|
| `input_odometry.tum` SHA256 | `e74eca41…` | `f2f26d10…` | no |
| Input odometry samples | 11889 | 11919 | no |
| Input odometry APE RMSE m | 124.33 | 84.46 | no |
| Pose graph APE RMSE m | 146.82 | 94.23 | no |
| Accepted loop pairs | 0 | 0 | yes (vacuous) |
| `sonar_feedback_received` | 200 | 218 | no |
| `sonar_feedback_applied` | 200 | 218 | no |
| `update_count` | 12088 | 12136 | no |
| `skipped_stale` / `nonfinite` | 0 / 0 | 0 / 0 | yes |

Interpretation: C++ feedback-buffer unit tests are deterministic, but the full
replay pipeline is **not**. The sonar feedback counter gap (18 messages) and
`update_count` gap (48 IMU steps) point to timing / callback-order sensitivity
between `aqua_sonar_loc` → `aqua_imu_loc`, not stale-skip policy (stale counter
stays 0). Trajectory RMSE also diverges materially even with zero accepted loops.

**E2E determinism isolation attempt #2** (`/tmp/aqua_mbes_repeat_det`, 2026-06-11):

- Tightened replay env (intended to pin startup and DDS):
  - `ROS_DOMAIN_ID=130` (same for both runs);
  - `PLAY_RATE=0.5`, `PLAY_READ_AHEAD_QUEUE_SIZE=10000`;
  - `PLAY_START_DELAY_S=35`, `PLAY_TIMEOUT_MARGIN_S=15`;
  - `PLAY_WAIT_FOR_ALL_ACKED_MS=5000`;
  - `RECORD_READY_TIMEOUT_S=90`, `NODE_STARTUP_DELAY_S=10`, `POST_PLAY_SLEEP_S=5`;
  - `IMU_QOS_SENSOR_DEPTH=5`, `SONAR_QOS_SENSOR_DEPTH=5`;
  - `MBES_LOOP_CANDIDATE_DESCRIPTOR_WEIGHT=0.0`.
- Run1 (`weight_0_run1`) **completed**:
  - bag: `/tmp/aqua_mbes_repeat_det/bags/mbes_weight_0_run1` (~430 MB mcap);
  - input odometry: 12715 samples, APE RMSE **95.10 m** (vs reference);
  - pose graph RMSE **111.86 m**; 0 accepted loops (submap min_points gate).
- Run2 (`weight_0_run2`) **interrupted** before export finished; partial bag exists
  at `/tmp/aqua_mbes_repeat_det/bags/mbes_weight_0_run2` — **re-run required**
  before a second compare report.

The important finding is not "MBES loop closure is solved." It is:

- message-stamp submap assembly helps upstream variance;
- shallow queues are necessary;
- strict recorder readiness is necessary;
- sonar feedback must stay enabled for performance;
- stamp-ordered sonar **buffering in C++ is necessary but insufficient** for E2E
  repeatability;
- the remaining blocker is end-to-end timing: node startup order, sim-time clock
  sync, bag-play delay/ack waits, and possibly sonar/IMU callback scheduling
  before Phase 2 candidate-order work matters;
- nested-workspace overlay order must be correct or probes silently use wrong
  configs/nodes.

**E2E repeat probe #3 — leaked-node contamination found (`/tmp/aqua_mbes_repeat_det2`, 2026-06-11):**

- Same env as attempt #2 but with the shift-fixed `aqua_imu_loc` node
  (`PositionHistoryBuffer::shift_from` now applied to sonar feedback).
- Run1: 12678 samples, input APE RMSE **53.91 m** (vs 95.10 m in attempt #2 —
  shift fix materially improves the sonar-feedback trajectory). Zero duplicate
  stamps in `input_odometry.tum`.
- Run2: **25263 samples ≈ 2× run1, with 12615 of 12647 stamps duplicated
  (max multiplicity 3)** and APE RMSE 871 m. Two-plus `imu_loc_node`
  instances published `/aqua_imu_loc/odometry` into the same recording.
- Root cause: **leaked nodes from earlier runs/sessions survive on the same
  `ROS_DOMAIN_ID`.** `ps` showed 15 stale aqua nodes (imu_loc / sonar_loc /
  pose_graph / mbes_loop_closure) aged 2–6 h, orphaned from previous probes.
  `record_mbes_demo.sh` cleanup only sent INT/TERM to the `ros2 run` wrapper
  PIDs — node binaries can outlive that, then wake up when the next replay
  publishes on their domain. This retroactively taints probes #1/#2:
  hash-mismatch conclusions there cannot be attributed to callback scheduling
  until reproduced with a verified-clean process table.
- Fixes landed:
  - `record_mbes_demo.sh` `cleanup_processes()` hardened: INT → wait-for-death
    (up to 8 s) → TERM → `pkill -KILL -P` on each wrapper's children → KILL.
  - Probe driver (attempt #4: `/tmp/aqua_mbes_repeat_det3/run_probe.sh`) adds a
    pre-flight guard (abort if any aqua node is alive) and a post-run leak
    check + kill.
- Operational notes: `pkill -f <node name>` from the agent shell did NOT kill
  the orphaned nodes (sandbox/signal delivery quirk); direct `kill -KILL <pid>`
  works. Always verify with `pgrep -c -f ...` after killing.

**E2E repeat probe #4 — first trustworthy verdict (`/tmp/aqua_mbes_repeat_det3`, 2026-06-11):**

- Verified-clean process table before each run (pre-flight guard), shift-fixed
  node, hardened cleanup, same env as attempt #2. Both runs finished with
  **zero duplicate stamps** (single publisher confirmed).
- Result: hashes still differ. run1 12694 samples / input APE RMSE 56.43 m;
  run2 12660 samples / 51.56 m. Compare report:
  `/tmp/aqua_mbes_repeat_det3/mbes_repeat_probe_compare.md`.
- First stamps are identical (start is pinned); the runs genuinely diverge
  in-flight: inter-run raw position diff (common window, no alignment) is
  already 13 m at t+13 s, mean 26.9 m, max 219 m. This is real estimator-path
  nondeterminism (sonar registration feedback amplifying timing noise), not a
  boundary/recording artifact.
- The det2 pose-graph "blow-up" (RMSE ~1e5 m) **did not reproduce** on a clean
  table (run1 53.19 m / run2 57.96 m, sane sample counts) — it was stale-node
  cross-talk, not a pose_graph_node bug. Closed.
- Suspect for the ±34 sample-count gap: `IMU_QOS_SENSOR_DEPTH=5` /
  `SONAR_QOS_SENSOR_DEPTH=5` shallow queues dropping a slightly different
  message subset each run under machine load (box was at load ~25-37 during
  these runs). Untested hypothesis — needs an idle-machine A/B (depth 5 vs 50)
  before concluding.
- Caveat: both runs ran while heavy non-ROS jobs (yolo training etc.) loaded
  the box. Timing-sensitivity bisects on a loaded machine are noisy; redo the
  key A/Bs when idle.

**E2E repeat probe #5 — QoS depth 5 vs 50 A/B (`/tmp/aqua_mbes_repeat_det4`, 2026-06-11):**

- det3 protocol with `IMU_QOS_SENSOR_DEPTH=50` / `SONAR_QOS_SENSOR_DEPTH=50`,
  clean process table both runs (hardened cleanup v2 — child-PID snapshot
  before INT — leaked nothing).
- Sample counts: 12699 vs 12702 (**gap 34 → 3**). Queue drops at depth 5 are
  confirmed as the source of the sample-count gap.
- But trajectory determinism got worse, not better: windowed inter-run mean
  diff 84.6 m (depth 5: 26.9 m), max 387 m; and accuracy vs reference degraded
  to 82.0/88.4 m RMSE (depth 5: 56.4/51.6 m). "Shallow queues are necessary
  for accuracy" survives the leaked-node retest; deep queues fix neither
  determinism nor accuracy.
- Conclusion: the in-flight divergence is NOT message loss — it is timing /
  callback-order sensitivity in the sonar registration → UKF feedback loop
  amplifying chaotically. Caveat: load was ~35-39 during both A/B arms; one
  run per config; idle-machine confirmation still advisable before deep work.
- Tooling: `compare_mbes_repeat_probe.py` now reports a
  "Windowed inter-run agreement" section (raw common-window diff, no
  alignment; gate `--window-agreement-mean-max-m`, default 1.0 m, optional
  `--require-window-agreement` exit code). Unit tests:
  `test/test_compare_mbes_repeat_probe.py` (5 tests).

Next work on this track (ordered):

1. Keep `IMU_QOS_SENSOR_DEPTH=5` (accuracy-optimal). Attack callback-order
   sensitivity directly: add explicit node-ready waits, log first/last sonar
   feedback stamps per run, bisect `PLAY_START_DELAY_S` vs
   `PLAY_WAIT_FOR_ALL_ACKED_MS`; consider making the sonar feedback
   application step idempotent w.r.t. arrival order (apply strictly on IMU
   stamp boundaries from the stamp-ordered buffer, which was the Phase 1
   design intent — verify it actually drains that way under replay).
2. Phase 2 gate decision: exact hash equality is demonstrably unattainable;
   adopt the windowed gate once a defensible threshold is met (current
   distance: mean 26.9 m at depth 5 — three orders of magnitude away, so the
   gate change alone does not unblock Phase 2).
3. Add `export_estimator_status.py` (bag → CSV) and wire into benchmark wrapper.
4. Optional CMake: install `mbes_loop_closure.yaml` from `aqua_sonar_loc`;
   install `compare_mbes_repeat_probe.py` for `ros2 run`.

Probe hygiene (mandatory from now on): before ANY replay probe/benchmark run,
`pgrep -af "imu_loc_node|sonar_loc_node|pose_graph_node|mbes_loop_closure_node"`
must come back empty — leaked nodes from interrupted runs wake up on the next
replay and double-publish. `pkill -f` from an agent shell may silently fail to
signal orphaned nodes; use direct `kill -KILL <pid>` and re-verify. The
attempt-#4 driver (`/tmp/aqua_mbes_repeat_det3/run_probe.sh`) encodes this.

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

1. Repeatability is still not strong enough on MBES — including upstream odometry.

   Probes #1/#2 were contaminated by leaked nodes from interrupted runs
   (discovered 2026-06-11); with that fixed and a verified-clean process
   table, probe #4 still shows genuine in-flight divergence between identical
   runs (windowed inter-run mean diff 26.9 m, ±34 samples). The QoS A/B
   (probe #5) ruled out message drops as the trajectory-divergence driver.
   The remaining suspect is callback-order sensitivity in the sonar
   registration → UKF feedback loop. Descriptor tuning remains meaningless
   until the windowed inter-run agreement gate passes at a defensible bound.

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

## Visual Fusion Regression - RESOLVED ROOT CAUSE (2026-06-11)

**Status: fixed in-tree AND validated at 1.0x on a quiet machine (see "1.0x
validation" below); the 0.2175 m public row is retired by 0.2140 m.**

### What was actually wrong (beyond the 2026-06-10 diagnosis)

The Fix A lag-compensation implementation (`position_history_` ring buffer +
`lag_compensated_position_measurement()`) had a **positive feedback flaw**:
filter corrections were never folded back into the stored history. A delayed
visual sample arriving after a correction looked up the *uncorrected* past
position, re-measured the error the filter had already absorbed, and re-applied
it. With several queued/late samples this over-corrects and destroys the
trajectory.

Measured A/B (same code, same machine load, 0.25x replay, 300/300 coverage):

| Condition | Fused SE(3) RMSE |
|-----------|-----------------:|
| lag comp, `max_age_s=1.0` (late samples admitted) | 0.4446-0.4491 m |
| lag comp, `max_age_s=0.25` (late samples dropped) | 0.2480 m |
| lag comp + **history shift fix**, `max_age_s=1.0`, run1 | **0.1979 m** (same-run visual 0.1842) |
| lag comp + history shift fix, `max_age_s=1.0`, run2 | 0.2208 m (same-run visual 0.1959) |

The run1/run2 spread tracks the same-run visual input spread, i.e. it is
replay/load variance, not filter instability.

The fix: `PositionHistoryBuffer::shift_from(stamp, delta)` — after every
position update (visual and sonar paths), the applied correction is added to
history entries at/after the measurement stamp, so later delayed measurements
see the corrected past. Unit tests:
`ShiftFromAppliesCorrectionToTailOnly`,
`ShiftFromStopsDelayedMeasurementReapplication`.

This also de-risks the MBES Phase 1 sonar feedback path, which shares the same
buffer and was the suspected source of E2E nondeterminism sensitivity.

### Benchmark tooling change (same-run gate)

`run_tank_visual_fusion_benchmark.py` now records the visual input topic to
`*_visual_input.tum` during the same replay and reports
`same-run visual input SE(3) RMSE`. The regression gate defaults to that
same-run value instead of the historical hardcoded 0.0947 m, because the
0.0947 row came from a 200-sample partial-coverage run and is not
load-comparable. CLI `--standalone-visual-rmse-m` still overrides; negative
disables.

### Context that recalibrates the old acceptance criteria

- Under current machine load (other experiments pinning all cores) the
  standalone visual frontend itself measures ~0.18-0.20 m at 0.25x replay,
  not 0.0947 m. The honest target is "fused ≤ same-run visual", which is now
  within 0.014 m; the absolute numbers need an idle machine + 1.0x replay for
  a claimable row.
- Remaining known contributor at the time: the initial transient before the
  first visual update. **Implemented and now validated:**
  `imu.visual.snap_first_update` (true in the Tank config, runtime test
  `SnapFirstVisualUpdateJumpsToMeasurement`).

### 1.0x validation (2026-06-11, quiet machine, load1 ≈ 14-16)

Four valid realtime rows, all snap on, full/near-full coverage
(details in `docs/benchmarks/tank_aqua_slam.md`):

| Config | Coverage | Fused | Same-run visual | Gap |
|--------|---------:|------:|----------------:|----:|
| max age 0.25 | 300/300 | 0.2161 | 0.1959 | +0.020 |
| max age 1.0 | 294/300 | 0.2169 | 0.1912 | +0.026 |
| age 1.0, variance floor 0.0025 | 294/300 | 0.2196 | 0.1912 | +0.028 |
| age 1.0, **variance cap 0.005** | 300/300 | **0.2140** | 0.1959 | +0.018 |

Findings:

- Snap verified live: first innovation 0.516 m absorbed in one step
  (`moved_xy=0.5156, cov_xy≈0`).
- The variance *floor* knob is inert (frontend publishes 0.04 m² covariance,
  above any sensible floor); only `imu.visual.max_position_variance` (cap)
  changes the visual weighting. Tank config now caps at 0.005.
- The filter tracks its visual input in XY at 3.7-5.7 cm — the fusion loop is
  healthy. The fused-vs-visual SE(3) gap decomposes per-axis as: fused wins z
  decisively (0.037 vs 0.175; pressure anchor vs visual z-drift of
  −0.68±0.36 m), matches y, loses only x (0.205 vs 0.082).
- x is the same dominant axis as the pre-fix diagnosis and the axis the
  `aqua_dvl_prior_visual` diagnostic (0.0154 m) fixes via same-sequence DVL
  prior tuning. **Next accuracy lever: correct the DVL x scale/extrinsic bias
  in a held-out-transferable way** (mount angle, per-axis DVL variance, lever
  arm) instead of per-sequence priors.

## Visual Fusion Regression - Original Diagnosis and Handoff (2026-06-10)

This section is the active handoff for the highest-ROI accuracy work: closing
the gap to AQUA-SLAM on Tank `short_test`. It records a code-level diagnosis
made on 2026-06-10 so the next contributor (human or agent) can implement and
measure without re-deriving it.

### Problem statement

Measured rows (`docs/benchmarks/tank_aqua_slam.md`, SE(3) alignment):

- AQUA-SLAM: 0.0194 m RMSE.
- `aqua_visual_frontend` standalone: 0.0947 m RMSE.
- `aqua_localization+visual` (fused, best row): 0.2175 m RMSE.

The fused estimate is 0.1228 m **worse** than its best input. A fusion layer
should never lose to its best sensor, so this regression is a bug-class
problem, not a tuning problem. Fixing it alone moves the headline row from
11.2x to 4.9x of AQUA-SLAM with zero new sensors.

### Root-cause diagnosis (code-grounded, in priority order)

1. **No measurement-delay compensation on visual position updates.**
   `apply_position_odometry()` in `aqua_imu_loc/src/imu_loc_node.cpp`
   (lines ~298-339) accepts a visual `Odometry` whose stamp may be up to
   `imu.visual.max_age_s` (default **1.0 s**) older than the latest IMU
   stamp, then passes it to `AdditiveUkf::update_position()`
   (`additive_ukf.cpp` ~245), which compares it against the **current**
   state. There is no state-history buffer. The fusion benchmark
   (`run_tank_visual_fusion_benchmark.py`) runs `stereo_visual_odometry.py`
   live during replay, so its PnP processing latency (700 ORB features) is
   real wall-clock latency. Every stale update drags the estimate backward
   along the trajectory by roughly `vehicle_speed × measurement_age`. This
   is the leading suspect for the 0.12 m regression.

2. **Visual z contaminates the pressure-depth channel.**
   `update_position()` updates x, y, z jointly. Pressure depth is the
   stack's best channel; visual z carries PnP noise plus extrinsic error.
   The per-axis floor in `apply_position_odometry()` applies the same floor
   to z as to x/y, so a noisy visual z can fight the pressure update.

3. **Temporally-correlated visual drift fused as white noise.**
   The frontend publishes an absolute integrated pose at up to 20 Hz with
   per-axis variance `max(0.04, 2/inliers)`
   (`scripts/stereo_visual_odometry.py` ~396). Fusing a slowly-drifting
   absolute track as independent measurements collapses the UKF position
   covariance and over-trusts the visual track; DVL/pressure can no longer
   correct it. Note `imu.visual.position_variance_floor` (default 0.04)
   also caps how humble the update can be made without code changes.

### Fix plan

- **Fix A (lag compensation, node-level, no UKF API change).**
  In `ImuLocNode`, keep a small ring buffer of `(stamp, filter position)`
  appended on every IMU prediction (~2 s depth). When a visual/sonar
  position with stamp `t_m` arrives, look up the buffered position
  `x(t_m)`, then call `update_position()` with the pseudo-measurement
  `z' = x_now + (z - x(t_m))` so the innovation is evaluated at measurement
  time. Apply to both the visual and sonar paths (the sonar feedback
  determinism work in Phase 1 wants the same buffer). Also drop the default
  `imu.visual.max_age_s` from 1.0 to ~0.25 s.

- **Fix B (z decoupling).**
  Add `imu.visual.fuse_z` (default `true` for back-compat; set `false` in
  the Tank config). When false, inflate the z variance (e.g. 1e6) and zero
  the z row/column off-diagonals before `update_position()`, so pressure
  owns depth.

- **Fix C (covariance honesty, after A/B are measured).**
  Subsample visual updates and/or grow the published variance with time
  since the last visual anchor; longer-term, fuse relative visual deltas
  instead of an absolute drifting track. Run the planned sweep from
  `docs/benchmarks/aqua_slam_error_budget.md` (`run_tank_visual_fusion_sweep.py`
  with `imu.visual.position_variance_floor` × `imu.visual.max_age_s` pairs)
  only after Fix A, otherwise the sweep optimizes around the lag bug.

- **Fix D (regression gate).**
  Make `run_tank_visual_fusion_benchmark.py` emit an explicit FAIL/warn row
  when fused RMSE > standalone visual RMSE, so this class of regression can
  never be reported silently again.

### Acceptance criteria

- Fused row on `short_test` ≤ 0.0947 m (standalone visual), same alignment
  and bag window as the existing rows.
- DVL+pressure-only row (0.4291 m) does not regress.
- A unit test covers the state-history lookup (exact stamp, interpolated
  stamp, stamp older than buffer).

### Reproduction state (what is already done / known pitfalls)

- `/tmp/short_test_ros2_visual` was regenerated on 2026-06-10 with:
  `ros2 run aqua_localization convert_tank_dataset_bag.py
  --src aqua_localization/datasets/public/tank_dataset/short_test.bag
  --dst /tmp/short_test_ros2_visual --include-cameras`
  (sqlite3 storage; 104 DVL→TwistStamped, 6341 passthrough). `/tmp` is
  volatile — regenerate if missing.
- GT reference: export `/apriltag_slam/GT` from
  `datasets/public/tank_dataset/short_test_ros2` with
  `export_rosbag_odometry_tum.py` to `/tmp/tank_short_test_gt.tum`.
- **Pitfall:** the workspace `install/` tree is stale — it contains only ~21
  `aqua_localization` scripts while the source tree has far more
  (`export_rosbag_odometry_tum.py` is missing from install). Rebuild first:
  `colcon build --packages-select aqua_localization --symlink-install`,
  and after C++ changes also `--packages-select aqua_imu_loc`.
- Baseline command for the 0.2175 m row and the sweep flags (scale,
  base-from-camera extrinsics, ORB settings) are recorded verbatim in
  `docs/benchmarks/aqua_slam_error_budget.md`.
- Jazzy replay note: `ros2 bag play` uses `--playback-duration`, not
  `--duration`.

### Wider strategy context (why this first)

Intelligence gathered 2026-06-10 on AQUA-SLAM (T-RO 2025, Xu/Zhang/Wang):

- It does **not** use pressure; depth/z is a fair axis to win
  (`aqua_slam_error_budget.py` should grow a per-axis breakdown).
- Its README admits random multithread crashes on long sequences; its paper
  loses to Basalt on Structure Easy SLAM mode (0.06 vs 0.04 m).
- FAR-AVIO (arXiv:2512.20355) already beats it on hard Tank sequences with
  a Schur-complement **EKF** (~75% translation RMSE reduction on Structure
  Hard) — proof that a filter-based stack can win; tightly-coupled graph
  optimization is not the only path.
- Therefore the order of battle is: (1) fix this fusion regression,
  (2) kill the same-sequence scale fit via proper stereo calibration,
  (3) route visual keyframe edges + pressure unary + DVL/IMU odometry into
  the existing g2o pose graph with visual loop closure, (4) in parallel,
  run AQUA-SLAM on Medium/Hard/long sequences to measure crash rate and
  divergence for a robustness/completion-rate claim, (5) keep the MBES
  track as a separate claim where AQUA-SLAM cannot compete.

## Main Engineering Plan

### Phase 1 - Deterministic MBES replay

Goal: repeated baseline runs on the same MBES source window produce the same
upstream odometry, same pose-graph keyframe chain, and same accepted loop
endpoint pairs.

**Status: in progress — E2E not yet deterministic.**

Infrastructure done:

- strict recorder readiness: done;
- recorder cleanup on failure: done;
- install-prefix profile resolution + nested overlay preference: done;
- shallow replay queues (default depth 5 in MBES profiles): done;
- endpoint stamps in status rows: done;
- timestamp-buffered MBES submaps: done;
- sonar feedback diagnostic override (`IMU_SONAR_ODOMETRY_TOPIC`): done;
- C++ stamp-ordered sonar feedback buffer + status counters: done;
- unit-test repeatability for buffer logic: done;
- `compare_mbes_repeat_probe.py`: done;
- `acquire_mbes_beach_pond.py` + local `beach_pond_ros2` mcap: done;
- first full E2E repeat probe executed: done (shows failure);
- tightened-replay isolation probe: run1 done, run2 interrupted.

Still open (Phase 1):

- Achieve matching `input_odometry.tum` SHA256 and `sonar_feedback_*` counters
  across two consecutive runs with **identical** env (same `ROS_DOMAIN_ID`,
  same play rate/delay/ack settings).
- Decide whether delayed sonar observations beyond the buffer horizon should
  carry an explicit status reason beyond `skipped_stale` (currently 0 in probes).
- Add bag-side export for `EstimatorStatus` and keyframe endpoint stamp sets
  (compare script exists for directories; bag export still manual).
- Extend compare script to pose-graph keyframe chain and optimization count.
- Keep `IMU_SONAR_ODOMETRY_TOPIC=` as a diagnostic only, not a default.
- ~~Implement measurement-time state lookup for sonar (and visual) position
  updates.~~ done, **including the retroactive history correction
  (`shift_from`)** that the first implementation was missing — delayed
  feedback no longer re-applies absorbed corrections (was a positive feedback
  that destroyed trajectories at `max_age_s=1.0`). Re-run the MBES repeat
  probe with the shift-fixed node before further timing work: the same flaw
  applied to sonar feedback and plausibly amplified replay-order sensitivity
  into kilometer-scale divergence.

Definition of done:

- two `WEIGHTS=0,0` baseline runs on `beach_pond` have:
  - identical `input_odometry.tum` hash (or documented float tolerance with cause);
  - matching `sonar_feedback_received/applied` and `update_count` tails;
  - stable accepted endpoint pairs when loops exist;
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

### MBES determinism (Jazzy + nested workspace, current default dev env)

```bash
cd aqua_loc_ws/aqua_localization
export AQUA_LOC_WS=aqua_loc_ws
source scripts/setup_nested_workspace_env.sh

# Rebuild when C++ or scripts change
colcon build --packages-select aqua_sonar_loc aqua_imu_loc aqua_pose_graph aqua_localization

pytest -q aqua_localization/test/test_run_mbes_loop_benchmark.py \
  aqua_localization/test/test_record_mbes_demo.py
```

**Single benchmark run** (determinism-tuned env):

```bash
OUT_DIR=/tmp/aqua_mbes_repeat_det/weight_0_run2 \
MBES_OUT=/tmp/aqua_mbes_repeat_det/bags/mbes_weight_0_run2 \
WORKSPACE="$PWD" \
LOCAL_SETUP=install/setup.bash \
ROS_SETUP=/opt/ros/jazzy/setup.bash \
MBES_SRC="$PWD/datasets/public/mbes_slam/beach_pond_ros2" \
MBES_DURATION=120 \
ROS_DOMAIN_ID=130 \
PLAY_RATE=0.5 \
PLAY_READ_AHEAD_QUEUE_SIZE=10000 \
PLAY_START_DELAY_S=35 \
PLAY_TIMEOUT_MARGIN_S=15 \
PLAY_WAIT_FOR_ALL_ACKED_MS=5000 \
RECORD_READY_TIMEOUT_S=90 \
NODE_STARTUP_DELAY_S=10 \
POST_PLAY_SLEEP_S=5 \
IMU_QOS_SENSOR_DEPTH=5 \
SONAR_QOS_SENSOR_DEPTH=5 \
MBES_LOOP_CANDIDATE_DESCRIPTOR_WEIGHT=0.0 \
bash aqua_localization/scripts/run_mbes_loop_benchmark.sh
```

**Compare two probe directories** (after symlinking `input_odometry.tum`,
`mbes_loop_status.csv`, and `estimator_status.csv` into each run dir):

```bash
python3 aqua_localization/scripts/compare_mbes_repeat_probe.py \
  /tmp/aqua_mbes_repeat_det/weight_0_run1 \
  /tmp/aqua_mbes_repeat_det/weight_0_run2 \
  --out /tmp/aqua_mbes_repeat_det/mbes_repeat_probe_compare.md
```

**Two-run sweep** (uses incrementing `ROS_DOMAIN_ID`; prefer identical domain
for strict repeatability — run the single benchmark command twice instead):

```bash
OUT_ROOT=/tmp/aqua_mbes_repeat_next \
WEIGHTS=0,0 \
MBES_DURATION=120 \
SWEEP_ROS_DOMAIN_ID_START=130 \
WORKSPACE="$PWD" \
LOCAL_SETUP=install/setup.bash \
ROS_SETUP=/opt/ros/jazzy/setup.bash \
MBES_SRC="$PWD/datasets/public/mbes_slam/beach_pond_ros2" \
PLAY_RATE=0.5 \
PLAY_READ_AHEAD_QUEUE_SIZE=10000 \
PLAY_START_DELAY_S=35 \
PLAY_TIMEOUT_MARGIN_S=15 \
PLAY_WAIT_FOR_ALL_ACKED_MS=5000 \
RECORD_READY_TIMEOUT_S=90 \
NODE_STARTUP_DELAY_S=10 \
bash aqua_localization/scripts/run_mbes_candidate_descriptor_weight_sweep.sh
```

Do not set `IMU_SONAR_ODOMETRY_TOPIC=` for normal performance runs. Use it only
to isolate the sonar feedback path.

### Humble isolated install (optional, when validating Humble-specific bag metadata)

```bash
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/aqua_mbes_det_log build \
  --packages-up-to aqua_sonar_loc aqua_imu_loc aqua_pose_graph aqua_localization \
  --build-base /tmp/aqua_mbes_det_build \
  --install-base /tmp/aqua_mbes_det_install
source /tmp/aqua_mbes_det_install/setup.bash
```

### Tank held-out sequences (blocked on data delivery)

Official Google Form submitted for `Structure_Easy.bag` and `Medium.bag`.
When email arrives:

```bash
python3 aqua_localization/scripts/acquire_tank_heldout_sequences.py --convert-all
python3 aqua_localization/scripts/run_tank_sequence_heldout_calibration.py ...
```

Until then, held-out Tank calibration and Medium/Hard benchmark rows remain
blocked.

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
- Add a short "How to reproduce the MBES determinism probe" doc (commands now
  live in this PLAN.md "Recommended Next Command Sequence"; still needs a
  dedicated `docs/` page with artifact paths and expected pass/fail criteria).
- Publish a 3DGS training smoke result or explicitly document why the small
  pack is only an input artifact.

## Contribution-Sized Issues To Open

Good first technical issues:

- ~~Add a script that compares accepted endpoint pairs between two MBES status
  CSVs and reports overlap.~~ done (`compare_mbes_repeat_probe.py`).
- Add `export_estimator_status.py` (rosbag2 → CSV, mirrors `record_status.py` columns).
- Add a script that compares `/aqua_pose_graph/keyframe` endpoint timestamp
  sets between two recorded bags.
- ~~Add `aqua_imu_loc` status counters for sonar feedback accepted/skipped.~~ done.
- Add markdown generation for the open-loop sonar-feedback diagnostic table.
- ~~Add a small synthetic test for out-of-order sonar feedback observations.~~ done (`test_feedback_odometry_buffer`).
- Install `compare_mbes_repeat_probe.py` via CMake so `ros2 run` works.
- Install `mbes_loop_closure.yaml` via `aqua_sonar_loc` CMake install rules.

Medium issues:

- ~~Implement timestamp-ordered feedback buffering in `aqua_imu_loc`.~~ done (C++ buffer; E2E still flaky).
- Finish E2E determinism: node-ready waits + measurement-time state history for sonar/visual position updates.
- Enforce batch consistency as a loop acceptance gate.
- Generate SOTA-readiness reports from benchmark artifact directories.
- Add a held-out Tank Dataset benchmark row (waiting on Structure_Easy / Medium bags).
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

The 2026-06-11 afternoon session discovered that everything this document
called "in-tree" (shift_from lag compensation, sonar stamp buffer, status
counters, probe tooling, recorder hardening) had **never been committed** and
was lost with the unmounted `/media/sasaki/aiueo` workspace — only PLAN.md
survived — and then reconstructed the core from this document's design notes
on the new canonical checkout (`/home/sasaki/workspace/aqua_loc_ws/
aqua_localization`, Jazzy): commit `4cea97a` rebuilds `PositionHistoryBuffer`
+ `shift_from`, `snap_first_update`, the variance caps, the
`buffer_by_stamp` sonar feedback path, the five `EstimatorStatus` counters,
the tank config, and 9/9 passing tests, and the follow-up commit rebuilds
`record_status.py` counters and `compare_mbes_repeat_probe.py` with the
windowed inter-run agreement gate (smoke-tested). Next session, in order:
(1) finish the lost-tooling list in "Still lost — not yet reconstructed"
(probe pytest file, recorder hardening v2, benchmark same-run visual gate,
acquisition scripts); (2) re-acquire Tank `short_test` and `beach_pond_ros2`
(datasets/ is empty here) and re-run the Tank visual fusion benchmark to
re-validate the historical 0.2140 m row — until that lands, every measured
number in this file is historical and non-claimable; (3) only then resume
the pre-incident agenda: DVL x-axis bias (held-out-transferable), MBES
in-flight divergence instrumentation (keep depth 5), held-out Tank bags
still awaited by email. Commit after every validated step — "in-tree" is
not a state.

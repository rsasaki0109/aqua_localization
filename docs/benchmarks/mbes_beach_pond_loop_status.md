# MBES-SLAM `beach_pond` Loop-Status Benchmark

This page is the real-bag artifact target for the experimental MBES loop-closure
frontend. It is intentionally separate from the AQUA-SLAM Tank Dataset
head-to-head: `beach_pond` is where `aqua_localization` should prove MBES
registration, loop diagnostics, and false-positive control.

## Current Status

Status: `tuning measured, false-positive audit pending`

The first real `beach_pond` loop-status replay now reaches registration and
accepted loop candidates when the MBES submap density is tuned for this bag. The
default loop-closure profile is still conservative: with `submaps.min_points=300`
and `submaps.voxel_leaf_m=0.5`, the 120 s Humble/sqlite replay mostly reports
`too few downsampled points`. The measured tuning run below uses
`submaps.min_points=120` and `submaps.voxel_leaf_m=0.25`, which produces finite
registration statistics and accepted loop candidates.

Do not treat the accepted loops as validated accuracy evidence yet. The next
step is a visual false-positive audit in RViz/rerun and a longer replay window.
The generated audit priority list for the current run is
[`mbes_beach_pond_loop_audit.md`](mbes_beach_pond_loop_audit.md).

## Reproducible Run

First acquire and convert the source bag using
[`datasets/mbes_slam_beach_pond_acquisition.md`](../../datasets/mbes_slam_beach_pond_acquisition.md).

Run the full artifact pipeline:

```bash
WORKSPACE=$PWD \
MBES_SRC=$PWD/datasets/public/mbes_slam/beach_pond_ros2 \
MBES_OUT=/tmp/aqua_mbes_beach_pond_with_loop_status \
OUT_DIR=/tmp/aqua_mbes_loop_benchmark \
MBES_DURATION=120 \
ros2 run aqua_localization run_mbes_loop_benchmark.sh
```

The runner executes the readiness check, records a results-included replay bag,
exports `/mbes_loop_closure/status`, and writes the benchmark row.

For a ROS 2 Humble workspace converted to sqlite3, use the explicit storage and
tuning overrides used for the measured row:

```bash
WORKSPACE=$PWD \
ROS_SETUP=/opt/ros/humble/setup.bash \
RECORD_STORAGE=sqlite3 \
RECORD_TOPIC_FLAG= \
PLAY_DURATION_ARG= \
MBES_SRC=$PWD/datasets/public/mbes_slam/beach_pond_ros2_sqlite \
MBES_OUT=/tmp/aqua_mbes_beach_pond_tuned_120 \
OUT_DIR=/tmp/aqua_mbes_loop_benchmark_tuned_120 \
MBES_DURATION=120 \
MBES_LOOP_MIN_POINTS=120 \
MBES_LOOP_VOXEL_LEAF_M=0.25 \
NOTE="real replay, duration 120s, min_points=120, voxel=0.25, Humble sqlite" \
ros2 run aqua_localization run_mbes_loop_benchmark.sh
```

To run the steps manually, first check that the local `beach_pond` bag has the
required MBES, reference odometry, and IMU topics:

```bash
ros2 run aqua_localization check_mbes_benchmark_ready.py \
  --bag datasets/public/mbes_slam/beach_pond_ros2 \
  --out /tmp/mbes_beach_pond_readiness.md \
  --min-duration-s 60
```

Record a results-included replay bag with loop diagnostics:

```bash
WORKSPACE=$PWD \
MBES_SRC=$PWD/datasets/public/mbes_slam/beach_pond_ros2 \
MBES_OUT=/tmp/aqua_mbes_beach_pond_with_loop_status \
MBES_DURATION=120 \
./aqua_localization/scripts/record_mbes_demo.sh
```

Export the status stream:

```bash
ros2 run aqua_localization export_mbes_loop_status.py \
  --bag /tmp/aqua_mbes_beach_pond_with_loop_status \
  --out /tmp/mbes_beach_pond_loop_status.csv \
  --summary-out /tmp/mbes_beach_pond_loop_status.md \
  --descriptor-sweep-out /tmp/mbes_beach_pond_descriptor_sweep.md
```

Generate the benchmark table row from the exported CSV:

```bash
ros2 run aqua_localization mbes_loop_benchmark_row.py \
  --csv /tmp/mbes_beach_pond_loop_status.csv \
  --dataset MBES-SLAM \
  --sequence beach_pond \
  --duration 120 \
  --note "first real replay"
```

Expected generated files:

| Artifact | Purpose |
|----------|---------|
| `/tmp/mbes_beach_pond_readiness.md` | Preflight report for required topics, message counts, and bag duration. |
| `/tmp/aqua_mbes_beach_pond_with_loop_status` | Results-included replay bag with MBES loop diagnostics. |
| `/tmp/mbes_beach_pond_loop_status.csv` | Raw `/mbes_loop_closure/status` samples for every tested candidate. |
| `/tmp/mbes_beach_pond_loop_status.md` | Accepted/rejected/no-candidate counts, status reasons, fitness, correction, and descriptor quantiles. |
| `/tmp/mbes_beach_pond_descriptor_sweep.md` | Candidate descriptor threshold grid for pre-registration gating. |
| `mbes_loop_benchmark_row.py` output | One Markdown row for the measurement table below. |
| `audit_mbes_loop_candidates.py` output | Accepted-loop visual audit priority list. |

## Measurement Table

Fill this table only from exported summaries. Rows marked `unaudited` must not
be used as loop-closure accuracy claims until the false-positive audit below is
completed.

| Dataset | Sequence | Duration s | Status samples | Accepted | Rejected | No candidate | Converged | Median fitness | P95 correction m | Notes |
|---------|----------|-----------:|---------------:|---------:|---------:|-------------:|----------:|---------------:|-----------------:|-------|
| MBES-SLAM | `beach_pond` | 120 | 277 | 35 | 178 | 64 | 163 | 0.1930 | 3.7891 | unaudited tuning run, `min_points=120`, `voxel=0.25`, Humble sqlite |

## Tuning Summary

The tuned 120 s run produced 213 registration attempts. The accepted loops had
median fitness `0.110679` and P95 accepted fitness `0.891069`. The main rejection
reasons were:

| Reason | Count |
|--------|------:|
| duplicate loop suppressed | 103 |
| no candidate submaps | 64 |
| registration did not converge | 50 |
| fitness score exceeds gate | 21 |
| rotation correction exceeds gate | 2 |
| translation correction exceeds gate | 2 |

The descriptor sweep suggests useful first-pass descriptor gates around
`centroid <= 1.29 m`, `extent ratio <= 5.69`, and
`point-count ratio >= 0.42` if the next replay needs to reduce registration
load without removing most plausible candidates. Keep descriptor gates disabled
until the accepted-loop audit is complete.

## False-Positive Audit

Before marking this case `measured` in
[`real_bag_evaluation_manifest.json`](real_bag_evaluation_manifest.json), audit
accepted loop candidates against the RViz markers or rerun overlay.

| Check | Required evidence | Result |
|-------|-------------------|--------|
| Accepted edge geometry | Accepted marker connects visually plausible revisits, not adjacent duplicate submaps. | Pending for the 35 accepted loops in the tuned 120 s run. |
| Pose-graph effect | `/aqua_pose_graph/path` changes in the expected direction after loop insertion. | TBD |
| Registration gate | Accepted candidates have finite fitness and correction below the configured gate. | PASS for exported status: accepted fitness P95 `0.891069`, configured max `2.0`. |
| Descriptor gate | Descriptor sweep keeps enough plausible candidates while reducing obvious misses. | Candidate starting point: centroid `1.29 m`, extent ratio `5.69`, point-count ratio `0.42`; not enabled yet. |
| Duplicate suppression | Near-repeat accepted loops are suppressed by keyframe gap / cooldown settings. | PASS mechanically: 103 `duplicate loop suppressed` statuses. Visual audit still pending. |

## Promotion Rule

Promote `mbes-beach-pond-loop-status` from `scaffolded` to `measured` only when:

- the exported summary has nonzero status samples,
- the readiness report passes for the source bag,
- the measurement table above is filled from the generated summary,
- every accepted loop has a false-positive audit note,
- the descriptor sweep is linked or copied into this benchmark folder, and
- the replay duration, bag path, config files, and commit hash are recorded.

This keeps the MBES story strong without overstating the current loop-closure
reliability.

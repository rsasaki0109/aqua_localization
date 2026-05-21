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
The stricter keyframe-separation run is tracked in
[`mbes_beach_pond_loop_audit_gap40.md`](mbes_beach_pond_loop_audit_gap40.md).
The stricter rotation/descriptor gate run is tracked in
[`mbes_beach_pond_loop_audit_gap40_gate.md`](mbes_beach_pond_loop_audit_gap40_gate.md).
Its accepted-loop keyframe geometry worksheet is
[`mbes_beach_pond_loop_geometry_gap40_gate.md`](mbes_beach_pond_loop_geometry_gap40_gate.md).

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

To suppress near-neighbor accepted loops during audit, add:

```bash
MBES_LOOP_MIN_KEYFRAME_SEPARATION=40
```

To also suppress high-rotation and large-extent accepted candidates, add:

```bash
MBES_LOOP_MAX_CORRECTION_ROTATION_RAD=0.4 \
MBES_LOOP_DESCRIPTOR_MAX_EXTENT_RATIO=5.0
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
| `plot_mbes_loop_audit.py` output | Plan-view PNG of the pose graph and accepted-loop audit priorities. |
| `audit_mbes_loop_geometry.py` output | Accepted-loop keyframe geometry worksheet for RViz/rerun review. |

Publish the accepted-loop audit overlay in RViz:

```bash
ros2 run aqua_localization publish_mbes_loop_audit_markers.py \
  --bag /tmp/aqua_mbes_beach_pond_tuned_120 \
  --csv /tmp/aqua_mbes_loop_benchmark_tuned_120/mbes_beach_pond_loop_status.csv
```

Add `/mbes_loop_audit/markers` as a `MarkerArray` display beside the normal
`/mbes_loop_closure/markers` topic. Red edges are the highest-priority accepted
loops to inspect first; labels show rank, priority, keyframe IDs, fitness, and
translation correction.

For a quick non-interactive audit preview, generate the plan-view PNG:

```bash
ros2 run aqua_localization plot_mbes_loop_audit.py \
  --bag /tmp/aqua_mbes_beach_pond_tuned_120 \
  --csv /tmp/aqua_mbes_loop_benchmark_tuned_120/mbes_beach_pond_loop_status.csv \
  --out docs/media/mbes_beach_pond_loop_audit.png
```

## Measurement Table

Fill this table only from exported summaries. Rows marked `unaudited` must not
be used as loop-closure accuracy claims until the false-positive audit below is
completed.

| Dataset | Sequence | Duration s | Status samples | Accepted | Rejected | No candidate | Converged | Median fitness | P95 correction m | Notes |
|---------|----------|-----------:|---------------:|---------:|---------:|-------------:|----------:|---------------:|-----------------:|-------|
| MBES-SLAM | `beach_pond` | 120 | 277 | 35 | 178 | 64 | 163 | 0.1930 | 3.7891 | unaudited tuning run, `min_points=120`, `voxel=0.25`, Humble sqlite |
| MBES-SLAM | `beach_pond` | 120 | 338 | 35 | 194 | 109 | 133 | 0.9520 | 2.9900 | unaudited stricter candidate run, `min_points=120`, `voxel=0.25`, `min_keyframe_separation=40`, Humble sqlite |
| MBES-SLAM | `beach_pond` | 120 | 462 | 17 | 261 | 184 | 110 | 25.4749 | 4.7003 | unaudited strict gate run, `min_points=120`, `voxel=0.25`, `min_keyframe_separation=40`, `max_rotation=0.4`, `descriptor_extent=5.0`, Humble sqlite |

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
`point-count ratio >= 0.42` if a replay needs to reduce registration load
without removing most plausible candidates.

### Candidate-Separation Sweep

The first false-positive-control sweep raised
`candidates.min_keyframe_separation` from `20` to `40` while keeping
`submaps.min_points=120` and `submaps.voxel_leaf_m=0.25`. This keeps accepted
loop count unchanged but removes short-gap accepted loops and improves accepted
loop fitness/correction tails.

| Setting | Accepted | Short-gap accepted <= 40 | Accepted fitness median | Accepted fitness P95 | Accepted correction P95 m |
|---------|---------:|-------------------------:|------------------------:|---------------------:|--------------------------:|
| separation 20 | 35 | 11 | 0.1107 | 0.8911 | 3.3794 |
| separation 40 | 35 | 0 | 0.0506 | 0.4520 | 2.4376 |

This does not validate the loops by itself; it only removes a clear near-neighbor
risk class before RViz/rerun inspection.

### Rotation And Descriptor Gate Sweep

The next sweep kept `min_keyframe_separation=40` and added
`gates.max_correction_rotation_rad=0.4` plus
`descriptor.max_extent_ratio=5.0`. It removes the obvious rotation-near-gate and
large-extent accepted candidates, but it is aggressive: accepted loops drop from
35 to 17 and no-candidate statuses rise from 109 to 184.

| Setting | Accepted | High-risk accepted | Accepted fitness median | Accepted fitness P95 | Accepted correction P95 m | Accepted rotation P95 rad |
|---------|---------:|-------------------:|------------------------:|---------------------:|--------------------------:|--------------------------:|
| separation 40 | 35 | 5 | 0.0506 | 0.4520 | 2.4376 | 0.4259 |
| separation 40 + rotation/extent gates | 17 | 2 | 0.0198 | 0.8012 | 4.1239 | 0.3793 |

Use the strict gate run as an audit candidate set, not as the default profile
yet. The two remaining high-priority accepted loops are translation-near-gate
cases (`396 -> 1108` and `48 -> 709`) and need visual inspection before
tightening the translation gate.

## False-Positive Audit

Before marking this case `measured` in
[`real_bag_evaluation_manifest.json`](real_bag_evaluation_manifest.json), audit
accepted loop candidates against the RViz markers or rerun overlay.

| Check | Required evidence | Result |
|-------|-------------------|--------|
| Accepted edge geometry | Accepted marker connects visually plausible revisits, not adjacent duplicate submaps. | In progress; the strict gate geometry worksheet resolves keyframe positions for 16/17 accepted loops. One high-priority row (`1105 -> 1231`) is missing keyframe geometry and must be checked in the replay bag before promotion. |
| Pose-graph effect | `/aqua_pose_graph/path` changes in the expected direction after loop insertion. | TBD |
| Registration gate | Accepted candidates have finite fitness and correction below the configured gate. | PASS mechanically for the strict gate row, but the two translation-near-gate accepted loops still need visual audit. |
| Descriptor gate | Descriptor sweep keeps enough plausible candidates while reducing obvious misses. | `descriptor.max_extent_ratio=5.0` is active in the strict gate row; centroid and point-count gates remain disabled. |
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

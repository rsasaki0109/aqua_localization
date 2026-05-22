# MBES-SLAM `beach_pond` Loop-Status Benchmark

This page is the real-bag artifact target for the experimental MBES loop-closure
frontend. It is intentionally separate from the AQUA-SLAM Tank Dataset
head-to-head: `beach_pond` is where `aqua_localization` should prove MBES
registration, loop diagnostics, and false-positive control.

## Current Status

Status: `scaffolded`

The repository has the replay command, RViz/rerun visualization paths, loop
status exporter, and descriptor sweep report shape. The local workspace does not
currently include the MBES-SLAM `beach_pond` rosbag, so this page does not claim
measured loop counts yet.

## Reproducible Run

Check that the local `beach_pond` bag has the required MBES, reference odometry,
and IMU topics before launching a benchmark run:

```bash
ros2 run aqua_localization check_mbes_benchmark_ready.py \
  --bag /path/to/beach_pond_ros2 \
  --out /tmp/mbes_beach_pond_readiness.md \
  --min-duration-s 60
```

Record a results-included replay bag with loop diagnostics:

```bash
MBES_SRC=/path/to/beach_pond_ros2 \
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
| `/tmp/mbes_beach_pond_loop_status.csv` | Raw `/mbes_loop_closure/status` samples for every tested candidate. |
| `/tmp/mbes_beach_pond_loop_status.md` | Accepted/rejected/no-candidate counts, status reasons, fitness, correction, and descriptor quantiles. |
| `/tmp/mbes_beach_pond_descriptor_sweep.md` | Candidate descriptor threshold grid for pre-registration gating. |
| `mbes_loop_benchmark_row.py` output | One Markdown row for the measurement table below. |

## Measurement Table

Fill this table only from the exported summary. Leave cells as `TBD` until the
same replay/export command above has been run.

| Dataset | Sequence | Duration s | Status samples | Accepted | Rejected | No candidate | Converged | Median fitness | P95 correction m | Notes |
|---------|----------|-----------:|---------------:|---------:|---------:|-------------:|----------:|---------------:|-----------------:|-------|
| MBES-SLAM | `beach_pond` | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | awaiting local real-bag replay |

## False-Positive Audit

Before marking this case `measured` in
[`real_bag_evaluation_manifest.json`](real_bag_evaluation_manifest.json), audit
accepted loop candidates against the RViz markers or rerun overlay.

| Check | Required evidence | Result |
|-------|-------------------|--------|
| Accepted edge geometry | Accepted marker connects visually plausible revisits, not adjacent duplicate submaps. | TBD |
| Pose-graph effect | `/aqua_pose_graph/path` changes in the expected direction after loop insertion. | TBD |
| Registration gate | Accepted candidates have finite fitness and correction below the configured gate. | TBD |
| Descriptor gate | Descriptor sweep keeps enough plausible candidates while reducing obvious misses. | TBD |
| Duplicate suppression | Near-repeat accepted loops are suppressed by keyframe gap / cooldown settings. | TBD |

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

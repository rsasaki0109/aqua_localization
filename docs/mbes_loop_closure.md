# MBES Loop Closure Front-End Plan

The pose graph backend can now accept external loop closures on
`/aqua_pose_graph/loop_constraint`. This document pins the intended MBES
front end so the next implementation can stay small and measurable.

## Current Building Blocks

- `aqua_pose_graph` builds a g2o SE(3) keyframe chain from upstream odometry.
- `aqua_msgs/PoseGraphKeyframe` exposes the keyframe ID and pose assigned by
  the graph backend.
- `aqua_msgs/PoseGraphLoopConstraint` carries a relative SE(3) constraint
  between two existing keyframes.
- `aqua_sonar_loc` already has PCL ICP/GICP/NDT backends and MBES-specific
  quality gates.
- `aqua_sonar_loc/mbes_loop_closure_node` is the first experimental MBES
  front end. It accumulates short submaps between pose-graph keyframes,
  searches older odometry-near submaps, registers them with ICP/GICP/NDT, and
  publishes accepted constraints to `/aqua_pose_graph/loop_constraint`.
- The same node publishes `aqua_msgs/LoopClosureStatus` on
  `/mbes_loop_closure/status` for each tested candidate, including rejection
  reason, convergence, fitness score, and correction magnitude.
- Candidate loop edges are also published as `visualization_msgs/MarkerArray`
  on `/mbes_loop_closure/markers`, with accepted edges drawn brighter than
  rejected edges for RViz tuning sessions.
- `aqua_localization/scripts/pose_graph_loop_demo.py` publishes a synthetic
  odometry chain plus one loop constraint for smoke-testing the graph input.

## Target MBES Pipeline

1. Subscribe to accepted MBES fans and pose-graph keyframes.
2. Accumulate a local bathymetric submap for each keyframe interval.
3. Keep a searchable index of older submaps.
4. Reject candidates inside a temporal/keyframe exclusion window.
5. Run submap-vs-submap GICP, ICP, or NDT with the odometry relative
   transform as the initial guess.
6. Gate on convergence, fitness, correction magnitude, and transform
   consistency.
7. Publish `aqua_msgs/PoseGraphLoopConstraint` with a conservative
   information matrix.
8. Publish `aqua_msgs/LoopClosureStatus` so tuning can distinguish "no
   candidates" from rejected or accepted registration results.
9. Re-export the rerun.io demo with the pose-graph path, accepted loop edges,
   and loop-closure status plots.

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

The same smoke path can be run as a single command:

```bash
ros2 run aqua_localization pose_graph_loop_smoke.sh
```

## First Real-Data Target

Use MBES-SLAM `beach_pond` because the repository already has:

- dataset conversion notes,
- GICP config,
- rerun export,
- committed screenshots and GIFs.

The first real-data milestone does not need to find every loop closure. A
single accepted submap-vs-submap loop that visibly changes the optimized
path is enough for the next README-worthy demo.

Launch shape:

```bash
ros2 launch aqua_localization replay.launch.py \
  start_bag:=true \
  bag_path:=aqua_localization/datasets/public/mbes_slam/beach_pond_ros2 \
  bag_sonar_points_topic:=/norbit/detections \
  use_sim_time:=true \
  enable_imu_loc:=true \
  enable_sonar_loc:=true \
  enable_pose_graph:=true \
  enable_mbes_loop_closure:=true \
  imu_params_file:=$(ros2 pkg prefix aqua_imu_loc)/share/aqua_imu_loc/config/mbes_slam.yaml \
  sonar_params_file:=$(ros2 pkg prefix aqua_sonar_loc)/share/aqua_sonar_loc/config/mbes_slam.yaml \
  mbes_loop_closure_params_file:=$(ros2 pkg prefix aqua_sonar_loc)/share/aqua_sonar_loc/config/mbes_loop_closure.yaml
```

Tune in this order:

1. `submaps.voxel_leaf_m` and `submaps.min_points` until submaps are dense
   enough but not too slow.
2. `candidates.max_distance_m` and `candidates.min_keyframe_separation` until
   plausible revisits are tested.
3. `candidates.descriptor_weight` with
   `candidates.descriptor_centroid_scale_m`,
   `candidates.descriptor_extent_scale`, and
   `candidates.descriptor_point_count_ratio_scale` when
   `candidates.max_per_keyframe` is too small to test every plausible revisit.
   This only moves shape-similar bathymetric submaps earlier in the
   pre-registration queue; it does not accept or reject loops by itself.
4. `descriptor.max_centroid_distance_m`, `descriptor.max_extent_ratio`, and
   `descriptor.min_point_count_ratio` after collecting descriptor distributions
   from a replay. Leave these disabled until real-bag ranges are understood.
5. `gates.max_fitness_score`, `gates.max_correction_translation_m`, and
   `gates.max_correction_rotation_rad` until false positives are rejected.
6. `gates.min_plan_view_separation_m` with
   `gates.max_short_plan_view_rotation_rad` when accepted-loop geometry shows
   nearly co-located plan-view endpoints getting large rotation corrections.
   Both values are disabled by default; when both are positive, a candidate is
   rejected as `short plan-view rotation gate rejected` if its odometry-derived
   plan-view separation is below the configured distance and its registration
   correction rotation exceeds the configured short-edge rotation cap.
7. `loop.min_repeat_keyframe_gap` to suppress near-duplicate accepted loops
   while preserving distinct revisits.
8. `loop.consistency.max_correction_translation_delta_m`,
   `loop.consistency.max_correction_rotation_delta_rad`, and optionally
   `loop.consistency.min_support_count` after trusted accepted loops exist.
   Positive delta values reject new accepted-looking candidates whose
   odometry-to-loop correction lacks enough support from previously accepted
   loop corrections.
8. `loop.translation_sigma_m` and `loop.rotation_sigma_rad` after comparing
   optimized path changes against the MBES-SLAM reference odometry.

Export the loop-status stream after a replay to make tuning measurable:

```bash
ros2 run aqua_localization export_mbes_loop_status.py \
  --bag aqua_localization/datasets/public/mbes_slam/demo_with_estimate \
  --out /tmp/mbes_loop_status.csv \
  --summary-out /tmp/mbes_loop_status.md \
  --descriptor-sweep-out /tmp/mbes_loop_descriptor_sweep.md \
  --consistency-sweep-out /tmp/mbes_loop_consistency_sweep.md \
  --consistency-rejection-audit-out /tmp/mbes_loop_consistency_rejections.md \
  --batch-consistency-out /tmp/mbes_loop_batch_consistency.md \
  --batch-consistency-selected-csv-out /tmp/mbes_loop_batch_selected_loops.csv \
  --consistency-min-support-count 1
```

Export trajectory evidence from the same results-included replay:

```bash
ros2 run aqua_localization mbes_loop_trajectory_metrics.py \
  --bag aqua_localization/datasets/public/mbes_slam/demo_with_estimate \
  --out /tmp/mbes_loop_trajectory_metrics.md \
  --out-dir /tmp/mbes_loop_trajectory_metrics
```

The CSV preserves every `/mbes_loop_closure/status` sample. The markdown
summary reports accepted, rejected, and no-candidate counts, rejection
reasons, fitness quantiles, correction translation/rotation quantiles, and
descriptor centroid-distance, extent-ratio, point-count-ratio, and consistency
nearest-delta quantiles.
When `/aqua_pose_graph/optimization_count` and
`/aqua_pose_graph/optimization_chi2` are present in the recorded bag, the
summary also reports how many g2o optimization runs actually happened and the
latest active chi-square. This helps confirm that loop constraints, not the
odometry-only chain, are driving pose-graph optimization work.
Pass the same markdown file to `mbes_loop_benchmark_row.py --summary` to carry
those optimization diagnostics into the benchmark table row without copying
the values by hand.
The trajectory metrics report exports `/nav/processed/odometry`,
`/aqua_imu_loc/odometry`, and the latest `/aqua_pose_graph/path` as TUM files,
then compares input odometry vs. pose graph APE against the dataset reference.
Use it after normal, consistency-gated, and selected-loop replays to measure
whether loop closures improve trajectory error rather than only changing loop
counts.
Descriptor fields are still exported when descriptor thresholds are disabled,
so replay summaries can be used to choose initial threshold values before
turning the gate on. The descriptor sweep report evaluates percentile-derived
threshold grids and reports how many tested candidates would pass each
combination. The consistency sweep report uses recorded correction poses when
available, with scalar correction-magnitude fallback for older bags, to mirror
the accepted-loop consistency guard and propose initial threshold values.
Candidate descriptor ranking is a weaker pre-registration retrieval knob. Set
`candidates.descriptor_weight` positive to combine normalized pose distance with
centroid, extent, and point-count descriptor mismatch before the
`candidates.max_per_keyframe` budget is applied. The matching scales default to
the same order used by descriptor-signature replay probes (`0.6 m`, `0.15`, and
`0.10`). Keep the weight at `0.0` for baseline runs; increase it only in paired
normal/selected comparisons where `mbes_descriptor_retrieval.md` shows that the
desired endpoint appears in the replay candidate pool but is ranked too low.
Use the descriptor-weight sweep wrapper to keep those paired runs comparable:

```bash
OUT_ROOT=/tmp/aqua_mbes_candidate_descriptor_weight_sweep \
WEIGHTS=0.0,0.25,0.5,1.0,2.0 \
MBES_DURATION=120 \
MBES_PREPARE_HUMBLE_METADATA=1 \
MBES_HUMBLE_WINDOW_S=180 \
./aqua_localization/scripts/run_mbes_candidate_descriptor_weight_sweep.sh
```

Each weight gets a separate benchmark replay directory and recorded bag. The
wrapper writes `mbes_candidate_descriptor_weight_sweep.md` and
`mbes_candidate_descriptor_weight_sweep.csv`, comparing input RMSE, pose-graph
RMSE, accepted-loop counts, candidate rejections, and matched-time coverage
against the `0.0` baseline. Rows marked `check coverage` should be treated as
diagnostic until replay coverage is aligned. The wrapper assigns incrementing
`ROS_DOMAIN_ID` values from `SWEEP_ROS_DOMAIN_ID_START` so sequential cases do
not reuse stale transient-local graph publishers. Repeat a weight, such as
`WEIGHTS=0,0,1.0,2.0`, to estimate replay variance; duplicate outputs keep the
first directory name and then add `_run2`, `_run3`, and later suffixes.
The consistency rejection audit lists actual `loop consistency rejected`
samples by support deficit and nearest correction delta after replaying with
positive consistency thresholds. Inspect loop geometry before enabling positive
`loop.consistency.*` thresholds.
The batch consistency report builds a pairwise graph from accepted and
`loop consistency rejected` corrections, then selects one internally consistent
clique. Use it as the offline PCM-like handoff from tuning to replay planning:
selected loop IDs are candidates for a constrained replay, while batch-rejected
IDs need RViz/rerun review before they can support an accuracy claim.
The selected-loop CSV can be passed back into `mbes_loop_closure_node` through
`loop.selection.allowlist_csv` to replay with only the offline-selected loop
pairs.

### Reading the Descriptor Sweep

See [the descriptor sweep example](examples/mbes_loop_descriptor_sweep.md) for
the report shape and a worked interpretation flow. Treat the sweep as a
candidate budget tool: each row answers how many historical submaps would reach
registration if those descriptor thresholds were enabled. The strictest row is
not automatically best, because registration and correction gates still need
enough plausible candidates to verify.

Use one row as an initial descriptor config:

```yaml
descriptor:
  enabled: true
  max_centroid_distance_m: <Centroid <= m>
  max_extent_ratio: <Extent <= ratio>
  min_point_count_ratio: <Point count >= ratio>
```

After replaying with those values, export the status stream again and compare
`descriptor gate rejected` counts, registration failures, accepted markers in
RViz, and optimized path changes. Descriptor thresholds should reduce wasted or
implausible registration attempts; they should not be treated as calibrated
defaults until they have been checked on the target bag.

### Reading Short-Edge Rotation Gate Results

The accepted-loop geometry review reports the plan-view distance between each
accepted candidate/current keyframe pair. On the 2026-06-04 100 s
`beach_pond` diagnostic replays, many harmful selected loops had plan-view
edges below 1 m while asking for rotation corrections near the gate. The
short-edge guard rejected 15 such candidates, but the replay still worsened
trajectory RMSE. A stricter global `gates.max_correction_rotation_rad: 0.2`
then improved the selected replay pose-graph RMSE by 4.82 m versus its input in
a paired diagnostic run, while exact selected-loop IDs still drifted between
normal and selected replays. The runtime guard for the short-edge pattern is:

```yaml
gates:
  min_plan_view_separation_m: <Plan XY below m>
  max_short_plan_view_rotation_rad: <Rotation cap for short plan-view edges>
```

For wrapper-based experiments, pass the same values as:

```bash
MBES_LOOP_MIN_PLAN_VIEW_SEPARATION_M=<Plan XY below m>
MBES_LOOP_MAX_SHORT_PLAN_VIEW_ROTATION_RAD=<Rotation cap rad>
```

Treat this as a false-positive guard, not as proof that short plan-view loops
are invalid. A true revisit can be spatially close, so keep the gate disabled
until the geometry worksheet shows the specific short-edge/high-rotation
failure mode and trajectory metrics confirm that the cap reduces harm.

### Reading the Consistency Sweep

See [the consistency sweep example](examples/mbes_loop_consistency_sweep.md) for
the report shape. Each row estimates how many currently accepted loop
corrections would remain supported if the runtime consistency guard used those
translation and rotation delta thresholds. New replays use
`LoopClosureStatus.correction_pose` for the same SE(3) delta magnitude checked
by the runtime guard; old bags without that field fall back to scalar
correction magnitudes. The first accepted loop always bootstraps the guard, so
run the sweep only after visually auditing the earliest accepted loops. Newer
status CSVs also include `consistency_support_count`,
`consistency_required_support_count`,
`consistency_nearest_translation_delta_m`, and
`consistency_nearest_rotation_delta_rad`, so a rejected candidate shows both
how many prior loops supported it and how close the nearest prior correction
was.

Use one row as an initial consistency config:

```yaml
loop:
  consistency:
    max_correction_translation_delta_m: <Translation delta <= m>
    max_correction_rotation_delta_rad: <Rotation delta <= rad>
    min_support_count: 1
```

`min_support_count: 1` preserves the original any-support guard. Raising it to
`2` or more requires multiple previously accepted loops to agree once enough
accepted-loop history exists; the bootstrap requirement is clamped to the
available history size so the second trusted loop is not impossible to accept.
Replay with those values and confirm that `loop consistency rejected` samples
are the intended outlier loop corrections. A typical bad single-support replay
has `consistency_support_count` below `consistency_required_support_count`; if
the nearest deltas are just above threshold, loosen thresholds or keep
`min_support_count: 1` until more trusted loops are available. If accepted loops
split into several valid motion regimes, leave the guard disabled until a batch
consistency selector can reason over batches of candidate transforms.
Use `/tmp/mbes_loop_consistency_rejections.md` to sort those rejections by
support deficit before reviewing RViz/rerun markers.

### Reading the Batch Consistency Report

See [the batch consistency example](examples/mbes_loop_batch_consistency.md) for
the report shape and a worked interpretation flow.
`--batch-consistency-out` produces a PCM-like selection report over finite loop
corrections that were either accepted or rejected only by the runtime
consistency guard. It uses `LoopClosureStatus.correction_pose` when present,
falls back to scalar correction magnitudes for older bags, and connects two
loops when both translation and rotation correction deltas are below the
configured thresholds. If
`--batch-consistency-translation-threshold-m` or
`--batch-consistency-rotation-threshold-rad` is omitted, the exporter uses the
configured auto quantile from the candidate pairwise delta distribution.
Use `--batch-consistency-max-fitness-score` when the maximum clique prefers
geometrically consistent but weak registrations. This filters candidate loops
before clique search, and the benchmark wrapper forwards it as
`MBES_LOOP_BATCH_CONSISTENCY_MAX_FITNESS_SCORE`.

Example with explicit thresholds:

```bash
ros2 run aqua_localization export_mbes_loop_status.py \
  --bag /tmp/aqua_mbes_beach_pond_with_loop_status \
  --out /tmp/mbes_loop_status.csv \
  --batch-consistency-out /tmp/mbes_loop_batch_consistency.md \
  --batch-consistency-selected-csv-out /tmp/mbes_loop_batch_selected_loops.csv \
  --batch-consistency-translation-threshold-m 1.3 \
  --batch-consistency-rotation-threshold-rad 0.16 \
  --batch-consistency-max-fitness-score 0.2
```

The selected set is not a full SOTA claim. Treat it as a replay/audit input:
the next run should apply or inspect the selected loop IDs, compare pose-graph
APE/RPE against the dataset reference, and keep false-positive notes attached
to every accepted loop. To apply it during a replay, run the recorder with:

```bash
MBES_LOOP_SELECTION_ALLOWLIST_CSV=/tmp/mbes_loop_batch_selected_loops.csv \
./aqua_localization/scripts/record_mbes_demo.sh
```

When `loop.selection.allowlist_csv` is set, the node still requires descriptor,
registration, fitness, and correction gates to pass, but it replaces the online
accepted-loop consistency guard with the offline selected ID set. Otherwise
valid loops that are not in the CSV are published in status as
`loop selection rejected` and are not sent to the pose graph.
With `loop.selection.prioritize_candidates: true` (the default), candidates
that match exact allowlist IDs or endpoint timestamp signatures are tried
before other distance-ranked candidates. This keeps `candidates.max_per_keyframe`
from spending the replay budget on nearby off-allowlist endpoints before the
offline-selected endpoint has been registered. Set
`MBES_LOOP_SELECTION_PRIORITIZE_CANDIDATES=0` to reproduce the older
distance-only ordering during diagnostics.
The CSV currently matches exact `(candidate_id,current_id)` pairs from the
source replay. The exporter also writes `current_keyframe_timestamp` and
`candidate_keyframe_timestamp` columns when the bag contains
`/aqua_pose_graph/keyframe`, so later audits can distinguish loops that have a
similar status timestamp but different endpoint keyframes. A selected replay
with 0 accepted loops can still be a useful integrity probe when the allowlist
audit passes, but it is not trajectory evidence. Treat the selected-vs-normal
APE comparison as claimable only when the replay accepts audited selected loops
and the comparison report does not warn about baseline drift or mismatched
matched-time coverage.

If exact keyframe IDs drift between replays, the node can also use the status
export columns as an explicit signature fallback. This is disabled by default:
set `loop.selection.match_timestamp_window_s` positive, then optionally bound
`loop.selection.match_max_fitness_delta`,
`loop.selection.match_max_translation_delta_m`, and
`loop.selection.match_max_rotation_delta_rad`. Exact ID matches are still
accepted first; the signature path only decides otherwise accepted-looking
loops that missed the exact `(candidate_id,current_id)` pair. When endpoint
timestamps are present in the allowlist CSV, signature matching requires both
the current and candidate keyframe timestamps to fall inside the configured
window. This is stricter than the older status-timestamp-only fallback and can
turn a diagnostic PASS into a FAIL if the replay accepts a nearby but different
candidate endpoint.
Selected CSV rows also carry `descriptor_centroid_distance_m`,
`descriptor_extent_ratio`, and `descriptor_point_count_ratio`. Set
`loop.selection.match_max_descriptor_centroid_delta_m`,
`loop.selection.match_max_descriptor_extent_ratio_delta`, and
`loop.selection.match_max_descriptor_point_count_ratio_delta` to make signature
matching and allowlist candidate prioritization descriptor-refined. These
descriptor deltas are disabled by default and require a selected CSV exported
after the descriptor columns were added; exact ID matches still bypass the
signature fallback.
Use the matching `check_mbes_loop_allowlist_replay.py --signature-*` options
when auditing a signature replay; exact-ID-only strict audits are still useful
for integrity probes that should not accept drifted IDs.

The 2026-06-05 Humble sqlite diagnostic at
`/tmp/aqua_mbes_selected_loop_compare_priority_rot02` used
`submaps.min_points=120`, `submaps.voxel_leaf_m=0.25`,
`gates.max_correction_rotation_rad=0.2`, DCS delta `0.1`, endpoint signatures,
and allowlist-aware candidate prioritization. It accepted 1 selected loop,
matched it by endpoint signature with 0 off-allowlist accepted loops, and
improved selected pose-graph RMSE by 7.25 m versus normal replay. It is still
not claimable trajectory evidence because the selected pose graph was worse
than its own input odometry by 4.83 m and only one selected endpoint survived.
The next front-end step is descriptor-level loop identity/retrieval rather than
only broader timestamp tolerances.

A follow-up descriptor-refined signature replay at
`/tmp/aqua_mbes_selected_loop_compare_descriptor_sig_w5` widened the timestamp
window to 5 s and bounded descriptor deltas, but accepted 0 selected loops. The
audit still passed because no off-allowlist loops were accepted. The descriptor
retrieval report from the same bundle ranked 5 descriptor signatures against
the selected replay status rows; endpoint-timestamp recall was 0/5 through
top 5 and 1/5 at top 10. The useful finding is that offline selection can
prefer replay-brittle or weak loop registrations unless candidate quality is
filtered before clique selection, and that scalar descriptor similarity is not
yet a robust loop-proposal stage.

To run the normal replay, selected-loop replay, and metric comparison as one
artifact bundle:

```bash
OUT_ROOT=/tmp/aqua_mbes_selected_loop_comparison \
MBES_DURATION=120 \
./aqua_localization/scripts/run_mbes_selected_loop_replay_comparison.sh
```

The wrapper writes separate normal/selected benchmark directories,
`mbes_selected_loop_replay_comparison.md`, and
`mbes_selected_loop_allowlist_audit.md`. It also writes
`mbes_selected_loop_coverage.md` before the strict allowlist audit so failed
selected replays still explain whether selected IDs were replayed, missed by
timestamp coverage, or replaced by accepted-looking candidates with different
IDs. It writes `mbes_descriptor_retrieval.md`, which scores selected-loop
descriptor signatures against replay candidate rows and reports exact-ID and
endpoint-timestamp recall@k. It also writes
`mbes_selected_loop_remapped_allowlist.csv` and `mbes_selected_loop_remap.md`,
which map normal selected loop rows onto accepted-looking target replay rows
inside a timestamp window. Use that remapped CSV only as a diagnostic second-pass
allowlist; it is not a substitute for a stable loop identity or false-positive
review. The comparison report checks pose-graph RMSE from the two
`mbes_loop_trajectory_metrics.py` reports; the allowlist audit checks that a
selected replay did not accept loop IDs outside the CSV used for
`loop.selection.allowlist_csv`. The wrapper exits before the selected replay if
the normal replay writes only a CSV header and no selected loop rows. By default
the wrapper also exits non-zero when the allowlist audit finds off-allowlist
accepted loops. Set
`ALLOWLIST_AUDIT_STRICT=0` only when you need to preserve a diagnostic replay
bundle despite that failure.

The wrapper also assigns separate `ROS_DOMAIN_ID` values to the normal and
selected replays by default. This keeps stale transient-local
`/aqua_pose_graph/keyframe` publishers or interrupted prior runs from feeding
old keyframes into a new loop-closure node. Set `NORMAL_ROS_DOMAIN_ID` and
`SELECTED_ROS_DOMAIN_ID` when you need fixed values for a reproducible run log.
If coverage still shows keyframe-ID drift, set
`POSE_GRAPH_ODOMETRY_TOPIC=/nav/processed/odometry` to generate pose-graph
keyframes from the recorded source navigation topic instead of the replayed
`/aqua_imu_loc/odometry` estimate. Use that as a determinism probe first; do
not treat it as an accuracy claim unless the baseline definition is documented
with the report.

Replay recording waits for rosbag2 to subscribe to essential output topics
before playback starts (`RECORD_READY_TIMEOUT_S=75` by default; override
`RECORD_READY_TOPICS` to tune the set). Playback also starts with
`PLAY_START_DELAY_S=25` so rosbag2 can discover source input publishers before
messages flow. Keep both waits enabled for paired comparisons so normal and
selected runs start with similar topic coverage.

Useful live checks while tuning:

```bash
ros2 topic echo /mbes_loop_closure/status
ros2 topic echo /aqua_pose_graph/loop_constraint_count
ros2 topic echo /aqua_pose_graph/optimization_count
ros2 topic echo /aqua_pose_graph/optimization_chi2
```

The pose-graph backend can apply a robust kernel to external loop constraints
via:

```yaml
loop_constraints:
  robust_kernel:
    type: dcs
    delta: 1.0
```

This is a backend safety layer for false-positive sonar loop closures. It is
not a replacement for front-end auditing, degeneracy-aware factors, switchable
constraints, or consensus-based loop selection; it just limits how hard one bad
loop can pull the graph while those stronger filters are being developed.
`dcs` is the recommended field default, `huber` is available for comparison
runs, and `none` gives the plain quadratic loop-edge baseline.

Paper trail for the backend decision:

- Dynamic Covariance Scaling is the first backend guard because g2o already
  ships it and it directly targets bad loop-closure edges:
  <https://doi.org/10.1109/ICRA.2013.6630557>.
- Switchable Constraints remain a next candidate when loop acceptance needs a
  learned/optimized switch variable per closure:
  <https://nikosuenderhauf.github.io/assets/papers/IROS12-switchableConstraints.pdf>.
- Pairwise or group-k consistency selection is a stronger front-end/back-end
  gate for batches of candidate loops:
  <https://robots.engin.umich.edu/publications/jmangelson-2018a.pdf> and
  <https://doi.org/10.1177/02783649241256970>.

The live front end now has a lightweight accepted-loop consistency guard using
the same idea at a smaller scope: each accepted loop records its correction from
the odometry guess to the registration result, and later candidates can be
rejected as `loop consistency rejected` when their correction lacks the
configured support count from recorded accepted loops. The status stream records
the support count, required support count, and nearest translation/rotation
delta to the accepted-loop history for each evaluated candidate. Keep this
disabled until at least one replay has trusted accepted loops; a bad bootstrap
set would otherwise become the reference. Generate
`/tmp/mbes_loop_consistency_sweep.md` from the same replay used for descriptor
tuning to choose conservative initial translation/rotation delta thresholds,
then replay once more and check that rejected candidates are the intended
inconsistent loops.

`LoopClosureStatus.candidate_id` is `UINT32_MAX` when a keyframe has no
eligible historical submap. Rejections report the specific gate that failed,
`descriptor gate rejected` when the pre-registration shape check rejects a
candidate, `duplicate loop suppressed` when accepted-loop cooldown blocks a
near-repeat, or `loop consistency rejected` when an accepted-looking candidate
disagrees with previously accepted loop corrections. The consistency diagnostic
fields show whether a rejection was caused by too few supporting loops or by
translation/rotation deltas that are far outside threshold. This makes overly
strict candidate, descriptor, fitness, correction, repeat, or consistency
thresholds visible without reading debug logs.

In RViz, use the dedicated tuning config:

```bash
ros2 launch aqua_localization replay.launch.py \
  start_bag:=true \
  bag_path:=aqua_localization/datasets/public/mbes_slam/beach_pond_ros2 \
  bag_sonar_points_topic:=/norbit/detections \
  use_sim_time:=true \
  enable_pose_graph:=true \
  enable_mbes_loop_closure:=true \
  enable_rviz:=true \
  rviz_config_file:=$(ros2 pkg prefix aqua_localization)/share/aqua_localization/rviz/mbes_loop_closure.rviz
```

The config shows `/aqua_sonar_loc/points_filtered`,
`/aqua_pose_graph/path`, and `/mbes_loop_closure/markers`. Accepted loop
candidates are green and thicker; rejected candidates are red and thinner.
This makes it easy to see whether tuning is producing plausible geometric
edges before trusting them as pose-graph constraints.

`aqua_localization/scripts/rerun_export_mbes.py` understands the optional
pose-graph outputs when they are present in the results-included bag:

```bash
./aqua_localization/scripts/rerun_export_mbes.py \
  --bag aqua_localization/datasets/public/mbes_slam/demo_with_estimate \
  --out docs/media/mbes_slam.rrd
```

The 3D view overlays `/aqua_pose_graph/path` and accepted
`/aqua_pose_graph/loop_constraint` edges. The side plots include
`/mbes_loop_closure/status` accepted, fitness, and correction traces.

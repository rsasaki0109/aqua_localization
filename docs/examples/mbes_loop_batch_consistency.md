# MBES Loop Batch Consistency Example

This is a synthetic example of the markdown produced by:

```bash
ros2 run aqua_localization export_mbes_loop_status.py \
  --bag aqua_localization/datasets/public/mbes_slam/demo_with_estimate \
  --out /tmp/mbes_loop_status.csv \
  --batch-consistency-out /tmp/mbes_loop_batch_consistency.md \
  --batch-consistency-selected-csv-out /tmp/mbes_loop_batch_selected_loops.csv \
  --batch-consistency-translation-threshold-m 1.3 \
  --batch-consistency-rotation-threshold-rad 0.16
```

Use this report after the descriptor and runtime consistency sweeps. It is an
offline PCM-like selector over finite corrections that were accepted, or that
were rejected only by the runtime consistency guard.

## Input

- Source: synthetic example
- Candidate loop corrections: 6
- Pairwise correction pairs: 15
- Delta source: 15 pose-aware pairs, 0 scalar-magnitude fallback pairs
- Translation threshold: 1.30 m
- Rotation threshold: 0.16 rad

## Selection Summary

| Metric | Value |
|--------|------:|
| Selected consistent set | 4/6 |
| Batch-rejected candidates | 2 |
| Pairwise consistent edges | 7/15 |
| Algorithm | exact maximum clique |

## Selected Loop IDs

| Rank | Time s | Current | Candidate | Accepted | Degree | Fitness | Correction trans m | Correction rot rad | Status |
|-----:|-------:|--------:|----------:|---------:|-------:|--------:|-------------------:|-------------------:|--------|
| 1 | 18.4 | 42 | 7 | 1 | 3 | 0.12 | 0.84 | 0.07 | accepted |
| 2 | 25.1 | 57 | 12 | 1 | 3 | 0.18 | 0.91 | 0.08 | accepted |
| 3 | 33.9 | 73 | 20 | 1 | 3 | 0.21 | 1.02 | 0.10 | accepted |
| 4 | 40.6 | 89 | 31 | 0 | 3 | 0.24 | 1.08 | 0.11 | loop consistency rejected |

## Batch-Rejected Candidates

| Rank | Time s | Current | Candidate | Accepted | Degree | Fitness | Correction trans m | Correction rot rad | Status |
|-----:|-------:|--------:|----------:|---------:|-------:|--------:|-------------------:|-------------------:|--------|
| 1 | 47.2 | 105 | 44 | 0 | 0 | 0.28 | 3.60 | 0.42 | loop consistency rejected |
| 2 | 52.8 | 121 | 55 | 1 | 1 | 0.32 | 2.40 | 0.31 | accepted |

## How to Use the Report

The selected set is an audit and replay input, not a trajectory metric. A good
next replay should either constrain the pose graph with only the selected loop
IDs or inspect those IDs first in RViz/rerun, then compare APE/RPE against the
dataset reference. The batch-rejected rows are the first false-positive review
targets because they do not belong to the largest internally consistent
correction set.

The companion `/tmp/mbes_loop_batch_selected_loops.csv` has `current_id` and
`candidate_id` columns that can be used directly by:

```bash
MBES_LOOP_SELECTION_ALLOWLIST_CSV=/tmp/mbes_loop_batch_selected_loops.csv \
./aqua_localization/scripts/record_mbes_demo.sh
```

For drifted replay IDs, keep the same CSV and enable the opt-in signature
fallback from the exported status columns:

```bash
MBES_LOOP_SELECTION_ALLOWLIST_CSV=/tmp/mbes_loop_batch_selected_loops.csv \
MBES_LOOP_SELECTION_MATCH_TIMESTAMP_WINDOW_S=0.25 \
MBES_LOOP_SELECTION_MATCH_MAX_FITNESS_DELTA=0.05 \
MBES_LOOP_SELECTION_MATCH_MAX_TRANSLATION_DELTA_M=0.5 \
MBES_LOOP_SELECTION_MATCH_MAX_ROTATION_DELTA_RAD=0.05 \
MBES_LOOP_SELECTION_MATCH_MAX_DESCRIPTOR_CENTROID_DELTA_M=0.5 \
MBES_LOOP_SELECTION_MATCH_MAX_DESCRIPTOR_EXTENT_RATIO_DELTA=0.2 \
MBES_LOOP_SELECTION_MATCH_MAX_DESCRIPTOR_POINT_COUNT_RATIO_DELTA=0.2 \
./aqua_localization/scripts/record_mbes_demo.sh
```

Descriptor signature deltas are optional. Use them when the selected CSV was
exported with descriptor columns and endpoint timestamps alone leave nearby
candidate distractors inside the replay window.

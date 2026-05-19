# MBES Loop Descriptor Sweep Example

This is a synthetic example of the markdown produced by:

```bash
ros2 run aqua_localization export_mbes_loop_status.py \
  --bag aqua_localization/datasets/public/mbes_slam/demo_with_estimate \
  --descriptor-sweep-out /tmp/mbes_loop_descriptor_sweep.md
```

Do not copy these values into `config/mbes_loop_closure.yaml`. Use this file to
understand the report shape, then generate a sweep from your own replay.

## Input

- Source: synthetic example
- Tested candidates: 36
- Accepted by current runtime configuration: 4
- Descriptor thresholds at replay time: disabled

## Descriptor Threshold Sweep

Rows are sorted from stricter to looser descriptor gates. `Would pass` counts
how many tested candidates would pass the descriptor gate before registration.

| Centroid <= m | Extent <= ratio | Point count >= ratio | Would pass | Pass % |
|--------------:|----------------:|---------------------:|-----------:|-------:|
| 0.80 | 1.20 | 0.90 | 1 | 2.8% |
| 0.80 | 1.35 | 0.85 | 2 | 5.6% |
| 1.10 | 1.35 | 0.85 | 4 | 11.1% |
| 1.10 | 1.60 | 0.75 | 7 | 19.4% |
| 1.60 | 1.60 | 0.75 | 11 | 30.6% |
| 1.60 | 2.10 | 0.65 | 17 | 47.2% |
| 2.40 | 2.10 | 0.65 | 24 | 66.7% |

## How to Use the Table

Start with a row that leaves enough candidates to inspect in RViz, not the
strictest row. A descriptor gate is only a cheap pre-registration filter; final
acceptance still depends on the registration fitness and correction gates.

Translate one row into descriptor parameters like this:

```yaml
descriptor:
  enabled: true
  max_centroid_distance_m: 1.10
  max_extent_ratio: 1.60
  min_point_count_ratio: 0.75
```

Then replay the bag, export `/mbes_loop_closure/status` again, and compare:

- `descriptor gate rejected` count
- registration attempts that still fail fitness or correction gates
- accepted loop markers in `rviz/mbes_loop_closure.rviz`
- optimized path changes against the MBES-SLAM reference odometry

If the descriptor gate rejects almost everything, loosen centroid distance or
point-count ratio first. If many geometrically implausible candidates still
reach registration, tighten centroid distance or extent ratio before changing
the fitness gate.

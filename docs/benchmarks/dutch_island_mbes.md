# MBES-SLAM Dutch Island: Dead Reckoning and Bathymetric Loop Closure (2026-10-12)

Second Norbit survey of the MBES-SLAM datasets
([seaward.science/data/pos](https://seaward.science/data/pos/), Hammond,
Rowley, Roman, IJRR 2021): `dutch_harbor.tar.gz` / `large_area.bag`, Dutch
Island, Narragansett Bay. Same vessel, sensors and `/tf_static` as
[`beach_pond`](beach_pond_dead_reckoning.md); 8870 s, 18 km, 4-21 m deep,
parallel survey lines plus crossing transits. Reference: the bag's RTK
GNSS-INS `/nav/processed/odometry` (GNSS position is never an input).

Picked because beach_pond's crossings lie in a flat basin and are not
identifiable ([feasibility](beach_pond_mbes_loop_feasibility.md)). Dutch
Island has relief: depth std 3.3 m over the survey, slope p90 26 %.

## Dead reckoning

`run_beach_pond_benchmark.py --bag large_area_ros2 --start-offset-s 0`
(the DVL twist and heading are in the main bag; fingerprints compass
`59c295205a9bab09`, no compass `bc34316795c0d492`):

| Method | RMSE m | Max m | Final m | drift 100 m % | drift 1000 m % |
|--------|------:|------:|------:|------:|------:|
| aqua_graph (IMU + DVL + compass + depth hold) | **4.46** | 8.79 | 8.45 | 0.32 | 0.20 |
| DVL DR, compass yaw | 4.54 | 8.79 | 8.60 | 0.33 | 0.20 |
| DVL DR, Madgwick AHRS yaw | 556.58 | 1403.09 | 178.82 | 105.48 | 66.06 |
| aqua_graph without compass (IMU + DVL + depth hold) | 498.76 | 1402.68 | 308.18 | 93.19 | 54.90 |
| DVL DR, integrated gyro yaw | 553.09 | 1395.88 | 173.24 | 105.10 | 65.83 |

Without the compass the first 300 s are fine (1.73 m), then the
uncorrected gyro z bias takes over: yaw error 30 deg at 30 min, 74 deg at
1 h, 388 deg at the end. That is the regime loop closures have to fix.

## Bathymetric loop closure

### A bug in the loop-closure node (fixed)

`mbes_loop_closure` gave PCL the *inverse* of the dead-reckoned relative
pose as the source(current) -> target(candidate) initial transform, and
inverted the converged transform again before using it as the loop
constraint. Every loop therefore started twice the relative pose from the
truth, and a converged one became an inverted constraint. Before the fix,
all 8 loops it accepted on Dutch Island were wrong.
`RegistrationPipeline::match` now takes and returns the pose of the current
submap in the candidate frame, with a regression test. This also affects
the GICP part of the beach_pond loop findings (see the note there).

### GICP does not register these submaps

Offline, on two submaps built from *reference* poses at a crossing, PCL
GICP with the node's settings drifts to 1.4 m / 30 deg error **starting
from the true pose** (fitness 23 m^2). Partially overlapping, gently
sloped 2.5D seafloor patches give point-to-plane registration little to
hold on to.

### Coarse height-map search (new, off by default)

`registration.coarse.*` grids both submaps as height maps in the
gravity-levelled candidate frame and searches yaw (+-30 deg) x horizontal
shift (+-30 m), scoring the clipped RMS height residual (vertical offset =
median difference), then refines at a quarter of the steps. It rejects
placements whose overlap is flat (`min_relief_m`, detrended std) or not
unique (`min_distinctiveness`: best score elsewhere / best score), and
`registration.backend: none` uses its result as the registration.

- Reference-pose crossings (Python prototype, 200 m windows): the true
  pose is identified at 8 of 10 crossings with relief >= 2 m and 0 of 6
  under 1 m.
- C++ search on 40 s reference-pose submaps at three such crossings,
  started 5-12 m and up to 14 deg off: back within 0.1-1.5 m and 0-1.4 deg.
- On the node's own 60 m submaps (dead-reckoned motion compensation),
  started 5 m / 6 deg off at true overlaps: within ~1.5 m wherever relief
  >= 0.5 m and distinctiveness >= 1.2; small or flat overlaps fail.

### First online result: no loop accepted, none wrong

The first version of `dutch_island_mbes_slam.yaml` (no compass, 60 m keyframes, coarse search
with relief >= 0.5 m, distinctiveness >= 1.3, >= 300 overlapping cells,
`backend: none`): 516 candidates tested, **0 accepted** (187 too few
cells, 174 insufficient relief, 147 no overlap in range, 8 ambiguous). The
pose-graph path therefore equals dead reckoning. Two runs give the same
fingerprint (`4d7da5cee8012782`).

With the gates lowered to see what they block, only 16-22 of ~110 tested
candidates were true overlaps (the dead-reckoned candidate search is
already tens of metres off), the rest produced false placements, and no
gate setting kept a correct loop while dropping the false ones. Lowering
the gates is not an option.

### Where the online pipeline loses the loops (2026-10-12)

Measured on the node's own submaps (`debug.submap_dump_dir`, beams up to
63 deg) against the reference, with the coarse search run offline on every
pair (`--param ... coarse.min_relief_m:=0.0 min_distinctiveness:=0.0`):

- **Overlap geometry.** Outer beams are usable here (salt water): the
  height disagreement between passes is 0.065 m (inner), 0.12 m (45-63 deg)
  and 0.23 m (63-72 deg) median. Widening to 63 deg roughly doubles the
  overlapping 60 s windows (48 -> 106).
- **Candidates are not the bottleneck.** Of 203 truly overlapping submap
  pairs, the dead-reckoned guess is within the +-30 m / +-30 deg search
  range for 45 (median guess error 225 m / 44 deg over all 203), and the
  candidate search already tests 27 of those 45. A wider candidate radius
  only adds pairs the search cannot align.
- **Registration is.** On the 45 in-range true pairs the search returns
  the truth (3.5 m / 5 deg) for 15; where DR is already within 1 m, flat
  overlaps (relief < 0.2 m) still pull it tens of metres away. No
  per-candidate gate separates them: the best combination (relief >= 0.4 m,
  distinctiveness >= 1.2) keeps 4 correct and 3 false loops. A
  truncated-least-squares "support" score (off-map cells as outliers) was
  tried and did worse (8 correct of 376 placements vs 15 of 329).

### Group-k (PCM-style) loop selection

`loop.group.*` (off by default) holds gated loops in a pending pool and
publishes them only once at least `min_group_size` of them are mutually
consistent: for loops (a->b, z1) and (c->d, z2) the cycle
`z1^-1 (Ta^-1 Tc) z2 (Td^-1 Tb)` must be within 3 m / 5 deg plus
0.03 m/s and 0.03 deg/s of the odometry span `|ta-tc| + |tb-td|` (<= 200 s).

- Offline, on all 329 placements of the run above: correct loops close
  in time agree to <= 3 m / 5 deg; with relief >= 0.3 m, distinctiveness
  >= 1.05 and groups of >= 4, 6 loops survive: 5 correct and one 1.6 m /
  13 deg off (per-candidate gates alone: 9 correct, 18 false).
- Online (`--param mbes_loop_closure:loop.group.enabled:=true`,
  `min_group_size:=4`, `sensor.max_across_track_ratio:=2.0`,
  `coarse.min_relief_m:=0.3`, `min_distinctiveness:=1.05`,
  `min_overlap_cells:=150`): 4 loops published, 2 within 1 m / 2.4 deg and
  2 at 1.7 m / 6.1 deg and 5.9 m / 4.8 deg; no wild false loop. They all
  sit in one crossing area, so the pose-graph RMSE is unchanged (540.7 m vs
  540.8 m for dead reckoning at the same keyframes): a handful of local
  loops cannot undo a 388 deg yaw drift.

### Loop closures -> gyro bias in aqua_graph

The no-compass yaw drift is the vertical gyro bias, Earth rate included,
which the scenario locks at zero: measured against the reference, it is
~60 deg/h for the first hour and climbs to ~160 deg/h after 70 min
(5 min windows). A yaw-only model (dead reckoning = reference velocity
rotated by that integrated bias) reproduces the run (513 m vs 499 m RMSE;
1 m without the yaw error). In that model, one bias correction from a
single cluster of loops brings the RMSE to 340-430 m; tracking the bias
with loops every 5-15 min to 60-160 m.

`loop_bias.*` in aqua_graph (off unless `loop_bias.loop_topic` is set):
aqua_graph keeps its attitude and gyro-bias history and the keyframe
stamps. For each published loop, the dead-reckoned minus measured
relative yaw over the span observes the mean vertical bias over that
span (sigma `yaw_sigma_rad` / span). A scalar random-walk filter fuses
the observations, and the estimator is re-anchored with the fused bias.
The pose graph still handles the past; the bias fix stops future drift.

Online, full run, no compass (63 deg beams, relief >= 0.3 m,
distinctiveness >= 1.05, >= 300 cells, groups of >= 4):

| Run | Loops (correct) | aqua_graph RMSE m | Max m | Final m |
|-----|----------------:|------:|------:|------:|
| No loop feedback (group-k only) | 4 (2 within 2.4 deg) | 498.7 | 1403.2 | 307.0 |
| Loop -> gyro bias, groups >= 4 (scenario) | 4 (3) | **292.5** | 957.3 | 133.7 |
| Loop -> gyro bias, groups >= 3 | 3 (3) | 280.7 | 931.6 | 131.0 |

The loops all come from one crossing at ~75 min; the fused bias jumps
from ~-25 deg/h (slow drift of the locked bias) to 174 deg/h (truth
~130-150 deg/h at that time) and ends at 153 deg/h. No further group
forms afterwards: the second half is flatter (282 of 538 candidates fail
the relief gate), and the 21 gated loops there never reach a consistent
group. The pose graph adds nothing on top (RMSE at keyframes 314 m vs
317 m for its path). This was the scenario until match windows below
(fingerprint `3f1d5af9af6de2e5`).

### Multi-keyframe match windows

`registration.match_window: n` registers the union of n consecutive
submaps instead of one: centred on the candidate, trailing the current
keyframe (its successors do not exist yet), each in its own keyframe
frame, so the loop constraint is still between the two keyframes.

Offline, on the submaps of the bias-corrected run, with node-like
candidates (dead-reckoned anchors within 60 m):

| Window | True overlaps | Placed correctly | Gated correct : wrong |
|-------:|------:|------:|------:|
| 1 | 62 | 31 | 18 : 31 (relief >= 0.3, distinct >= 1.05) |
| 3 | 129 | 84 | 106 : 58 (same gates) |
| 5 | 163 | 100 | 115 : 43 (relief >= 0.4, distinct >= 1.1) |

Online, full run (bias feedback and groups of >= 4 as above):

| Run | Loops (correct) | Yaw change after 75 min | RMSE after 75 min* | RMSE all |
|-----|------:|------:|------:|------:|
| No loops (bias locked at 0) | 0 | ~+278 deg | - | 498.7 |
| Window 1 (previous scenario) | 4 (3) | -4 deg | 25 m | **292.5** |
| Window 3 (scenario, fingerprint `384f6c91a7dba88d`) | 33 (31) | -4 deg | **9 m** | 314.6 |
| Window 5, relief >= 0.4, distinct >= 1.1 | 8 (4) | +175 deg | 289 m | 552.6 |

\* the part after the first loop, aligned on its own.

With window 3, loops arrive at 75, 85, 91, 93, 115, 126 and 140 min,
and after each one the fused bias is within ~10-40 deg/h of the truth;
yaw then holds within a few degrees for the rest of the run. With
window 5 a few false loops pulled the bias off and yaw ran away again.

The whole-run RMSE does not improve because it is set by the first
75 min, which have no usable loops: in the first 70 min only 19 submap
pairs truly overlap and none registers correctly, so the ~115 deg of
yaw accumulated there stays (the 293 m vs 315 m difference is how each
run's later part aligns against that). Recovering it needs either a
heading source for the start (an initial bias estimate) or loops from
late to early areas, which the dead-reckoned candidate search cannot
find after 100 deg of drift (a place-recognition search would).

### With the compass, and on beach_pond (2026-10-12)

The same loop pipeline (63 deg beams, height-map search, 3-submap
windows, groups of >= 4; no `loop_bias`, the compass observes the bias)
on Dutch Island with the GNSS compass. Pose-graph overrides on top of
the scenario: odometry edges at compass quality
(`edges.default_translation_information: 44`, `..._rotation_information:
40000`), and in the last two rows the new
`edges.odometry_attitude_prior_information: 40000`:

| Run | Loops (correct) | Pose graph RMSE m | Dead reckoning at keyframes m |
|-----|------:|------:|------:|
| Scenario edge information (no-compass values) | 56 (52) | 67.4 | 4.4 |
| Compass edge information | 66 (56) | 23.5 | 4.4 |
| + correction gate 6 m / 0.1 rad | 37 (35) | 17.6 | 4.4 |
| + attitude prior | 76 (70) | 6.1 | 4.4 |
| + attitude prior and 6 m / 0.1 rad gate | 66 (63) | 5.5 | 4.4 |

The loops are right, but with relative edges only the pose graph lets the
attitude random-walk along the chain and so throws away the compass's
absolute heading: loops with 1-2.5 deg of yaw error (within what counts
as correct) rotated whole segments, and correct long loops were left
6-23 m unsatisfied (edge lengths changed by < 5 cm: the distortion was
pure rotation). The attitude prior fixes that, but the graph still does
not beat dead reckoning: with the compass it drifts 0.2 % of distance,
and loops accurate to 1-2 m (plus the few at 3-5 m) have nothing left to
correct. On this vessel with a compass, MBES loops do not improve the
trajectory.

beach_pond (`beach_pond_mbes_slam.yaml`, no compass): with GICP alone,
after the registration fix, all 6 published loops were ~50 m wrong. With
this pipeline it publishes none (flat basin, see
[the feasibility note](beach_pond_mbes_loop_feasibility.md)); dead
reckoning is unchanged (287.6 m RMSE). The scenario now uses it
(fingerprint `76f38e4f14fd6917`).

## Reproduce

```bash
curl -LO https://seaward.science/files/pos-datasets/bag/dutch_harbor.tar.gz
tar xzf dutch_harbor.tar.gz
rosbags-convert --src dutch_harbor/large_area.bag --dst large_area_ros2 \
  --dst-storage mcap --src-typestore ros1_noetic --dst-typestore ros2_jazzy \
  --include-topic /norbit/detections /nav/sensors/microstrain/imu/raw \
    /nav/processed/microstrain/imu/madgwick /nav/processed/odometry \
    /nav/sensors/navsat/ubx_pos/fix /nav/processed/nortek/dvl_twist \
    /nav/processed/navsat/ubx_hdg/imu /tf_static
ros2 run aqua_replay run_beach_pond_benchmark.py --bag large_area_ros2 \
  --start-offset-s 0 --out-dir /tmp/dutch_bench
ros2 run aqua_replay offline_replay \
  --scenario $(ros2 pkg prefix aqua_replay)/share/aqua_replay/scenarios/dutch_island_mbes_slam.yaml \
  --bag large_area_ros2 --out /tmp/dutch_island_mbes_slam
```

`debug.submap_dump_dir` writes every finalized submap (points in the
submap frame plus its world pose) for offline inspection.

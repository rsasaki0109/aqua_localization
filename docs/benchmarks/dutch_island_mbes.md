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

### Online result: no loop accepted, none wrong

`dutch_island_mbes_slam.yaml` (no compass, 60 m keyframes, coarse search
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

What would make it work:

- loop selection by mutual consistency (PCM / group-k): correct loops
  agree on the accumulated drift, false ones scatter; per-loop gates have
  reached their limit;
- larger and longer-overlap submaps than one keyframe interval (the
  successful offline crossings used 40-200 s of track);
- bounding the gyro-bias yaw drift (the root cause of the 225 m guess
  errors) rather than searching wider.

With the compass the drift is 0.2 % of distance, so on this vessel loops
matter for the compass-free case only.

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

# MBES-SLAM `beach_pond` Dataset Acquisition

This is the short operator checklist for getting the real MBES-SLAM
`beach_pond` bag into the layout expected by the loop-status benchmark. The
longer bring-up note is [`mbes_slam_demo.md`](mbes_slam_demo.md).

## Source

- Dataset: MBES-SLAM / POS marine robotics dataset.
- Sequence: `beach_pond`.
- Paper: Krasnosky, Roman, Casagrande, "A bathymetric mapping and SLAM dataset
  with high-precision ground truth for marine robotics", IJRR 2022.
- Public download page cited by the paper: <https://www.Seaward.Science/data/pos>
- Direct tarball used by this repository:
  <https://seaward.science/files/pos-datasets/bag/beach_pond.tar.gz>
- Verified on 2026-05-22: `HTTP 200`, `Content-Length: 2584672367`.

## Expected Layout

After download, extraction, and ROS 2 conversion, the benchmark path should be:

```text
datasets/public/mbes_slam/
  beach_pond.tar.gz
  beach_pond/
    beach_pond.bag
    20200717_beach_pond.surv
    ctd/
  beach_pond_ros2/
    metadata.yaml
    *.mcap
```

The loop-status benchmark commands assume:

```text
datasets/public/mbes_slam/beach_pond_ros2
```

## Download

```bash
mkdir -p datasets/public/mbes_slam
cd datasets/public/mbes_slam

wget --no-check-certificate \
  https://seaward.science/files/pos-datasets/bag/beach_pond.tar.gz

tar xzf beach_pond.tar.gz
```

Expected extracted ROS 1 bag:

```text
datasets/public/mbes_slam/beach_pond/beach_pond.bag
```

## Convert to ROS 2

Keep only the standard-typed topics needed by `aqua_localization`:

```bash
rosbags-convert \
  --src datasets/public/mbes_slam/beach_pond/beach_pond.bag \
  --dst datasets/public/mbes_slam/beach_pond_ros2 \
  --dst-storage mcap \
  --src-typestore ros1_noetic \
  --dst-typestore ros2_jazzy \
  --include-topic /norbit/detections \
                  /nav/sensors/microstrain/imu/raw \
                  /nav/processed/microstrain/imu/madgwick \
                  /nav/sensors/microstrain/mag/raw \
                  /nav/processed/odometry \
                  /nav/sensors/navsat/ubx_pos/fix \
                  /tf /tf_static
```

## Readiness Check

Before recording loop-status benchmark artifacts:

```bash
ros2 run aqua_localization check_mbes_benchmark_ready.py \
  --bag datasets/public/mbes_slam/beach_pond_ros2 \
  --out /tmp/mbes_beach_pond_readiness.md \
  --min-duration-s 60
```

The report must pass these required inputs:

| Role | Required topic |
|------|----------------|
| MBES points | `/norbit/detections` |
| Reference odometry | `/nav/processed/odometry` |
| IMU | `/nav/processed/microstrain/imu/madgwick` or `/nav/sensors/microstrain/imu/raw` |

If this fails, do not run the loop-status benchmark yet. Fix the conversion
topic list or bag path first.

## Next Step

Once the readiness report passes, continue with:

```bash
WORKSPACE=$PWD \
MBES_SRC=$PWD/datasets/public/mbes_slam/beach_pond_ros2 \
MBES_OUT=/tmp/aqua_mbes_beach_pond_with_loop_status \
OUT_DIR=/tmp/aqua_mbes_loop_benchmark \
MBES_DURATION=120 \
ros2 run aqua_localization run_mbes_loop_benchmark.sh
```

Then follow
[`docs/benchmarks/mbes_beach_pond_loop_status.md`](../docs/benchmarks/mbes_beach_pond_loop_status.md)
to export loop-status CSV, generate the benchmark row, and complete the
false-positive audit.

## Notes

- The tarball is large, so keep it out of git.
- The converted `beach_pond_ros2/` bag is also local benchmark data and should
  stay out of git.
- Record the exact commit hash, replay duration, and readiness report when
  promoting the benchmark case from `scaffolded` to `measured`.

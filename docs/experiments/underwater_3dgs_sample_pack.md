# Underwater 3DGS Sample Pack Workflow

This guide describes the small artifact that should be attached to a GitHub
Release once a camera-enabled Tank/AQUALOC bag is available locally.

The current checked-in Tank `short_test_ros2` bag is intentionally small and
does not include camera image or CameraInfo topics. Use a Tank conversion made
with `--include-cameras`, or an equivalent public camera bag.

## Target Artifact

- Name: `tank_short_test_3dgs_pack_20frames.zip`
- Source: Tank Dataset `short_test`
- Size target: small enough for GitHub Release download and quick inspection
- Frame count: 20 images
- Transform format: `nerfstudio`
- Pose convention: `world_from_camera = world_from_base @ base_from_camera`

## Build

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run aqua_localization export_3dgs_pack_pipeline.py \
  --bag datasets/public/tank_dataset/short_test_ros2_with_cameras \
  --dataset "Tank Dataset" \
  --sequence short_test \
  --out /tmp/tank_short_test_3dgs_pack_20frames \
  --force \
  --max-frames 20 \
  --stride 5 \
  --max-time-diff 0.05 \
  --base-from-camera -0.25 -0.45 0.0 0.0 0.0 0.0 1.0 \
  --format nerfstudio

cd /tmp
zip -r tank_short_test_3dgs_pack_20frames.zip tank_short_test_3dgs_pack_20frames
```

## Expected Files

```text
tank_short_test_3dgs_pack_20frames/
  README.md
  manifest.json
  pack_index.json
  summary.json
  frames.json
  transforms.json
  images/
    frame_000000.png
    ...
```

## Minimal Checks

```bash
python3 -m json.tool /tmp/tank_short_test_3dgs_pack_20frames/summary.json >/dev/null
python3 -m json.tool /tmp/tank_short_test_3dgs_pack_20frames/transforms.json >/dev/null
find /tmp/tank_short_test_3dgs_pack_20frames/images -type f | wc -l
```

The image count should match `summary.json` `counts.frames`. If
`counts.transforms` is lower than `counts.frames`, inspect skipped timestamps in
`transforms.json` metadata and increase `--max-time-diff` only if the bag clocks
are known to be consistent.

## Example `summary.json`

```json
{
  "schema": "aqua_localization.underwater_3dgs_pack_pipeline.v1",
  "dataset": "Tank Dataset",
  "sequence": "short_test",
  "status": "complete",
  "counts": {
    "frames": 20,
    "transforms": 20,
    "skipped_transforms": 0
  },
  "formats": {
    "transforms_format": "nerfstudio"
  }
}
```

## Example `frames.json` Entry

```json
{
  "file_path": "images/frame_000000.png",
  "timestamp_ns": 1652274610771027456,
  "message_stamp_ns": 1652274610771027456,
  "topic": "/camera/left/image_raw",
  "width": 1280,
  "height": 720
}
```

## Example `transforms.json` Entry

```json
{
  "file_path": "images/frame_000000.png",
  "transform_matrix": [
    [1.0, 0.0, 0.0, -0.25],
    [0.0, 1.0, 0.0, -0.45],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0]
  ],
  "metadata": {
    "time_diff_ns": 0
  }
}
```

## Release Checklist

1. Build the pack from a camera-enabled public bag.
2. Run the minimal checks above.
3. Upload `tank_short_test_3dgs_pack_20frames.zip` to the next GitHub Release.
4. Add the Release URL to the GitHub Pages 3DGS demo page.
5. Keep the README wording experimental until a trained reconstruction artifact
   is also published.

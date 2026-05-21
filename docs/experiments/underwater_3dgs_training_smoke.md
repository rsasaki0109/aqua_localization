# Underwater 3DGS Training Smoke Test

This guide turns the published v0.3 sample pack into a small external training
smoke test. The repository does not require nerfstudio, gsplat, CUDA, or other
training dependencies for normal ROS 2 development.

## Scope

- Input: `tank_short_test_3dgs_pack_20frames.zip`
- Source dataset: Tank Dataset `short_test`
- Frames: 20 PNG images
- Poses: 20 matched `/apriltag_slam/GT` transforms
- Format: nerfstudio-style `transforms.json`
- Gate: `check_3dgs_training_ready.py` must pass before training

This is a smoke test for data plumbing, not a reconstruction-quality benchmark.

## Fetch The Pack

```bash
mkdir -p /tmp/aqua_3dgs_smoke
cd /tmp/aqua_3dgs_smoke

curl -L \
  -o tank_short_test_3dgs_pack_20frames.zip \
  https://github.com/rsasaki0109/aqua_localization/releases/download/v0.3/tank_short_test_3dgs_pack_20frames.zip

unzip -o tank_short_test_3dgs_pack_20frames.zip
```

## Check Readiness

Run this from a built and sourced `aqua_localization` workspace:

```bash
ros2 run aqua_localization check_3dgs_training_ready.py \
  --pack /tmp/aqua_3dgs_smoke/tank_short_test_3dgs_pack_20frames
```

Expected result:

```text
3DGS training ready: true
format: nerfstudio
frames: 20
images: 20
intrinsics source: manual
```

## Training Command Placeholder

Use an external training environment. Keep it outside the ROS workspace unless
you intentionally want to install training dependencies there.

```bash
# Example shape only. Pin the actual tool version in the training PR.
cd /tmp/aqua_3dgs_smoke/tank_short_test_3dgs_pack_20frames

# nerfstudio-style path
ns-train splatfacto \
  --data . \
  --max-num-iterations 300 \
  --output-dir /tmp/aqua_3dgs_smoke/nerfstudio_out
```

If using a gsplat-based script instead, keep the same input directory and write
all outputs under `/tmp/aqua_3dgs_smoke/`.

## Expected Smoke Outputs

- `train.log` or captured terminal output
- first render or viewer screenshot
- training command and tool version
- elapsed time and GPU/CPU note
- failure note if the tool rejects the tiny 20-frame pack

Do not report reconstruction quality from this smoke test. A real comparison
needs a larger sequence, held-out views, and a separate metric plan.

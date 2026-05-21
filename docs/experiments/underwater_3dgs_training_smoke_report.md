# Underwater 3DGS Training Smoke Report

This records the first post-v0.5 training smoke attempt for the published Tank
Dataset sample pack. The result is intentionally limited: the data readiness
gate passes, but external training was not run because the local machine does
not currently have a usable training tool or GPU runtime.

## Run Metadata

- Date: 2026-05-22 06:52 JST
- Operator: Codex on local workstation
- Repository commit: `ae68f4d`
- Sample pack: `/tmp/tank_short_test_3dgs_pack_20frames`
- Sample pack zip: `/tmp/tank_short_test_3dgs_pack_20frames.zip`
- Sample pack SHA256:
  `96c8c63ea1ad07aaba6b3a7369330ab5749ebf6ae24eecc2f2f2484836cfd3d3`
- Report status: readiness pass / training blocked

## Machine

- OS: Ubuntu 22.04.5 LTS
- Kernel: Linux 6.8.0-111-generic x86_64
- CPU: Intel(R) Core(TM) i7-10875H CPU @ 2.30GHz
- CPU threads: 16
- RAM: 31 GiB total, 22 GiB available at run time
- GPU: not usable for this run
- GPU driver: `nvidia-smi` failed with `Driver/library version mismatch`
- CUDA / ROCm: not validated
- Python: 3.10.12

## Training Tool

- Tool: not available
- Tool version: not recorded
- Install method: not installed in this environment
- Environment path: not applicable
- Extra dependencies: not installed

Checked commands:

```bash
command -v ns-train
command -v gsplat
```

Both commands returned no executable path.

## Readiness Check

Command:

```bash
python3 aqua_localization/scripts/check_3dgs_training_ready.py \
  --pack /tmp/tank_short_test_3dgs_pack_20frames
```

Output:

```text
3DGS training ready: true
pack: /tmp/tank_short_test_3dgs_pack_20frames
format: nerfstudio
frames: 20
images: 20
intrinsics: 612x512 fx=655.0 fy=655.0 cx=306.0 cy=256.0
intrinsics source: manual
```

Additional sanity checks:

```text
zip size: 12M
transforms.json: valid JSON
transforms.json frames: 20
images directory: 20 PNG frames plus .gitkeep
camera model: pinhole
```

## Training Command

Training was not run.

The intended command shape remains:

```bash
cd /tmp/tank_short_test_3dgs_pack_20frames
ns-train splatfacto \
  --data . \
  --max-num-iterations 300 \
  --output-dir /tmp/aqua_3dgs_smoke/nerfstudio_out
```

## Result

- Outcome: blocked after readiness pass
- Elapsed time: readiness check completed immediately
- Peak GPU memory: not measured
- Output directory: not created
- Log path: not created
- First render path: not created
- Viewer screenshot path: not created

## Failure Details

- Failing command: external training was not attempted
- Error summary:
  - `ns-train` was not installed.
  - `gsplat` was not installed.
  - `nvidia-smi` failed with `Driver/library version mismatch`.
- Suspected cause: local training environment is not provisioned and the NVIDIA
  userspace/kernel driver stack is inconsistent.
- Next fix:
  - Provision a clean nerfstudio or gsplat environment outside the ROS
    workspace.
  - Repair the local NVIDIA driver/runtime mismatch.
  - Re-run this report with the same sample pack SHA256 and capture a 300-step
    training log.

## Notes

- The published v0.3 sample pack is structurally ready for nerfstudio-style
  training input.
- This report does not claim reconstruction quality.
- The next meaningful result is a short training run with pinned tool versions,
  elapsed time, GPU memory, logs, and the first render or viewer screenshot.

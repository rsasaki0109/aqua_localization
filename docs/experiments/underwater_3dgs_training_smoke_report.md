# Underwater 3DGS Training Smoke Report

Use this template for one training smoke-test attempt. Keep the result factual:
the goal is to prove data plumbing and capture failures, not to claim
reconstruction quality.

## Run Metadata

- Date:
- Operator:
- Repository commit:
- Sample pack:
- Sample pack SHA256:
- Report status: pass / fail / blocked

## Machine

- OS:
- CPU:
- RAM:
- GPU:
- GPU driver:
- CUDA / ROCm:
- Python:

## Training Tool

- Tool: nerfstudio / gsplat / other
- Tool version:
- Install method:
- Environment path:
- Extra dependencies:

## Readiness Check

Command:

```bash
ros2 run aqua_localization check_3dgs_training_ready.py \
  --pack /tmp/aqua_3dgs_smoke/tank_short_test_3dgs_pack_20frames
```

Output:

```text
3DGS training ready: true
format: nerfstudio
frames: 20
images: 20
intrinsics source: manual
```

## Training Command

```bash
# Paste the exact command.
```

## Result

- Outcome: pass / fail / blocked
- Elapsed time:
- Peak GPU memory:
- Output directory:
- Log path:
- First render path:
- Viewer screenshot path:

## Failure Details

Fill this in only for fail or blocked runs.

- Failing command:
- Error summary:
- Suspected cause:
- Next fix:

## Notes

- Do not report reconstruction quality from this smoke test.
- Record tool warnings that may affect future larger experiments.
- Link any generated screenshot or render artifact from the follow-up PR.

# Cloud environment handover, 2026-10-10

The old local `51cea95` commit and external helpers were not restored. This
workspace contains newly prepared ROS Jazzy infrastructure and separately
registered reconstruction results. The requested `cloud-environment-runtime`
skill was unavailable; actual container/build/ROS checks were used instead.

Start the retained environment with:

```bash
bash /workspace/aqua-cloud/activate.bash
/workspace/.aqua-cloud/ros python3 /workspace/aqua-cloud/reuse_reconstruction_v4.py
```

Full reusable setup is `/workspace/aqua-cloud/saved-install-script-current.bash`.
It starts the retained image, checks pinned Python dependencies, builds all six
ROS packages, verifies rather than overwrites the registered v4 executable/SDK,
and foreground-verifies the complete native harbor_05 dataset. Its final rerun
passed, as did pose graph (5 keyframes/1 loop) and headless pressure/IMU fusion
readiness. The benchmark contract suite passed 69 tests; the existing ROS suite
previously passed 704 tests. These checks are not accuracy/SoTA evidence.

The native-data gate and snapshot record concern dataset integrity only. Their
`not_executed`/`not_performed` estimator/reference fields describe that earlier
readiness stage, not the later v4 evaluation. Current evaluation state and all
trial/score hashes are in [active status](active_real_data_evaluation.json) and
[results](harbor05_reconstruction_v4_results.json).

Runtime helpers, Python environment, retained Docker image archive, SDK/library,
calibration provenance, native data and immutable versioned evaluation artifacts
are outside the project checkout, under `/workspace/.aqua-cloud`,
`/workspace/aqua-cloud` and `/workspace/aqua-data`. They must be included in the
new environment filesystem snapshot; a Git checkout alone does not restore them.
Live processes do not persist. No fresh-task restoration or environment
publication is claimed. Saved settings were read back exactly; review/save the
settings and publish through the environment UI to make the prepared snapshot
available. Never reuse an old environment ID or its revision-15 draft.

Harbor_05 reference has now been read. Do not modify registered method/input/
scoring files in place and treat subsequent changes as blind evaluation. Preserve
all versions and use new methods plus new held-out evaluation registrations.
No Tank contact/application is authorized. Independent hardware truth and
competitive methods remain the next unexecuted gates.

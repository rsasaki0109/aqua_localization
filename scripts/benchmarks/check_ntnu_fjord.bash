#!/usr/bin/env bash
set -euo pipefail
exec 9>/workspace/aqua-next/ntnu/fjord-snapshot-check.lock
flock -n 9 || { echo 'NTNU snapshot checks already running' >&2; exit 1; }
# Read-only: missing/corrupt frozen artifacts require recovery, not an in-place rebuild.
/workspace/.aqua-cloud/ros python /workspace/aqua_localization/scripts/benchmarks/check_ntnu_snapshot.py --root /workspace/aqua-data/development/ntnu_fjord_01
/workspace/.aqua-cloud/ros /workspace/aqua-next/ntnu/imu-merge-fixed-test
/workspace/.aqua-cloud/ros /workspace/aqua-next/ntnu/observation-boundary-test
/workspace/.aqua-cloud/ros python -m pytest /workspace/aqua_localization/tests/benchmarks/test_ntnu_hf_acquisition.py /workspace/aqua_localization/tests/benchmarks/test_ntnu_sensor_only.py /workspace/aqua_localization/tests/benchmarks/test_ntnu_mono_input.py -q

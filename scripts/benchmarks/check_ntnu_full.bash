#!/usr/bin/env bash
set -euo pipefail
exec 9>/workspace/aqua-next/ntnu/fjord-full-snapshot-check.lock
flock -n 9 || { echo 'Full NTNU snapshot checks already running' >&2; exit 1; }
/workspace/.aqua-cloud/ros python /workspace/aqua_localization/scripts/benchmarks/check_ntnu_full_snapshot.py --root /workspace/aqua-data/development/ntnu_fjord_01
/workspace/.aqua-cloud/ros /workspace/aqua-next/ntnu/end-of-stream-test
/workspace/.aqua-cloud/ros python -m pytest /workspace/aqua_localization/tests/benchmarks/test_ntnu_full_input.py -q

#!/usr/bin/env bash
set -euo pipefail
exec 9>/workspace/aqua-next/ntnu/openvins-snapshot-check.lock
flock -n 9 || { echo 'OpenVINS snapshot checks already running' >&2; exit 1; }
/workspace/.aqua-cloud/ros python /workspace/aqua_localization/scripts/benchmarks/check_ntnu_openvins.py --root /workspace/aqua-data/development/ntnu_fjord_01
/workspace/.aqua-cloud/ros /workspace/aqua-next/ntnu/openvins-observability-test
# Integrity success does not mean failed initialization is ready for scoring.

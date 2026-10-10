#!/usr/bin/env bash
set -euo pipefail
output_directory=/workspace/aqua-next/ntnu/openvins-native-build-v8
mkdir "$output_directory"
test "$(git -C /workspace/.aqua-cloud/third-party/open_vins rev-parse HEAD)" = 69488123ed9362dd44b6f28e7f4680abbff1442b
# Reuse the verified numerical SDK. Never rebuild frozen SDK or prior adapters.
/workspace/.aqua-cloud/ros bash -c 'g++ -std=c++17 -O2 -I/workspace/.aqua-cloud/third-party/open_vins/ov_msckf/src -I/workspace/.aqua-cloud/third-party/open_vins/ov_core/src -I/workspace/.aqua-cloud/third-party/open_vins/ov_init/src -I/usr/include/eigen3 $(pkg-config --cflags opencv4) /workspace/aqua_localization/scripts/benchmarks/openvins_ntnu_v6.cpp -L/workspace/.aqua-cloud/openvins-build -Wl,-rpath,/workspace/.aqua-cloud/openvins-build -lov_msckf_lib -lboost_filesystem -lboost_system $(pkg-config --libs opencv4) -o /workspace/aqua-next/ntnu/openvins-native-build-v8/openvins-ntnu'
/workspace/.aqua-cloud/ros "$output_directory/openvins-ntnu" --describe /workspace/aqua_localization/cfg/benchmarks/ntnu_openvins_fixed.json > "$output_directory/resolved-profile.log" 2>&1

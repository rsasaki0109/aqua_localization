#!/usr/bin/env bash
set -euo pipefail
repo=/workspace/aqua_localization
dependency=/workspace/.aqua-cloud/third-party/open_vins
output_directory=${1:-/workspace/aqua-cloud}
mkdir -p "$output_directory"
output="$output_directory/openvins-reconstructed"
if [[ -e "$output_directory/reconstructed-build-manifest.json" ]]; then
  echo 'Existing versioned build retained; use a new output directory.' >&2
  exit 1
fi
bash /workspace/aqua-cloud/prepare_openvins.bash
/workspace/.aqua-cloud/ros bash -c 'g++ -std=c++17 -O2 -I/workspace/.aqua-cloud/third-party/open_vins/ov_msckf/src -I/workspace/.aqua-cloud/third-party/open_vins/ov_core/src -I/workspace/.aqua-cloud/third-party/open_vins/ov_init/src -I/usr/include/eigen3 $(pkg-config --cflags opencv4) /workspace/aqua_localization/scripts/benchmarks/openvins_reconstructed.cpp -L/workspace/.aqua-cloud/openvins-build -Wl,-rpath,/workspace/.aqua-cloud/openvins-build -lov_msckf_lib $(pkg-config --libs opencv4) -o "$1"' bash "$output"
/workspace/.aqua-cloud/ros "$output" --self-test
/workspace/.aqua-cloud/ros python3 "$repo/scripts/benchmarks/check_reconstructed_calibration.py" --executable "$output"
# Bind the produced executable to the exact source/dependency/compile settings.
/workspace/.aqua-cloud/ros python3 - "$output_directory" <<'PY'
import hashlib,json,subprocess,sys
from pathlib import Path
repo=Path('/workspace/aqua_localization')
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
record={'upstream_commit':subprocess.check_output(['git','-C','/workspace/.aqua-cloud/third-party/open_vins','rev-parse','HEAD'],text=True).strip(),
        'compile_flags':['-std=c++17','-O2'],
        'compiler':subprocess.check_output(['g++','--version'],text=True).splitlines()[0],
        'source_sha256':{str(repo/'scripts/benchmarks'/n):sha(repo/'scripts/benchmarks'/n) for n in ('openvins_reconstructed.cpp','openvins_pressure.hpp')},
        'executable_sha256':sha(Path(sys.argv[1])/'openvins-reconstructed'),
        'upstream_library_sha256':sha('/workspace/.aqua-cloud/openvins-build/libov_msckf_lib.so'),
        'self_check':'passed; mathematical integration, not accuracy evidence'}
with (Path(sys.argv[1])/'reconstructed-build-manifest.json').open('x') as f:
 f.write(json.dumps(record,indent=2)+'\n')
PY

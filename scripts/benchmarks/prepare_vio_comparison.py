"""Register a new support/scoring contract without replacing missing old plans.

The input conversion is runnable. Estimator comparison remains explicitly blocked
until the original frozen implementation/calibration is recovered or a separately
documented new method is registered. This file does not access a reference.
"""
import argparse
from datetime import datetime, timezone
import json
from importlib.metadata import version as package_version
import platform
from pathlib import Path

from audit_native_imu_support import sha256, write_new_json
from ensure_harbor05_data import BAG_MEMBER, BAG_SHA256, EXPECTED_ALL_MESSAGES
from openvins_input_support import CAMERA_OFFSET_NS, POLICY_VERSION

OPENVINS_COMMIT = "69488123ed9362dd44b6f28e7f4680abbff1442b"
SOURCE_FILES = ["scripts/benchmarks/" + name for name in (
    "openvins_input_support.py", "audit_native_imu_support.py",
    "openvins_supported_bag.py", "verify_openvins_stream.py",
    "prepare_vio_comparison.py", "ensure_harbor05_data.py")]
MISSING = ["original pressure update implementation and measurement convention",
           "frozen estimator configuration and camera/IMU/pressure calibration",
           "original native runner and stream verifier",
           "old comparison/scoring registrations and independent score checker"]


def register(root, repo, version="vio-comparison-v2"):
    root, repo = Path(root), Path(repo)
    ready = json.loads((root / "ready.json").read_text())
    if (ready.get("status") != "native_data_verified" or
        ready.get("native_bag_sha256") != BAG_SHA256 or
        ready.get("all_ros_message_count") != EXPECTED_ALL_MESSAGES):
        raise ValueError("complete native data verification is required before registration")
    if sha256(root / "native" / BAG_MEMBER) != BAG_SHA256:
        raise ValueError("native input differs from the verified ready record")
    destination = root / version
    if destination.parent != root or not version.startswith("vio-comparison-v"):
        raise ValueError("use a new comparison version below the data root")
    destination.mkdir(exist_ok=False)
    source_hashes = {name: sha256(repo / name) for name in SOURCE_FILES}
    common = {"schema": "aqualoc-vio-contract-v2", "registered_utc": datetime.now(timezone.utc).isoformat(),
              "sequence": "harbor_05", "reference_access": "not_performed",
              "native_bag_sha256": BAG_SHA256, "ready_sha256": sha256(root / "ready.json"),
              "source_sha256": source_hashes, "openvins_commit": OPENVINS_COMMIT,
              "runtime_versions": {"python": platform.python_version(),
                  "rosbags": package_version("rosbags"), "numpy": package_version("numpy")},
              "legacy_registration": "unavailable; not overwritten or reconstructed",
              "legacy_head": "51cea95 (user-reported; object unavailable)",
              "input_contract": {"policy_version": POLICY_VERSION,
                  "camera_offset_ns": CAMERA_OFFSET_NS, "pressure_offset_ns": 0,
                  "downstream_camera_offset_ns": 0,
                  "online_camera_time_offset_calibration": False,
                  "imu_consumption": "buffer native IMU through each included observation's upper bracket before feeding it; no extrapolation",
                  "support_interval": "closed native IMU header timestamp interval",
                  "outside_support": "exclude from derived stream, retain native sample and explicit audit",
                  "shared_camera_mask": True, "imu_extrapolation": False,
                  "timestamp_or_calibration_tuning_against_reference": False,
                  "expected_native_counts": {"imu": 34576, "camera": 3458, "pressure": 1650},
                  "expected_exclusions": [{"sensor": "camera", "native_ordinal": 0,
                      "exclusion_reason": "before_native_imu_support",
                      "distance_outside_support_ns": 6537851}],
                  "expected_boundary_provenance": "user-reported old verification; must match newly acquired data"},
              "trials": [{"id": f"{arm}-{index}", "arm": arm, "repeat": index}
                         for arm in ("vio", "pressure_vio") for index in (1, 2)],
              "comparison_status": "blocked_missing_frozen_method",
              "comparison_executable": False, "missing_prerequisites": MISSING,
              "method_sha256": None, "calibration_sha256": None,
              "freeze_gate": "all four successes/failures, output hashes, input/source/config hashes, runtime and initialization exclusions fixed before reference acquisition",
              "reference_limitation": "AQUALOC same-image COLMAP with depth-scaled metric reference; not independent hardware ground truth",
              "development_sequence": "harbor_06 already used reference; user-reported ~4.3% is not replicated here",
              "tank_data_application": "skipped by user; no contact or application"}
    scoring = dict(common, schema="aqualoc-vio-scoring-contract-v2",
        scoring_status="blocked_until_complete_estimator_freeze",
        scoring={"alignment": "rigid SE(3)", "scale": 1.0,
                 "interpolation_interval_max_seconds": 0.1,
                 "time_matching": "integer nanoseconds; exact or bracketed reference interpolation; no extrapolation",
                 "individual_trial_support": True, "common_support_of_all_four_trials": True,
                 "initialization_exclusions": "record actual initialization time and exclusions per trial; no reference-driven selection",
                 "failed_trials": "report every failure; no replacement runs after reference access",
                 "independent_verification": "independent timestamp correspondence and Horn quaternion alignment required",
                 "reference_optimization": {"clock": False, "scale": False, "calibration": False}})
    # Copy frozen source bytes; these snapshots cannot be overwritten by reruns.
    for relative in SOURCE_FILES:
        target = destination / "source" / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open("xb") as stream:
            stream.write((repo / relative).read_bytes())
        if sha256(target) != source_hashes[relative]:
            raise ValueError("source changed during registration; incomplete version retained")
    write_new_json(destination / "scoring-plan.json", scoring)
    common["scoring_plan_sha256"] = sha256(destination / "scoring-plan.json")
    write_new_json(destination / "plan.json", common)
    write_new_json(destination / "registration.json", {
        "status": "registered_input_contract_and_blocked_comparison",
        "plan_sha256": sha256(destination / "plan.json"),
        "scoring_plan_sha256": common["scoring_plan_sha256"],
        "reference_access": "not_performed"})
    return destination


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--version", default="vio-comparison-v2")
    args = parser.parse_args()
    print(register(args.root, args.repo, args.version))

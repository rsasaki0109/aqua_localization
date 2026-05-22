"""Tests for MBES loop audit RViz marker helper logic."""

import importlib.util
import sys
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "publish_mbes_loop_audit_markers.py"
)


def load_module():
    scripts_dir = str(SCRIPT_PATH.parent)
    if scripts_dir not in sys.path:
        sys.path.insert(0, scripts_dir)
    spec = importlib.util.spec_from_file_location("publish_mbes_loop_audit_markers", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_build_marker_specs_skips_missing_keyframes():
    module = load_module()
    audit = module.audit
    high = audit.LoopStatusRow(
        current_id=64,
        candidate_id=42,
        accepted=True,
        converged=True,
        fitness_score=1.8,
        correction_translation_m=4.7,
        correction_rotation_rad=0.42,
        descriptor_centroid_distance_m=1.0,
        descriptor_extent_ratio=6.0,
        descriptor_point_count_ratio=0.45,
        status="accepted",
    )
    missing = audit.LoopStatusRow(
        current_id=100,
        candidate_id=99,
        accepted=True,
        converged=True,
        fitness_score=0.2,
        correction_translation_m=0.3,
        correction_rotation_rad=0.1,
        descriptor_centroid_distance_m=0.1,
        descriptor_extent_ratio=1.0,
        descriptor_point_count_ratio=0.9,
        status="accepted",
    )

    class Args:
        max_fitness = 2.0
        max_translation_m = 5.0
        max_rotation_rad = 0.5
        min_keyframe_separation = 20
        descriptor_extent_warn = 5.0
        descriptor_point_ratio_warn = 0.5

    audit_rows = audit.accepted_audit_rows([missing, high], Args)
    keyframes = {
        42: module.KeyframePose(42, 1.0, 2.0, -5.0),
        64: module.KeyframePose(64, 4.0, 6.0, -7.0),
    }

    specs = module.build_marker_specs(
        audit_rows,
        keyframes,
        max_markers=10,
        label_z_offset=2.0,
    )

    assert len(specs) == 1
    assert specs[0].rank == 1
    assert specs[0].priority == "high"
    assert specs[0].candidate_xyz == (1.0, 2.0, -5.0)
    assert specs[0].current_xyz == (4.0, 6.0, -7.0)
    assert specs[0].label_xyz == (2.5, 4.0, -4.0)
    assert "#1 high 42->64" in specs[0].label


def test_marker_scale_and_colors_are_priority_specific():
    module = load_module()

    assert module.marker_scale("high") > module.marker_scale("medium")
    assert module.marker_scale("medium") > module.marker_scale("low")
    assert module.PRIORITY_COLORS["high"] != module.PRIORITY_COLORS["low"]


def test_parse_args_defaults_to_audit_marker_topic(tmp_path):
    module = load_module()

    args = module.parse_args([
        "--bag",
        str(tmp_path / "bag"),
        "--csv",
        str(tmp_path / "status.csv"),
    ])

    assert args.topic == "/mbes_loop_audit/markers"
    assert args.keyframe_topic == "/aqua_pose_graph/keyframe"
    assert args.max_markers == 35

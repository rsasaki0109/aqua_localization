"""Tests for MBES accepted-loop geometry review generation."""

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "audit_mbes_loop_geometry.py"
)


def load_module():
    scripts_dir = str(SCRIPT_PATH.parent)
    if scripts_dir not in sys.path:
        sys.path.insert(0, scripts_dir)
    spec = importlib.util.spec_from_file_location("audit_mbes_loop_geometry", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_build_geometry_rows_computes_keyframe_distances():
    module = load_module()
    audit = module.audit
    marker_helpers = module.marker_helpers
    row = audit.LoopStatusRow(
        current_id=12,
        candidate_id=4,
        accepted=True,
        converged=True,
        fitness_score=0.2,
        correction_translation_m=1.5,
        correction_rotation_rad=0.1,
        descriptor_centroid_distance_m=0.3,
        descriptor_extent_ratio=1.1,
        descriptor_point_count_ratio=0.9,
        status="accepted",
    )
    audit_row = audit.AuditRow(
        row=row,
        keyframe_gap=8,
        risk_score=0.4,
        priority="low",
        flags=(),
    )
    keyframes = {
        4: marker_helpers.KeyframePose(4, 0.0, 0.0, -5.0),
        12: marker_helpers.KeyframePose(12, 3.0, 4.0, -7.0),
    }

    rows = module.build_geometry_rows(
        [audit_row],
        keyframes,
        max_rows=10,
        min_plan_distance_m=2.0,
    )

    assert len(rows) == 1
    assert rows[0].plan_xy_distance_m == 5.0
    assert rows[0].depth_delta_m == 2.0
    assert rows[0].review_focus == "geometry-only review"


def test_missing_geometry_rows_reports_absent_keyframes():
    module = load_module()
    audit = module.audit
    marker_helpers = module.marker_helpers
    row = audit.LoopStatusRow(
        current_id=12,
        candidate_id=4,
        accepted=True,
        converged=True,
        fitness_score=0.2,
        correction_translation_m=1.5,
        correction_rotation_rad=0.1,
        descriptor_centroid_distance_m=0.3,
        descriptor_extent_ratio=1.1,
        descriptor_point_count_ratio=0.9,
        status="accepted",
    )
    audit_row = audit.AuditRow(
        row=row,
        keyframe_gap=8,
        risk_score=0.4,
        priority="low",
        flags=(),
    )
    keyframes = {
        4: marker_helpers.KeyframePose(4, 0.0, 0.0, -5.0),
    }

    missing = module.missing_geometry_rows([audit_row], keyframes, max_rows=10)

    assert missing == [module.MissingGeometryRow(audit_row, "current")]


def test_keyframe_id_range_reports_loaded_coverage():
    module = load_module()
    marker_helpers = module.marker_helpers
    keyframes = {
        4: marker_helpers.KeyframePose(4, 0.0, 0.0, -5.0),
        12: marker_helpers.KeyframePose(12, 3.0, 4.0, -7.0),
    }

    assert module.keyframe_id_range(keyframes) == (4, 12)
    assert module.keyframe_id_range({}) is None


def test_format_report_contains_geometry_table(tmp_path):
    module = load_module()
    audit = module.audit
    row = audit.LoopStatusRow(
        current_id=12,
        candidate_id=4,
        accepted=True,
        converged=True,
        fitness_score=1.8,
        correction_translation_m=4.5,
        correction_rotation_rad=0.45,
        descriptor_centroid_distance_m=0.3,
        descriptor_extent_ratio=1.1,
        descriptor_point_count_ratio=0.9,
        status="accepted",
    )
    geometry_row = module.GeometryAuditRow(
        audit_row=audit.AuditRow(
            row=row,
            keyframe_gap=8,
            risk_score=2.0,
            priority="high",
            flags=("translation near gate",),
        ),
        candidate_xyz=(0.0, 0.0, -5.0),
        current_xyz=(3.0, 4.0, -7.0),
        plan_xy_distance_m=5.0,
        depth_delta_m=2.0,
        review_focus="high-risk gate margin; translation near gate",
    )
    args = SimpleNamespace(
        bag=tmp_path / "bag",
        csv=tmp_path / "status.csv",
        max_fitness=2.0,
        max_translation_m=5.0,
        max_rotation_rad=0.5,
        priority="all",
    )

    text = module.format_report(
        [geometry_row],
        args=args,
        total_accepted=1,
        keyframe_count=20,
        keyframe_range=(4, 12),
        missing_rows=[],
    )

    assert "# MBES Accepted Loop Geometry Review" in text
    assert "Accepted loops with keyframe geometry: 1" in text
    assert "Accepted loops missing keyframe geometry: 0" in text
    assert "Loaded keyframe ID range: 4 -> 12" in text
    assert "4 -> 12" in text
    assert "5 | 2 | 4.5 | 0.45" in text
    assert "high-risk gate margin; translation near gate" in text


def test_parse_args_defaults(tmp_path):
    module = load_module()

    args = module.parse_args([
        "--bag",
        str(tmp_path / "bag"),
        "--csv",
        str(tmp_path / "status.csv"),
    ])

    assert args.keyframe_topic == "/aqua_pose_graph/keyframe"
    assert args.max_accepted == 100
    assert args.priority == "all"
    assert args.min_plan_distance_m == 2.0
    assert not args.require_complete

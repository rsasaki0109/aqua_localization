"""Tests for prepare_tank_dvl_heldout_inputs.py."""

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "prepare_tank_dvl_heldout_inputs.py"
)
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("prepare_tank_dvl_heldout_inputs", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def minimal_args(tmp_path, **overrides):
    profile = tmp_path / "profile.yaml"
    reference = tmp_path / "ref.tum"
    ros2_bag = tmp_path / "medium_ros2"
    benchmark = tmp_path / "bench.md"
    for path in (profile, reference, benchmark):
        path.write_text("x\n", encoding="utf-8")
    ros2_bag.mkdir()
    values = dict(
        sequence="Medium",
        profile=profile,
        ros1_bag=None,
        ros2_bag=ros2_bag,
        reference=reference,
        visual=None,
        out_dir=tmp_path / "out",
        benchmark_markdown=benchmark,
        skip_convert=False,
        skip_visual=False,
        skip_bundle=False,
        dry_run=True,
        visual_system="aqua_visual_frontend_direct",
        translation_scale=0.095,
        min_pnp_inliers=12,
        min_inlier_ratio=0.85,
        ransac_iterations=100,
        ransac_reprojection_error_px=4.0,
        ransac_confidence=0.99,
        max_step_translation_m=0.02,
        camera_fx=641.9,
        camera_fy=641.9,
        camera_cx=306.0,
        camera_cy=256.0,
        camera_bf=82.8,
        base_from_camera_x_m=-0.15,
        base_from_camera_y_m=-0.55,
        base_from_camera_z_m=0.0,
        orb_n_features=1000,
        orb_fast_threshold=12,
        opencv_threads=2,
        max_corrected_rmse_m=0.0194,
        max_gap_x=1.0,
        min_improvement_percent=None,
    )
    values.update(overrides)
    return SimpleNamespace(**values)


def test_resolve_paths_uses_sequence_stem(tmp_path):
    module = load_module()
    args = minimal_args(tmp_path, sequence="Medium Sequence")

    paths = module.resolve_paths(args)

    assert paths.ros2_bag == args.ros2_bag
    assert paths.visual_tum == tmp_path / "out" / "visual" / "Medium_Sequence_visual_frontend.tum"
    assert paths.bundle_out_dir == tmp_path / "out" / "validation_bundle"


def test_build_visual_command_uses_strict_defaults(tmp_path):
    module = load_module()
    args = minimal_args(tmp_path)
    paths = module.resolve_paths(args)

    command = module.build_visual_command(args, paths)

    assert "run_tank_visual_direct_benchmark.py" in command
    assert command[command.index("--translation-scale") + 1] == "0.095"
    assert command[command.index("--min-inlier-ratio") + 1] == "0.85"
    assert command[command.index("--max-step-translation-m") + 1] == "0.02"
    assert command[command.index("--base-from-camera-y-m") + 1] == "-0.55"


def test_build_bundle_command_omits_missing_optional_benchmark_markdown(tmp_path):
    module = load_module()
    args = minimal_args(tmp_path, benchmark_markdown=None)
    paths = module.resolve_paths(args)

    command = module.build_bundle_command(args, paths)

    assert "--benchmark-markdown" not in command


def test_missing_inputs_requires_visual_when_skip_visual(tmp_path):
    module = load_module()
    args = minimal_args(tmp_path, skip_visual=True)
    paths = module.resolve_paths(args)

    missing = module.missing_inputs(args, paths)

    assert "--visual is required when --skip-visual is used" in missing


def test_planned_commands_skip_existing_conversion_when_ros2_bag_exists(tmp_path):
    module = load_module()
    ros1 = tmp_path / "medium.bag"
    ros1.write_bytes(b"bag")
    args = minimal_args(tmp_path, ros1_bag=ros1, ros2_bag=None)
    paths = module.resolve_paths(args)
    paths.ros2_bag.mkdir(parents=True)

    commands = module.planned_commands(args, paths)

    labels = [label for label, _command in commands]
    assert "convert_ros1_to_ros2" not in labels
    assert "run_direct_visual" in labels
    assert "run_validation_bundle" in labels


def test_main_dry_run_writes_manifest(tmp_path, capsys):
    module = load_module()
    args = minimal_args(tmp_path)

    rc = module.main([
        "--sequence",
        args.sequence,
        "--profile",
        str(args.profile),
        "--ros2-bag",
        str(args.ros2_bag),
        "--reference",
        str(args.reference),
        "--benchmark-markdown",
        str(args.benchmark_markdown),
        "--out-dir",
        str(args.out_dir),
        "--dry-run",
    ])

    assert rc == 0
    assert (args.out_dir / "tank_dvl_heldout_inputs_manifest.md").exists()
    output = capsys.readouterr().out
    assert "run_tank_visual_direct_benchmark.py" in output
    assert "run_tank_dvl_validation_bundle.py" in output


def test_main_reports_missing_inputs_without_traceback(tmp_path, capsys):
    module = load_module()

    rc = module.main([
        "--sequence",
        "Medium",
        "--profile",
        str(tmp_path / "missing_profile.yaml"),
        "--ros2-bag",
        str(tmp_path / "missing_bag"),
        "--reference",
        str(tmp_path / "missing_ref.tum"),
        "--skip-bundle",
    ])

    assert rc == 2
    captured = capsys.readouterr()
    assert "missing required input" in captured.err
    assert "Traceback" not in captured.err

"""Tests for locate_tank_heldout_inputs.py."""

import importlib.util
import sys
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "locate_tank_heldout_inputs.py"
)
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("locate_tank_heldout_inputs", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_sequence_aliases_include_tank_medium_names():
    module = load_module()

    aliases = module.sequence_aliases("Medium")

    assert "medium" in aliases
    assert "halftankmedium" in aliases
    assert "whole_tank_medium" in aliases


def test_locate_inputs_finds_reference_bag_and_baseline(tmp_path):
    module = load_module()
    root = tmp_path / "data"
    ros2 = root / "HalfTankMedium_ros2"
    ros2.mkdir(parents=True)
    (ros2 / "metadata.yaml").write_text("rosbag2_bagfile_information:\n", encoding="utf-8")
    (ros2 / "HalfTankMedium.db3").write_bytes(b"db")
    (root / "HalfTankMedium_gt.tum").write_text("0 0 0 0 0 0 0 1\n", encoding="utf-8")
    (root / "Medium_visual_frontend.tum").write_text("0 0 0 0 0 0 0 1\n", encoding="utf-8")
    (root / "aqua_slam_medium_orb_odom.csv").write_text("%time,x\n", encoding="utf-8")
    (root / "Medium_aqua_slam_benchmark_row.md").write_text(
        "\n".join([
            "| Dataset | Sequence | System | Alignment | Samples | Matched s | Mean m | Median m | RMSE m | Max m | Note |",
            "|---------|----------|--------|-----------|--------:|----------:|-------:|---------:|-------:|------:|------|",
            "| Tank Dataset | Medium | AQUA-SLAM | SE(3) | 3 | 2.00 | 0.0000 | 0.0000 | 0.0200 | 0.0200 | smoke |",
        ])
        + "\n",
        encoding="utf-8",
    )

    report = module.locate_inputs("Medium", (root,), max_depth=4)

    assert module.first_path(report, "reference_tum") == root / "HalfTankMedium_gt.tum"
    assert module.first_path(report, "ros2_bag") == ros2
    assert module.first_path(report, "visual_tum") == root / "Medium_visual_frontend.tum"
    assert module.first_path(report, "aqua_slam_csv") == root / "aqua_slam_medium_orb_odom.csv"
    assert module.first_path(report, "baseline_row") == root / "Medium_aqua_slam_benchmark_row.md"
    assert "smoke-sized candidate" in report.by_role("baseline_row")[0].detail


def test_locate_inputs_finds_medium_download_archives(tmp_path):
    module = load_module()
    root = tmp_path / "downloads"
    root.mkdir()
    archive = root / "HalfTankMedium.tar.gz"
    archive.write_bytes(b"not really a tar")

    report = module.locate_inputs("Medium", (root,), max_depth=2)
    text = module.format_report(
        module.parse_args(["--sequence", "Medium", "--root", str(root)]),
        report,
    )

    assert module.first_path(report, "archive") == archive
    assert "Download Archive" in text
    assert "extract before locating inputs" in text


def test_locate_inputs_marks_short_matched_duration_baseline(tmp_path):
    module = load_module()
    root = tmp_path / "data"
    root.mkdir()
    (root / "Medium_aqua_slam_benchmark_row.md").write_text(
        "\n".join([
            "| Dataset | Sequence | System | Alignment | Samples | Matched s | Mean m | Median m | RMSE m | Max m | Note |",
            "|---------|----------|--------|-----------|--------:|----------:|-------:|---------:|-------:|------:|------|",
            "| Tank Dataset | Medium | AQUA-SLAM | SE(3) | 20 | 2.00 | 0.0000 | 0.0000 | 0.0200 | 0.0200 | short |",
        ])
        + "\n",
        encoding="utf-8",
    )

    report = module.locate_inputs("Medium", (root,), max_depth=2)

    assert "short matched duration" in report.by_role("baseline_row")[0].detail


def test_format_report_includes_blockers_and_link_commands(tmp_path):
    module = load_module()
    root = tmp_path / "data"
    root.mkdir()
    (root / "unrelated.txt").write_text("x\n", encoding="utf-8")
    args = module.parse_args([
        "--sequence",
        "Medium",
        "--root",
        str(root),
        "--profile",
        str(tmp_path / "profile.yaml"),
        "--benchmark-markdown",
        str(tmp_path / "bench.md"),
    ])

    report = module.locate_inputs(args.sequence, args.roots, args.max_depth)
    text = module.format_report(args, report)

    assert "Held-out validation is blocked" in text
    assert module.OFFICIAL_DOWNLOAD_URL in text
    assert "prepare_tank_dvl_heldout_inputs.py" in text


def test_iter_paths_skips_pytest_temp_dirs(tmp_path):
    module = load_module()
    good = tmp_path / "data"
    bad = tmp_path / "pytest-of-autoware"
    good.mkdir()
    bad.mkdir()
    (good / "Medium_gt.tum").write_text("0 0 0 0 0 0 0 1\n", encoding="utf-8")
    (bad / "Medium_gt.tum").write_text("0 0 0 0 0 0 0 1\n", encoding="utf-8")

    paths = tuple(module.iter_paths((tmp_path,), max_depth=3))

    assert good / "Medium_gt.tum" in paths
    assert bad / "Medium_gt.tum" not in paths


def test_main_writes_report(tmp_path):
    module = load_module()
    out = tmp_path / "report.md"

    rc = module.main([
        "--sequence",
        "Medium",
        "--root",
        str(tmp_path),
        "--out",
        str(out),
    ])

    assert rc == 0
    assert "# Tank Held-Out Input Locator" in out.read_text(encoding="utf-8")

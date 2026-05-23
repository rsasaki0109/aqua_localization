"""Tests for check_tank_aqua_slam_baseline_ready.py."""

import importlib.util
import subprocess
import sys
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "check_tank_aqua_slam_baseline_ready.py"
)
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("check_tank_aqua_slam_baseline_ready", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_tum(path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "\n".join([
            "0.000000000 0.000000000 0.000000000 0.000000000 0.0 0.0 0.0 1.0",
            "1.000000000 1.000000000 0.000000000 0.000000000 0.0 0.0 0.0 1.0",
            "2.000000000 2.000000000 0.000000000 0.000000000 0.0 0.0 0.0 1.0",
        ])
        + "\n",
        encoding="utf-8",
    )


def write_ros1_odom_csv(path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "\n".join([
            "%time,field.pose.pose.position.x,field.pose.pose.position.y,field.pose.pose.position.z,field.pose.pose.orientation.x,field.pose.pose.orientation.y,field.pose.pose.orientation.z,field.pose.pose.orientation.w",
            "0,0.0,0.0,0.0,0.0,0.0,0.0,1.0",
            "1.0,1.0,0.0,0.0,0.0,0.0,0.0,1.0",
            "2.0,2.0,0.0,0.0,0.0,0.0,0.0,1.0",
        ])
        + "\n",
        encoding="utf-8",
    )


def write_benchmark_row(path: Path, sequence="Medium", samples=20, matched_s=19.0):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "\n".join([
            "| Dataset | Sequence | System | Alignment | Samples | Matched s | Mean m | Median m | RMSE m | Max m | Note |",
            "|---------|----------|--------|-----------|--------:|----------:|-------:|---------:|-------:|------:|------|",
            (
                f"| Tank Dataset | {sequence} | AQUA-SLAM | SE(3) | "
                f"{samples} | {matched_s:.2f} | 0.0000 | 0.0000 | 0.0200 | 0.0200 | baseline |"
            ),
        ])
        + "\n",
        encoding="utf-8",
    )


def test_default_paths_follow_sequence_name():
    module = load_module()

    assert module.default_csv("Medium") == Path("/tmp/aqua_slam_medium_orb_odom.csv")
    assert module.default_reference("Structure Easy") == Path("/tmp/tank_structure_easy_gt.tum")
    assert module.default_baseline_dir("Medium") == Path("/tmp/aqua_slam_medium_baseline")


def test_ingest_ready_when_reference_and_csv_are_valid(tmp_path):
    module = load_module()
    reference = tmp_path / "ref.tum"
    csv = tmp_path / "aqua.csv"
    write_tum(reference)
    write_ros1_odom_csv(csv)

    args = module.parse_args([
        "--reference",
        str(reference),
        "--csv",
        str(csv),
        "--baseline-row",
        str(tmp_path / "missing_row.md"),
        "--benchmark-markdown",
        str(tmp_path / "missing_docs.md"),
        "--profile",
        str(tmp_path / "missing_profile.yaml"),
        "--bag",
        str(tmp_path / "missing_bag"),
        "--visual",
        str(tmp_path / "missing_visual.tum"),
    ])
    report = module.build_report(args)

    assert report.ingest_ready is True
    assert report.baseline_row_ready is False
    assert report.validation_ready is False
    assert report.csv.count == 3
    assert report.csv.duration_s == 2.0


def test_baseline_row_source_enables_gap_readiness(tmp_path):
    module = load_module()
    reference = tmp_path / "ref.tum"
    csv = tmp_path / "aqua.csv"
    row = tmp_path / "Medium_aqua_slam_benchmark_row.md"
    write_tum(reference)
    write_ros1_odom_csv(csv)
    write_benchmark_row(row)

    args = module.parse_args([
        "--reference",
        str(reference),
        "--csv",
        str(csv),
        "--baseline-row",
        str(row),
        "--benchmark-markdown",
        str(tmp_path / "missing_docs.md"),
    ])
    report = module.build_report(args)

    assert report.baseline_row_ready is True
    assert report.benchmark_sources[-1].matching_rows[0].sequence == "Medium"


def test_smoke_sized_baseline_row_does_not_enable_gap_readiness(tmp_path):
    module = load_module()
    row = tmp_path / "Medium_aqua_slam_benchmark_row.md"
    write_benchmark_row(row, samples=3, matched_s=2.0)

    args = module.parse_args([
        "--baseline-row",
        str(row),
        "--benchmark-markdown",
        str(tmp_path / "missing_docs.md"),
    ])
    report = module.build_report(args)
    text = module.format_report(report)

    assert report.baseline_row_ready is False
    assert len(report.benchmark_sources[-1].matching_rows) == 0
    assert len(report.benchmark_sources[-1].rejected_rows) == 1
    assert "3 samples below minimum 10" in text


def test_short_matched_duration_baseline_row_does_not_enable_gap_readiness(tmp_path):
    module = load_module()
    row = tmp_path / "Medium_aqua_slam_benchmark_row.md"
    write_benchmark_row(row, samples=20, matched_s=2.0)

    args = module.parse_args([
        "--baseline-row",
        str(row),
        "--benchmark-markdown",
        str(tmp_path / "missing_docs.md"),
    ])
    report = module.build_report(args)
    text = module.format_report(report)

    assert report.baseline_row_ready is False
    assert len(report.benchmark_sources[-1].matching_rows) == 0
    assert len(report.benchmark_sources[-1].rejected_rows) == 1
    assert "2.00 matched s below minimum 10.00" in text


def test_next_ingest_command_uses_tum_when_csv_is_missing(tmp_path):
    module = load_module()
    reference = tmp_path / "ref.tum"
    tum = tmp_path / "aqua.tum"
    write_tum(reference)
    write_tum(tum)

    args = module.parse_args([
        "--reference",
        str(reference),
        "--tum",
        str(tum),
        "--csv",
        str(tmp_path / "missing.csv"),
        "--baseline-row",
        str(tmp_path / "missing_row.md"),
        "--benchmark-markdown",
        str(tmp_path / "missing_docs.md"),
    ])
    report = module.build_report(args)
    text = module.format_report(report)

    assert report.ingest_ready is True
    assert f"--tum {tum}" in text
    assert f"--csv {tmp_path / 'missing.csv'}" not in text


def test_validation_ready_requires_profile_bag_visual_and_baseline_row(tmp_path):
    module = load_module()
    reference = tmp_path / "ref.tum"
    csv = tmp_path / "aqua.csv"
    visual = tmp_path / "visual.tum"
    row = tmp_path / "Medium_aqua_slam_benchmark_row.md"
    profile = tmp_path / "profile.yaml"
    bag = tmp_path / "bag"
    write_tum(reference)
    write_tum(visual)
    write_ros1_odom_csv(csv)
    write_benchmark_row(row)
    profile.write_text("name: profile\n", encoding="utf-8")
    bag.mkdir()

    args = module.parse_args([
        "--reference",
        str(reference),
        "--csv",
        str(csv),
        "--visual",
        str(visual),
        "--baseline-row",
        str(row),
        "--benchmark-markdown",
        str(tmp_path / "missing_docs.md"),
        "--profile",
        str(profile),
        "--bag",
        str(bag),
    ])
    report = module.build_report(args)
    text = module.format_report(report)

    assert report.validation_ready is True
    assert "Held-out validation bundle | PASS" in text
    assert "run_tank_dvl_validation_bundle.py" in text


def test_cli_returns_nonzero_and_writes_report_when_not_ready(tmp_path):
    out = tmp_path / "ready.md"

    proc = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            "--reference",
            str(tmp_path / "missing_ref.tum"),
            "--csv",
            str(tmp_path / "missing.csv"),
            "--baseline-row",
            str(tmp_path / "missing_row.md"),
            "--benchmark-markdown",
            str(tmp_path / "missing_docs.md"),
            "--out",
            str(out),
        ],
        text=True,
        capture_output=True,
    )

    assert proc.returncode == 1
    assert proc.stderr == ""
    report = out.read_text(encoding="utf-8")
    assert "Baseline ingest inputs | FAIL" in report
    assert "rostopic echo -p /AQUA_SLAM/orb_odom" in report

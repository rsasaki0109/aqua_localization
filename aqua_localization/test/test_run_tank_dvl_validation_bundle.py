"""Tests for run_tank_dvl_validation_bundle.py."""

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "run_tank_dvl_validation_bundle.py"
)
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("run_tank_dvl_validation_bundle", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_benchmark_row(path: Path, *, system="aqua_dvl_prior_visual", samples=20, matched_s=19.0):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "\n".join([
            "| Dataset | Sequence | System | Alignment | Samples | Matched s | Mean m | Median m | RMSE m | Max m | Note |",
            "|---------|----------|--------|-----------|--------:|----------:|-------:|---------:|-------:|------:|------|",
            (
                f"| Tank Dataset | Medium | {system} | SE(3) | {samples} | {matched_s:.2f} | "
                "0.0000 | 0.0000 | 0.0200 | 0.0200 | validation |"
            ),
        ])
        + "\n",
        encoding="utf-8",
    )


def coverage_args(tmp_path):
    return SimpleNamespace(
        dataset="Tank Dataset",
        sequence="Medium",
        target_system="aqua_dvl_prior_visual",
        min_target_samples=10,
        min_target_matched_s=10.0,
        out_dir=tmp_path / "bundle",
    )


def test_bundle_paths_are_scoped_under_output_dir(tmp_path):
    module = load_module()

    paths = module.bundle_paths(tmp_path / "bundle")

    assert paths.validation_summary == tmp_path / "bundle" / "validation" / "tank_dvl_prior_validation.md"
    assert paths.benchmark_row == tmp_path / "bundle" / "validation" / "tank_dvl_prior_benchmark_row.md"
    assert paths.gap_report == tmp_path / "bundle" / "benchmark_gap_report.md"
    assert paths.residual_csv == tmp_path / "bundle" / "tank_dvl_prior_residuals.csv"


def test_missing_inputs_reports_absent_paths(tmp_path):
    module = load_module()
    existing = tmp_path / "profile.yaml"
    existing.write_text("format_version: 1\n", encoding="utf-8")
    args = SimpleNamespace(
        profile=existing,
        bag=tmp_path / "missing_bag",
        reference=tmp_path / "missing_ref.tum",
        visual=tmp_path / "missing_visual.tum",
        benchmark_markdown=[tmp_path / "missing_benchmark.md"],
        skip_gap_report=False,
    )

    missing = module.missing_inputs(args)

    assert f"bag: {tmp_path / 'missing_bag'}" in missing
    assert f"reference: {tmp_path / 'missing_ref.tum'}" in missing
    assert f"visual: {tmp_path / 'missing_visual.tum'}" in missing
    assert f"benchmark_markdown[1]: {tmp_path / 'missing_benchmark.md'}" in missing


def test_make_validation_args_forwards_guards_and_gates(tmp_path):
    module = load_module()
    paths = module.bundle_paths(tmp_path / "bundle")
    args = SimpleNamespace(
        profile=tmp_path / "profile.yaml",
        sequence="Medium",
        bag=tmp_path / "bag",
        reference=tmp_path / "ref.tum",
        visual=tmp_path / "visual.tum",
        dataset="Tank Dataset",
        target_system="aqua_dvl_prior_visual",
        note="held-out candidate",
        allow_same_sequence=False,
        allow_profile_sequence_mismatch=False,
        max_corrected_rmse_m=0.0194,
        min_improvement_percent=80.0,
    )

    validation_args = module.make_validation_args(args, paths)

    assert validation_args.sequence == "Medium"
    assert validation_args.max_corrected_rmse_m == 0.0194
    assert validation_args.min_improvement_percent == 80.0
    assert validation_args.benchmark_row_out == paths.benchmark_row
    assert validation_args.system == "aqua_dvl_prior_visual"
    assert validation_args.note == "held-out candidate"


def test_target_row_coverage_accepts_sufficient_benchmark_row(tmp_path):
    module = load_module()
    paths = module.bundle_paths(tmp_path / "bundle")
    write_benchmark_row(paths.benchmark_row, samples=20, matched_s=19.0)

    failures = module.target_row_coverage_failures(coverage_args(tmp_path), paths)

    assert failures == []


def test_target_row_coverage_rejects_short_benchmark_row(tmp_path):
    module = load_module()
    paths = module.bundle_paths(tmp_path / "bundle")
    write_benchmark_row(paths.benchmark_row, samples=20, matched_s=2.0)

    failures = module.target_row_coverage_failures(coverage_args(tmp_path), paths)

    assert failures == [
        "aqua_dvl_prior_visual Medium: 2.00 matched s below minimum 10.00"
    ]


def test_target_row_coverage_rejects_smoke_sized_benchmark_row(tmp_path):
    module = load_module()
    paths = module.bundle_paths(tmp_path / "bundle")
    write_benchmark_row(paths.benchmark_row, samples=3, matched_s=19.0)

    failures = module.target_row_coverage_failures(coverage_args(tmp_path), paths)

    assert failures == [
        "aqua_dvl_prior_visual Medium: 3 samples below minimum 10"
    ]


def test_format_bundle_summary_reports_failures(tmp_path):
    module = load_module()
    paths = module.bundle_paths(tmp_path / "bundle")
    args = SimpleNamespace(
        sequence="Medium",
        profile=tmp_path / "profile.yaml",
    )
    result = module.BundleResult(
        validation_failures=["corrected RMSE too high"],
        gap_failures=["gap too high"],
        validation_summary="",
        gap_summary="",
        residual_summary="",
    )

    text = module.format_bundle_summary(args, paths, result)

    assert "Status: `FAIL`" in text
    assert "corrected RMSE too high" in text
    assert "gap too high" in text


def test_main_reports_missing_inputs_without_traceback(tmp_path, capsys):
    module = load_module()

    rc = module.main([
        "--profile",
        str(tmp_path / "profile.yaml"),
        "--sequence",
        "Medium",
        "--bag",
        str(tmp_path / "bag"),
        "--reference",
        str(tmp_path / "ref.tum"),
        "--visual",
        str(tmp_path / "visual.tum"),
        "--skip-gap-report",
    ])

    assert rc == 2
    captured = capsys.readouterr()
    assert "missing required input" in captured.err
    assert "Traceback" not in captured.err

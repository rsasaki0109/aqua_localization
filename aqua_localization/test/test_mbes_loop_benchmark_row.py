"""Tests for mbes_loop_benchmark_row.py."""

import csv
import importlib.util
import subprocess
import sys
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "mbes_loop_benchmark_row.py"


def load_module():
    spec = importlib.util.spec_from_file_location("mbes_loop_benchmark_row", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_status_csv(path: Path):
    fields = [
        "timestamp",
        "frame_id",
        "current_id",
        "candidate_id",
        "accepted",
        "converged",
        "fitness_score",
        "correction_translation_m",
        "correction_rotation_rad",
        "descriptor_centroid_distance_m",
        "descriptor_extent_ratio",
        "descriptor_point_count_ratio",
        "status",
    ]
    rows = [
        {
            "timestamp": "1.0",
            "frame_id": "map",
            "current_id": "10",
            "candidate_id": "3",
            "accepted": "1",
            "converged": "1",
            "fitness_score": "0.10",
            "correction_translation_m": "0.20",
            "correction_rotation_rad": "0.01",
            "descriptor_centroid_distance_m": "0.5",
            "descriptor_extent_ratio": "1.0",
            "descriptor_point_count_ratio": "0.9",
            "status": "accepted",
        },
        {
            "timestamp": "2.0",
            "frame_id": "map",
            "current_id": "11",
            "candidate_id": "4",
            "accepted": "0",
            "converged": "1",
            "fitness_score": "0.30",
            "correction_translation_m": "0.60",
            "correction_rotation_rad": "0.03",
            "descriptor_centroid_distance_m": "1.0",
            "descriptor_extent_ratio": "1.5",
            "descriptor_point_count_ratio": "0.7",
            "status": "fitness score exceeds gate",
        },
        {
            "timestamp": "3.0",
            "frame_id": "map",
            "current_id": "12",
            "candidate_id": str(2**32 - 1),
            "accepted": "0",
            "converged": "0",
            "fitness_score": "nan",
            "correction_translation_m": "nan",
            "correction_rotation_rad": "nan",
            "descriptor_centroid_distance_m": "nan",
            "descriptor_extent_ratio": "nan",
            "descriptor_point_count_ratio": "nan",
            "status": "no candidate submaps",
        },
    ]
    with path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def write_summary_md(path: Path, count: str = "4", chi2: str = "12.5"):
    path.write_text(
        "\n".join(
            [
                "# MBES Loop Closure Status Summary",
                "",
                "## Pose Graph Optimization",
                "",
                "- Count topic: `/aqua_pose_graph/optimization_count`",
                "- Count samples: 5",
                f"- Latest optimize count: {count}",
                "- Chi2 topic: `/aqua_pose_graph/optimization_chi2`",
                "- Chi2 samples: 5",
                f"- Latest active chi2: {chi2}",
                "",
            ]
        ),
        encoding="utf-8",
    )


def test_summarize_rows_counts_status_classes(tmp_path):
    module = load_module()
    path = tmp_path / "status.csv"
    write_status_csv(path)

    summary = module.summarize_rows(module.read_loop_status_csv(path))

    assert summary["samples"] == 3
    assert summary["accepted"] == 1
    assert summary["rejected"] == 1
    assert summary["no_candidate"] == 1
    assert summary["converged"] == 2
    assert summary["median_fitness"] == 0.2
    assert summary["p95_correction_m"] == 0.58


def test_format_row_matches_benchmark_table_shape(tmp_path):
    module = load_module()
    path = tmp_path / "status.csv"
    write_status_csv(path)

    class Args:
        dataset = "MBES-SLAM"
        sequence = "beach_pond"
        duration = 120.0
        note = "first replay | tuned"

    row = module.format_row(Args, module.summarize_rows(module.read_loop_status_csv(path)))

    assert row == (
        "| MBES-SLAM | `beach_pond` | 120 | 3 | 1 | 1 | 1 | 2 | "
        "0.2000 | 0.5800 | first replay \\| tuned |"
    )


def test_format_row_can_include_optimization_columns(tmp_path):
    module = load_module()
    path = tmp_path / "status.csv"
    write_status_csv(path)

    class Args:
        dataset = "MBES-SLAM"
        sequence = "beach_pond"
        duration = 120.0
        optimization_count = 4
        optimization_chi2 = 12.5
        note = "diagnostics"

    header = module.table_header(module.include_optimization_columns(Args))
    row = module.format_row(Args, module.summarize_rows(module.read_loop_status_csv(path)))

    assert "Optimize runs" in header
    assert "Latest chi2" in header
    assert row == (
        "| MBES-SLAM | `beach_pond` | 120 | 3 | 1 | 1 | 1 | 2 | "
        "0.2000 | 0.5800 | 4 | 12.5000 | diagnostics |"
    )


def test_read_optimization_summary_parses_exported_markdown(tmp_path):
    module = load_module()
    path = tmp_path / "summary.md"
    write_summary_md(path, count="7", chi2="31.25")

    summary = module.read_optimization_summary(path)

    assert summary.count == 7
    assert summary.chi2 == 31.25


def test_summary_values_fill_benchmark_row_columns(tmp_path):
    module = load_module()
    csv_path = tmp_path / "status.csv"
    summary_path = tmp_path / "summary.md"
    write_status_csv(csv_path)
    write_summary_md(summary_path, count="4", chi2="12.5")

    class Args:
        dataset = "MBES-SLAM"
        sequence = "beach_pond"
        duration = 120.0
        optimization_count = None
        optimization_chi2 = None
        note = "summary diagnostics"

    module.apply_summary_optimization(Args, module.read_optimization_summary(summary_path))
    row = module.format_row(
        Args,
        module.summarize_rows(module.read_loop_status_csv(csv_path)),
    )

    assert row == (
        "| MBES-SLAM | `beach_pond` | 120 | 3 | 1 | 1 | 1 | 2 | "
        "0.2000 | 0.5800 | 4 | 12.5000 | summary diagnostics |"
    )


def test_explicit_optimization_values_override_summary(tmp_path):
    module = load_module()
    path = tmp_path / "summary.md"
    write_summary_md(path, count="4", chi2="12.5")

    class Args:
        optimization_count = 9
        optimization_chi2 = 99.0

    module.apply_summary_optimization(Args, module.read_optimization_summary(path))

    assert Args.optimization_count == 9
    assert Args.optimization_chi2 == 99.0


def test_cli_prints_header_and_row(tmp_path):
    path = tmp_path / "status.csv"
    write_status_csv(path)

    proc = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            "--csv",
            str(path),
            "--dataset",
            "MBES-SLAM",
            "--sequence",
            "beach_pond",
            "--duration",
            "120",
            "--note",
            "first replay",
            "--header",
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    assert "| Dataset | Sequence | Duration s |" in proc.stdout
    assert "| MBES-SLAM | `beach_pond` | 120 | 3 | 1 | 1 | 1 | 2 |" in proc.stdout


def test_cli_reads_optimization_columns_from_summary(tmp_path):
    csv_path = tmp_path / "status.csv"
    summary_path = tmp_path / "summary.md"
    write_status_csv(csv_path)
    write_summary_md(summary_path, count="4", chi2="12.5")

    proc = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            "--csv",
            str(csv_path),
            "--summary",
            str(summary_path),
            "--dataset",
            "MBES-SLAM",
            "--sequence",
            "beach_pond",
            "--duration",
            "120",
            "--note",
            "summary diagnostics",
            "--header",
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    assert "Optimize runs" in proc.stdout
    assert (
        "| MBES-SLAM | `beach_pond` | 120 | 3 | 1 | 1 | 1 | 2 | "
        "0.2000 | 0.5800 | 4 | 12.5000 | summary diagnostics |"
    ) in proc.stdout


def test_cli_appends_to_output_file(tmp_path):
    csv_path = tmp_path / "status.csv"
    out = tmp_path / "rows.md"
    write_status_csv(csv_path)
    out.write_text("existing\n", encoding="utf-8")

    subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            "--csv",
            str(csv_path),
            "--dataset",
            "MBES-SLAM",
            "--sequence",
            "beach_pond",
            "--out",
            str(out),
            "--append",
        ],
        check=True,
    )

    text = out.read_text(encoding="utf-8")
    assert text.startswith("existing\n\n")
    assert "MBES-SLAM" in text

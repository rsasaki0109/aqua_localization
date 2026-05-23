"""Tests for summarize_tank_aqua_slam_baseline_todos.py."""

import importlib.util
import subprocess
import sys
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "summarize_tank_aqua_slam_baseline_todos.py"
)
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("summarize_tank_aqua_slam_baseline_todos", SCRIPT_PATH)
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


def write_benchmark_row(path: Path, samples: int = 20, matched_s: float = 19.0):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "\n".join([
            "| Dataset | Sequence | System | Alignment | Samples | Matched s | Mean m | Median m | RMSE m | Max m | Note |",
            "|---------|----------|--------|-----------|--------:|----------:|-------:|---------:|-------:|------:|------|",
            (
                "| Tank Dataset | Medium | AQUA-SLAM | SE(3) | "
                f"{samples} | {matched_s:.2f} | 0.0000 | 0.0000 | 0.0200 | 0.0200 | baseline |"
            ),
        ])
        + "\n",
        encoding="utf-8",
    )


def parse_args(module, tmp_path, extra=None):
    extra = extra or []
    return module.readiness.parse_args([
        "--reference",
        str(tmp_path / "ref.tum"),
        "--csv",
        str(tmp_path / "aqua.csv"),
        "--baseline-row",
        str(tmp_path / "row.md"),
        "--benchmark-markdown",
        str(tmp_path / "missing_docs.md"),
        "--profile",
        str(tmp_path / "profile.yaml"),
        "--bag",
        str(tmp_path / "bag"),
        "--visual",
        str(tmp_path / "visual.tum"),
        *extra,
    ])


def test_missing_inputs_next_action_records_aqua_slam_csv(tmp_path):
    module = load_module()

    report = module.readiness.build_report(parse_args(module, tmp_path))
    todos = module.build_todos(report)
    text = module.format_todos(report, todos)

    assert module.next_action(todos).title == "Reference TUM ready"
    assert "Record AQUA-SLAM odometry CSV" in text
    assert "rostopic echo -p /AQUA_SLAM/orb_odom" in text


def test_ingest_ready_next_action_generates_baseline_row(tmp_path):
    module = load_module()
    write_tum(tmp_path / "ref.tum")
    write_ros1_odom_csv(tmp_path / "aqua.csv")

    report = module.readiness.build_report(parse_args(module, tmp_path))
    todos = module.build_todos(report)
    text = module.format_todos(report, todos)

    assert module.next_action(todos).title == "Ingest AQUA-SLAM baseline row"
    assert "ingest_tank_aqua_slam_baseline.py" in text
    assert f"--csv {tmp_path / 'aqua.csv'}" in text


def test_ingest_command_uses_tum_when_csv_is_missing(tmp_path):
    module = load_module()
    reference = tmp_path / "ref.tum"
    tum = tmp_path / "aqua.tum"
    write_tum(reference)
    write_tum(tum)

    args = module.readiness.parse_args([
        "--reference",
        str(reference),
        "--csv",
        str(tmp_path / "missing.csv"),
        "--tum",
        str(tum),
        "--baseline-row",
        str(tmp_path / "row.md"),
        "--benchmark-markdown",
        str(tmp_path / "missing_docs.md"),
    ])
    report = module.readiness.build_report(args)
    text = module.format_todos(report, module.build_todos(report))

    assert f"--tum {tum}" in text
    assert f"--csv {tmp_path / 'missing.csv'}" not in text


def test_validation_ready_next_action_runs_bundle(tmp_path):
    module = load_module()
    write_tum(tmp_path / "ref.tum")
    write_tum(tmp_path / "visual.tum")
    write_ros1_odom_csv(tmp_path / "aqua.csv")
    write_benchmark_row(tmp_path / "row.md")
    (tmp_path / "profile.yaml").write_text("name: profile\n", encoding="utf-8")
    (tmp_path / "bag").mkdir()

    report = module.readiness.build_report(parse_args(module, tmp_path))
    todos = module.build_todos(report)
    text = module.format_todos(report, todos)

    assert report.validation_ready
    assert module.next_action(todos).title == "Run held-out DVL prior validation bundle"
    assert "run_tank_dvl_validation_bundle.py" in text
    assert "Ready to run validation bundle: **PASS**" in text


def test_smoke_sized_baseline_row_blocks_gap_check(tmp_path):
    module = load_module()
    write_tum(tmp_path / "ref.tum")
    write_ros1_odom_csv(tmp_path / "aqua.csv")
    write_benchmark_row(tmp_path / "row.md", samples=3, matched_s=2.0)

    report = module.readiness.build_report(parse_args(module, tmp_path))
    todos = module.build_todos(report)
    text = module.format_todos(report, todos)

    assert not report.baseline_row_ready
    assert module.next_action(todos).title == "AQUA-SLAM baseline row has enough samples"
    assert "require >= 10 samples" in text
    assert "Ready to run validation bundle: **FAIL**" in text


def test_cli_writes_todo_file_and_returns_nonzero_when_blocked(tmp_path):
    out = tmp_path / "todos.md"

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
    text = out.read_text(encoding="utf-8")
    assert "# Tank AQUA-SLAM Baseline TODOs" in text
    assert "Ready to run validation bundle: **FAIL**" in text

"""Tests for run_tank_aqua_slam_baseline_workflow.py."""

import importlib.util
import subprocess
import sys
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "run_tank_aqua_slam_baseline_workflow.py"
)
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("run_tank_aqua_slam_baseline_workflow", SCRIPT_PATH)
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


def write_benchmark_row(path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "\n".join([
            "| Dataset | Sequence | System | Alignment | Samples | Matched s | Mean m | Median m | RMSE m | Max m | Note |",
            "|---------|----------|--------|-----------|--------:|----------:|-------:|---------:|-------:|------:|------|",
            "| Tank Dataset | Medium | AQUA-SLAM | SE(3) | 3 | 2.00 | 0.0000 | 0.0000 | 0.0200 | 0.0200 | baseline |",
        ])
        + "\n",
        encoding="utf-8",
    )


def base_args(module, tmp_path, extra=None):
    extra = extra or []
    return module.parse_args([
        "--reference",
        str(tmp_path / "ref.tum"),
        "--csv",
        str(tmp_path / "aqua.csv"),
        "--baseline-dir",
        str(tmp_path / "baseline"),
        "--benchmark-markdown",
        str(tmp_path / "missing_docs.md"),
        "--profile",
        str(tmp_path / "profile.yaml"),
        "--bag",
        str(tmp_path / "bag"),
        "--visual",
        str(tmp_path / "visual.tum"),
        "--out-dir",
        str(tmp_path / "workflow"),
        *extra,
    ])


def test_workflow_blocks_and_writes_reports_when_inputs_missing(tmp_path):
    module = load_module()

    result = module.run_workflow(base_args(module, tmp_path))

    assert result.status == module.STATUS_BLOCKED
    assert (tmp_path / "workflow/readiness.md").exists()
    assert (tmp_path / "workflow/todos.md").exists()
    summary = (tmp_path / "workflow/workflow_summary.md").read_text(encoding="utf-8")
    assert "Status: `BLOCKED`" in summary
    assert "waiting for reference and AQUA-SLAM trajectory source" in summary


def test_workflow_auto_ingests_baseline_row_when_source_ready(tmp_path):
    module = load_module()
    write_tum(tmp_path / "ref.tum")
    write_ros1_odom_csv(tmp_path / "aqua.csv")

    result = module.run_workflow(base_args(module, tmp_path))

    assert result.status == module.STATUS_BLOCKED
    assert (tmp_path / "baseline/Medium_aqua_slam.tum").exists()
    row = (tmp_path / "baseline/Medium_aqua_slam_benchmark_row.md").read_text(encoding="utf-8")
    assert "| Tank Dataset | Medium | AQUA-SLAM | SE(3) | 3 | 2.00 | 0.0000" in row
    assert any(step.name == "Baseline ingest" and step.status == module.STATUS_PASS for step in result.steps)
    todos = (tmp_path / "workflow/todos.md").read_text(encoding="utf-8")
    assert "AQUA-SLAM baseline row ready" in todos


def test_workflow_dry_run_skips_validation_when_ready(tmp_path):
    module = load_module()
    write_tum(tmp_path / "ref.tum")
    write_tum(tmp_path / "visual.tum")
    write_ros1_odom_csv(tmp_path / "aqua.csv")
    write_benchmark_row(tmp_path / "baseline/Medium_aqua_slam_benchmark_row.md")
    (tmp_path / "profile.yaml").write_text("name: profile\n", encoding="utf-8")
    (tmp_path / "bag").mkdir()

    result = module.run_workflow(base_args(module, tmp_path, ["--dry-run"]))

    assert result.status == module.STATUS_BLOCKED
    assert any(
        step.name == "Validation bundle" and step.status == module.STATUS_SKIPPED
        for step in result.steps
    )
    assert "validation bundle was not executed" in (tmp_path / "workflow/workflow_summary.md").read_text(
        encoding="utf-8"
    )


def test_workflow_runs_validation_bundle_when_ready(tmp_path, monkeypatch):
    module = load_module()
    write_tum(tmp_path / "ref.tum")
    write_tum(tmp_path / "visual.tum")
    write_ros1_odom_csv(tmp_path / "aqua.csv")
    write_benchmark_row(tmp_path / "baseline/Medium_aqua_slam_benchmark_row.md")
    (tmp_path / "profile.yaml").write_text("name: profile\n", encoding="utf-8")
    (tmp_path / "bag").mkdir()
    called = {}

    def fake_run_bundle(bundle_args):
        called["out_dir"] = bundle_args.out_dir
        paths = module.validation_bundle.bundle_paths(bundle_args.out_dir)
        result = module.validation_bundle.BundleResult([], [], "validation", "gap", "residual")
        return paths, result, "# fake validation summary\n"

    monkeypatch.setattr(module.validation_bundle, "run_bundle", fake_run_bundle)

    result = module.run_workflow(base_args(module, tmp_path))

    assert result.status == module.STATUS_PASS
    assert called["out_dir"] == Path("/tmp/aqua_tank_dvl_prior_medium_validation_bundle")
    summary = (tmp_path / "workflow/workflow_summary.md").read_text(encoding="utf-8")
    assert "Status: `PASS`" in summary
    assert "# fake validation summary" in summary


def test_cli_returns_blocked_for_missing_inputs(tmp_path):
    proc = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            "--reference",
            str(tmp_path / "missing_ref.tum"),
            "--csv",
            str(tmp_path / "missing.csv"),
            "--baseline-dir",
            str(tmp_path / "baseline"),
            "--benchmark-markdown",
            str(tmp_path / "missing_docs.md"),
            "--out-dir",
            str(tmp_path / "workflow"),
        ],
        text=True,
        capture_output=True,
    )

    assert proc.returncode == 1
    assert "Status: `BLOCKED`" in proc.stdout
    assert proc.stderr == ""

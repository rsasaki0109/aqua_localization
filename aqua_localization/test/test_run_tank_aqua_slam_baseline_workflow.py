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


def write_benchmark_row(path: Path, samples: int = 3, matched_s: float = 2.0):
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


def write_ros2_bag_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)
    (path / "metadata.yaml").write_text("rosbag2_bagfile_information:\n", encoding="utf-8")
    (path / "Medium.db3").write_bytes(b"db")


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


def test_workflow_auto_locates_ready_inputs(tmp_path):
    module = load_module()
    data = tmp_path / "downloaded"
    write_tum(data / "HalfTankMedium_gt.tum")
    write_tum(data / "Medium_visual_frontend.tum")
    write_ros1_odom_csv(data / "aqua_slam_medium_orb_odom.csv")
    write_ros2_bag_dir(data / "HalfTankMedium_ros2")
    write_benchmark_row(data / "Medium_aqua_slam_benchmark_row.md", samples=20, matched_s=19.0)
    (tmp_path / "profile.yaml").write_text("name: profile\n", encoding="utf-8")

    args = module.parse_args([
        "--sequence",
        "Medium",
        "--profile",
        str(tmp_path / "profile.yaml"),
        "--baseline-dir",
        str(tmp_path / "baseline"),
        "--out-dir",
        str(tmp_path / "workflow"),
        "--auto-locate-inputs",
        "--locator-root",
        str(data),
        "--dry-run",
    ])
    result = module.run_workflow(args)

    assert result.status == module.STATUS_BLOCKED
    assert args.reference == data / "HalfTankMedium_gt.tum"
    assert args.bag == data / "HalfTankMedium_ros2"
    assert args.visual == data / "Medium_visual_frontend.tum"
    assert args.csv == data / "aqua_slam_medium_orb_odom.csv"
    assert data / "Medium_aqua_slam_benchmark_row.md" in args.benchmark_markdown
    assert any(step.name == "Input locator" and step.status == module.STATUS_PASS for step in result.steps)
    assert any(
        step.name == "Validation bundle" and step.status == module.STATUS_SKIPPED
        for step in result.steps
    )
    assert (tmp_path / "workflow/heldout_locator.md").exists()
    summary = (tmp_path / "workflow/workflow_summary.md").read_text(encoding="utf-8")
    assert "Input locator report" in summary
    assert "Held-out validation bundle inputs: `PASS`" in summary


def test_workflow_auto_locator_does_not_adopt_smoke_sized_baseline(tmp_path):
    module = load_module()
    data = tmp_path / "downloaded"
    write_benchmark_row(data / "Medium_aqua_slam_benchmark_row.md")

    args = module.parse_args([
        "--sequence",
        "Medium",
        "--baseline-dir",
        str(tmp_path / "baseline"),
        "--out-dir",
        str(tmp_path / "workflow"),
        "--auto-locate-inputs",
        "--locator-root",
        str(data),
        "--dry-run",
    ])
    result = module.run_workflow(args)

    assert result.status == module.STATUS_BLOCKED
    assert data / "Medium_aqua_slam_benchmark_row.md" not in args.benchmark_markdown
    assert any(step.name == "Input locator" and step.status == module.STATUS_BLOCKED for step in result.steps)


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

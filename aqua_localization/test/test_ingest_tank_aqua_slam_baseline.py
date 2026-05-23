"""Tests for ingest_tank_aqua_slam_baseline.py."""

import importlib.util
import sys
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "ingest_tank_aqua_slam_baseline.py"
)
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("ingest_tank_aqua_slam_baseline", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_tum(path: Path):
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


def test_default_paths_use_sequence_stem(tmp_path):
    module = load_module()

    paths = module.default_paths(tmp_path, "Medium Sequence")

    assert paths.estimate_tum == tmp_path / "Medium_Sequence_aqua_slam.tum"
    assert paths.benchmark_row == tmp_path / "Medium_Sequence_aqua_slam_benchmark_row.md"
    assert paths.manifest == tmp_path / "Medium_Sequence_aqua_slam_baseline.md"


def test_export_note_mentions_csv_conversion(tmp_path):
    module = load_module()
    args = module.parse_args([
        "--csv",
        str(tmp_path / "odom.csv"),
        "--reference",
        str(tmp_path / "ref.tum"),
        "--sequence",
        "Medium",
    ])

    assert "ros1_odometry_csv_to_tum.py" in module.export_note(args, used_csv=True)
    assert "external AQUA-SLAM TUM" in module.export_note(args, used_csv=False)


def test_main_converts_csv_and_writes_row_manifest(tmp_path, capsys):
    module = load_module()
    ref = tmp_path / "ref.tum"
    csv = tmp_path / "aqua.csv"
    out_dir = tmp_path / "out"
    write_tum(ref)
    write_ros1_odom_csv(csv)

    rc = module.main([
        "--csv",
        str(csv),
        "--reference",
        str(ref),
        "--sequence",
        "Medium",
        "--out-dir",
        str(out_dir),
        "--config",
        "underwater_orbslam3_blue_gx5_medium.yaml",
    ])

    assert rc == 0
    assert (out_dir / "Medium_aqua_slam.tum").exists()
    row = (out_dir / "Medium_aqua_slam_benchmark_row.md").read_text(encoding="utf-8")
    assert "| Tank Dataset | Medium | AQUA-SLAM | SE(3) | 3 | 2.00 | 0.0000" in row
    manifest = (out_dir / "Medium_aqua_slam_baseline.md").read_text(encoding="utf-8")
    assert "# External TUM Result" in manifest
    assert "underwater_orbslam3_blue_gx5_medium.yaml" in manifest
    assert "Tank AQUA-SLAM Baseline Ingestion" in capsys.readouterr().out


def test_main_can_append_row_to_markdown(tmp_path):
    module = load_module()
    ref = tmp_path / "ref.tum"
    est = tmp_path / "est.tum"
    append_to = tmp_path / "bench.md"
    write_tum(ref)
    write_tum(est)
    append_to.write_text("# Bench\n", encoding="utf-8")

    rc = module.main([
        "--tum",
        str(est),
        "--reference",
        str(ref),
        "--sequence",
        "Medium",
        "--out-dir",
        str(tmp_path / "out"),
        "--append-to",
        str(append_to),
    ])

    assert rc == 0
    assert "AQUA-SLAM" in append_to.read_text(encoding="utf-8")


def test_main_reports_missing_inputs_without_traceback(tmp_path, capsys):
    module = load_module()

    rc = module.main([
        "--csv",
        str(tmp_path / "missing.csv"),
        "--reference",
        str(tmp_path / "missing_ref.tum"),
        "--sequence",
        "Medium",
    ])

    assert rc == 2
    captured = capsys.readouterr()
    assert "reference not found" in captured.err
    assert "Traceback" not in captured.err

"""Tests for ingest_external_tum_result.py."""

import importlib.util
import subprocess
import sys
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "ingest_external_tum_result.py"
)
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("ingest_external_tum_result", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_tum(path: Path, rows):
    with path.open("w", encoding="utf-8") as fp:
        for row in rows:
            fp.write(" ".join(f"{v:.9f}" for v in row) + "\n")


def make_rows():
    return [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        [1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        [2.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    ]


def test_build_note_includes_external_provenance():
    module = load_module()
    args = module.parse_args([
        "--reference",
        "/tmp/ref.tum",
        "--estimate",
        "/tmp/est.tum",
        "--dataset",
        "Tank Dataset",
        "--sequence",
        "short_test",
        "--system",
        "AQUA-SLAM",
        "--config",
        "blue_gx5_short.yaml",
        "--commit",
        "abc1234",
        "--runtime",
        "docker",
        "--export",
        "ros1_odometry_csv_to_tum.py",
        "--note",
        "external baseline",
    ])

    note = module.build_note(args)

    assert "external baseline" in note
    assert "config=blue_gx5_short.yaml" in note
    assert "commit=abc1234" in note
    assert "runtime=docker" in note
    assert "export=ros1_odometry_csv_to_tum.py" in note


def test_cli_emits_aqua_slam_benchmark_row(tmp_path):
    ref = tmp_path / "ref.tum"
    est = tmp_path / "est.tum"
    write_tum(ref, make_rows())
    write_tum(est, make_rows())

    proc = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            "--reference",
            str(ref),
            "--estimate",
            str(est),
            "--dataset",
            "Tank Dataset",
            "--sequence",
            "short_test",
            "--system",
            "AQUA-SLAM",
            "--config",
            "underwater_orbslam3_blue_gx5_short.yaml",
            "--commit",
            "abc1234",
            "--header",
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    assert "| Dataset | Sequence | System |" in proc.stdout
    assert "| Tank Dataset | short_test | AQUA-SLAM | SE(3) | 3 | 2.00 | 0.0000" in proc.stdout
    assert "config=underwater_orbslam3_blue_gx5_short.yaml" in proc.stdout
    assert "commit=abc1234" in proc.stdout


def test_manifest_output_writes_file(tmp_path):
    ref = tmp_path / "ref.tum"
    est = tmp_path / "est.tum"
    out = tmp_path / "external_result.md"
    write_tum(ref, make_rows())
    write_tum(est, make_rows())

    subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            "--reference",
            str(ref),
            "--estimate",
            str(est),
            "--dataset",
            "Tank Dataset",
            "--sequence",
            "Structure_Easy",
            "--system",
            "AQUA-SLAM",
            "--manifest",
            "--out",
            str(out),
        ],
        check=True,
    )

    text = out.read_text(encoding="utf-8")
    assert "# External TUM Result" in text
    assert "## Benchmark Row" in text
    assert "Structure_Easy" in text
    assert "AQUA-SLAM" in text

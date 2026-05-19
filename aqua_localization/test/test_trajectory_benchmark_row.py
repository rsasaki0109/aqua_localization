"""Pure Python tests for trajectory_benchmark_row.py."""

import importlib.util
import subprocess
import sys
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "trajectory_benchmark_row.py"


def load_module():
    spec = importlib.util.spec_from_file_location("trajectory_benchmark_row", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_tum(path: Path, rows):
    with path.open("w", encoding="utf-8") as fp:
        for row in rows:
            fp.write(" ".join(f"{v:.9f}" for v in row) + "\n")


def test_format_row_escapes_cells():
    module = load_module()

    class Args:
        dataset = "Tank | Dataset"
        sequence = "short_test"
        system = "aqua_localization"
        note = "same | segment"

    stats = {
        "count": 10,
        "matched_seconds": 2.5,
        "mean": 0.1,
        "median": 0.08,
        "rmse": 0.12,
        "max": 0.4,
        "alignment": {"applied": True, "with_scale": False},
    }

    row = module.format_row(Args, stats)
    assert "Tank \\| Dataset" in row
    assert "same \\| segment" in row
    assert "SE(3)" in row


def test_cli_prints_header_and_row(tmp_path):
    rows = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        [1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        [2.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    ]
    ref = tmp_path / "ref.tum"
    est = tmp_path / "est.tum"
    write_tum(ref, rows)
    write_tum(est, rows)

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
            "aqua_localization",
            "--header",
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    assert "| Dataset | Sequence | System |" in proc.stdout
    assert "| Tank Dataset | short_test | aqua_localization | SE(3) | 3 | 2.00 | 0.0000" in proc.stdout


def test_cli_writes_output_file(tmp_path):
    rows = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        [1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    ]
    ref = tmp_path / "ref.tum"
    est = tmp_path / "est.tum"
    out = tmp_path / "table.md"
    write_tum(ref, rows)
    write_tum(est, rows)

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
            "--out",
            str(out),
            "--header",
        ],
        check=True,
    )

    text = out.read_text(encoding="utf-8")
    assert "Structure_Easy" in text
    assert "AQUA-SLAM" in text

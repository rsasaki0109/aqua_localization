"""Tests for summarize_mbes_candidate_descriptor_weight_sweep.py."""

import importlib.util
from pathlib import Path
import sys


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "summarize_mbes_candidate_descriptor_weight_sweep.py"
)
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location(
        "summarize_mbes_candidate_descriptor_weight_sweep",
        SCRIPT_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_status_csv(out_dir: Path, *, accepted: int, rejected: int):
    rows = [
        "candidate_id,accepted,converged,fitness_score,correction_translation_m,status",
    ]
    for index in range(accepted):
        rows.append(f"{index},true,true,0.20,0.30,accepted")
    for index in range(rejected):
        rows.append(
            f"{index + accepted},false,true,0.80,0.70,fitness score exceeds gate"
        )
    rows.append("4294967295,false,false,nan,nan,no candidate submaps")
    (out_dir / "mbes_beach_pond_loop_status.csv").write_text(
        "\n".join(rows) + "\n",
        encoding="utf-8",
    )


def write_metrics(
    out_dir: Path,
    *,
    input_rmse: float,
    graph_rmse: float,
    graph_median: float,
):
    (out_dir / "mbes_beach_pond_loop_trajectory_metrics.md").write_text(
        "\n".join([
            "# MBES Loop Closure Trajectory Metrics",
            "",
            "| Estimate | Matched samples | Matched s | Mean m | Median m | RMSE m | Max m |",
            "|----------|----------------:|----------:|-------:|---------:|-------:|------:|",
            f"| Input odometry | 10 | 9.00 | 1.0 | 0.8 | {input_rmse:.4f} | 2.0 |",
            f"| Pose graph path | 8 | 7.00 | 0.9 | {graph_median:.4f} | {graph_rmse:.4f} | 1.8 |",
            "",
        ]),
        encoding="utf-8",
    )


def make_case(tmp_path: Path, label: str, *, input_rmse: float, graph_rmse: float) -> Path:
    out_dir = tmp_path / label
    out_dir.mkdir()
    write_status_csv(out_dir, accepted=2, rejected=1)
    write_metrics(out_dir, input_rmse=input_rmse, graph_rmse=graph_rmse, graph_median=0.7)
    return out_dir


def test_parse_case_requires_weight_and_output_dir():
    module = load_module()

    case = module.parse_case("0.5:/tmp/out")

    assert case.weight == 0.5
    assert case.out_dir == Path("/tmp/out")


def test_format_markdown_marks_baseline_best_and_delta(tmp_path):
    module = load_module()
    baseline_dir = make_case(tmp_path, "weight_0", input_rmse=2.0, graph_rmse=1.6)
    tuned_dir = make_case(tmp_path, "weight_0p5", input_rmse=2.0, graph_rmse=1.1)
    args = module.parse_args([
        "--case", f"0.0:{baseline_dir}",
        "--case", f"0.5:{tuned_dir}",
    ])

    results = [
        module.load_case(module.CaseSpec(0.0, baseline_dir)),
        module.load_case(module.CaseSpec(0.5, tuned_dir)),
    ]
    text = module.format_markdown(results, args)

    assert "# MBES Candidate Descriptor Weight Sweep" in text
    assert "| 0.0000 | baseline | 2.0000 | 1.6000 | +0.4000 | 0.0000 |" in text
    assert "| 0.5000 | best | 2.0000 | 1.1000 | +0.9000 | +0.5000 |" in text
    assert "Best pose-graph RMSE: `1.1000` m" in text


def test_main_writes_markdown_and_csv(tmp_path):
    module = load_module()
    baseline_dir = make_case(tmp_path, "weight_0", input_rmse=2.0, graph_rmse=1.6)
    tuned_dir = make_case(tmp_path, "weight_1", input_rmse=2.0, graph_rmse=1.4)
    out = tmp_path / "summary.md"
    csv_out = tmp_path / "summary.csv"

    rc = module.main([
        "--case", f"0.0:{baseline_dir}",
        "--case", f"1.0:{tuned_dir}",
        "--out", str(out),
        "--csv-out", str(csv_out),
    ])

    assert rc == 0
    assert "MBES Candidate Descriptor Weight Sweep" in out.read_text(encoding="utf-8")
    csv_text = csv_out.read_text(encoding="utf-8")
    assert "descriptor_weight,input_rmse_m" in csv_text
    assert "1.0,2.0,9.0,1.4" in csv_text

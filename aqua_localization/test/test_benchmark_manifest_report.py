"""Tests for benchmark_manifest_report.py."""

import importlib.util
import json
import subprocess
import sys
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "benchmark_manifest_report.py"


def load_module():
    spec = importlib.util.spec_from_file_location("benchmark_manifest_report", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sample_manifest() -> dict:
    return {
        "schema": "aqua_localization.real_bag_evaluation.v1",
        "cases": [
            {
                "id": "tank-short-test",
                "dataset": "Tank Dataset",
                "sequence": "short_test",
                "status": "measured",
                "comparison_group": "visual-dvl-imu SLAM",
                "target_system": "aqua_localization+visual",
                "baselines": ["AQUA-SLAM"],
                "inputs": ["stereo", "IMU", "DVL"],
                "reference": "AprilTag GT",
                "metrics": ["translation APE RMSE", "visual coverage"],
                "artifacts": ["docs/benchmarks/tank_aqua_slam.md"],
                "command": "ros2 run aqua_localization run_tank_visual_fusion_benchmark.py",
                "next_step": "Narrow the RMSE gap.",
                "fairness_notes": ["Use the same sequence window."],
            },
            {
                "id": "aqualoc-harbor",
                "dataset": "AQUALOC",
                "sequence": "harbor_07",
                "status": "planned",
                "comparison_group": "future visual localization",
                "target_system": "aqua_localization visual frontend",
                "baselines": ["RTAB-Map"],
                "inputs": ["stereo"],
                "reference": "TBD",
                "metrics": ["tracking dropout count"],
                "artifacts": ["tracking status CSV"],
                "command": "",
                "next_step": "Confirm calibration.",
                "fairness_notes": ["Configure visual inputs before comparing."],
            },
        ],
    }


def test_load_manifest_and_filter_cases(tmp_path):
    module = load_module()
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(sample_manifest()), encoding="utf-8")

    cases = module.load_manifest(path)
    measured = module.filter_cases(cases, {"measured"})

    assert len(cases) == 2
    assert len(measured) == 1
    assert measured[0].case_id == "tank-short-test"
    assert measured[0].baselines == ("AQUA-SLAM",)


def test_format_report_includes_summary_and_details(tmp_path):
    module = load_module()
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(sample_manifest()), encoding="utf-8")

    report = module.format_report(module.load_manifest(path), source=path)

    assert "# Real-Bag Evaluation Run Sheet" in report
    assert "| Tank Dataset `short_test` | measured | aqua_localization+visual | AQUA-SLAM |" in report
    assert "```bash\nros2 run aqua_localization run_tank_visual_fusion_benchmark.py\n```" in report
    assert "_No replay command is pinned yet._" in report


def test_check_ready_fails_when_measured_command_missing(tmp_path):
    module = load_module()
    manifest = sample_manifest()
    manifest["cases"][0]["command"] = ""
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(manifest), encoding="utf-8")

    failures = module.readiness_failures(module.load_manifest(path))

    assert failures == ["tank-short-test: measured/ready case has no command"]


def test_cli_writes_filtered_report(tmp_path):
    manifest = tmp_path / "manifest.json"
    out = tmp_path / "report.md"
    manifest.write_text(json.dumps(sample_manifest()), encoding="utf-8")

    subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            str(manifest),
            "--status",
            "planned",
            "--out",
            str(out),
            "--check-ready",
        ],
        check=True,
    )

    text = out.read_text(encoding="utf-8")
    assert "AQUALOC `harbor_07`" in text
    assert "Tank Dataset" not in text

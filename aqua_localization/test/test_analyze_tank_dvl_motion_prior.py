"""CLI/report tests for analyze_tank_dvl_motion_prior.py."""

import importlib.util
import math
import sys
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "analyze_tank_dvl_motion_prior.py"
)
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_cli_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("analyze_tank_dvl_motion_prior", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_parse_args_defaults_keep_ros_topics_localized(tmp_path):
    module = load_cli_module()

    args = module.parse_args([
        "--bag",
        str(tmp_path / "bag"),
        "--reference",
        str(tmp_path / "ref.tum"),
        "--visual",
        str(tmp_path / "visual.tum"),
        "--out",
        str(tmp_path / "out.md"),
        "--csv",
        str(tmp_path / "steps.csv"),
    ])

    assert args.dvl_topic == "/dvl/twist"
    assert args.imu_topic == "/imu/data"
    assert args.mode == "gt_yaw"


def test_format_markdown_reports_dvl_summary(tmp_path):
    module = load_cli_module()
    from tank_dvl_prior_core import DvlPriorStep

    args = module.parse_args([
        "--bag",
        str(tmp_path / "bag"),
        "--reference",
        str(tmp_path / "ref.tum"),
        "--visual",
        str(tmp_path / "visual.tum"),
        "--out",
        str(tmp_path / "out.md"),
        "--csv",
        str(tmp_path / "steps.csv"),
    ])
    step = DvlPriorStep(
        start_stamp_s=0.0,
        end_stamp_s=1.0,
        offset_s=1.0,
        dt_s=1.0,
        dvl_step_m=1.0,
        reference_step_m=1.0,
        length_ratio=1.0,
        direction_cosine=1.0,
        heading_error_deg=0.0,
        dvl_cumulative_m=1.0,
        reference_cumulative_m=1.0,
        dvl_samples=2,
        covered=True,
        score=0.0,
    )

    text = module.format_markdown(args, [step], dvl_count=3)

    assert "Tank DVL Motion Prior Analysis" in text
    assert "DVL/reference cumulative ratio: 1" in text
    assert "DVL step direction is broadly aligned" in text


def test_format_markdown_handles_uncovered_steps(tmp_path):
    module = load_cli_module()
    from tank_dvl_prior_core import DvlPriorStep

    args = module.parse_args([
        "--bag",
        str(tmp_path / "bag"),
        "--reference",
        str(tmp_path / "ref.tum"),
        "--visual",
        str(tmp_path / "visual.tum"),
        "--out",
        str(tmp_path / "out.md"),
        "--csv",
        str(tmp_path / "steps.csv"),
    ])
    step = DvlPriorStep(
        start_stamp_s=0.0,
        end_stamp_s=1.0,
        offset_s=1.0,
        dt_s=1.0,
        dvl_step_m=0.0,
        reference_step_m=1.0,
        length_ratio=math.nan,
        direction_cosine=math.nan,
        heading_error_deg=math.nan,
        dvl_cumulative_m=0.0,
        reference_cumulative_m=1.0,
        dvl_samples=0,
        covered=False,
        score=math.inf,
    )

    text = module.format_markdown(args, [step], dvl_count=3)

    assert "Covered steps: 0 (0.0%)" in text
    assert "n/a" in text

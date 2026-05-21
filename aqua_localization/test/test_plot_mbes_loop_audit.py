"""Tests for MBES loop audit plan-view plotting."""

import importlib.util
import sys
from pathlib import Path

import pytest


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "plot_mbes_loop_audit.py"


def load_module():
    pytest.importorskip("matplotlib")
    scripts_dir = str(SCRIPT_PATH.parent)
    if scripts_dir not in sys.path:
        sys.path.insert(0, scripts_dir)
    spec = importlib.util.spec_from_file_location("plot_mbes_loop_audit", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_render_audit_plot_writes_png(tmp_path):
    module = load_module()
    marker_helpers = module.marker_helpers
    keyframes = {
        1: marker_helpers.KeyframePose(1, 0.0, 0.0, 0.0),
        2: marker_helpers.KeyframePose(2, 1.0, 0.0, 0.0),
        3: marker_helpers.KeyframePose(3, 2.0, 1.0, 0.0),
    }
    specs = [
        marker_helpers.MarkerSpec(
            rank=1,
            priority="high",
            candidate_id=1,
            current_id=3,
            candidate_xyz=(0.0, 0.0, 0.0),
            current_xyz=(2.0, 1.0, 0.0),
            label_xyz=(1.0, 0.5, 1.0),
            label="#1 high 1->3 fit=0.1 dt=0.2",
            flags=("rotation near gate",),
        )
    ]
    out = tmp_path / "audit.png"

    module.render_audit_plot(keyframes, specs, out, title="audit", max_labels=1)

    assert out.is_file()
    assert out.stat().st_size > 1000


def test_parse_args_defaults_to_all_tuned_accepted_loops(tmp_path):
    module = load_module()

    args = module.parse_args([
        "--bag",
        str(tmp_path / "bag"),
        "--csv",
        str(tmp_path / "status.csv"),
        "--out",
        str(tmp_path / "plot.png"),
    ])

    assert args.max_markers == 100
    assert args.max_labels == 12
    assert args.keyframe_topic == "/aqua_pose_graph/keyframe"

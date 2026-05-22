"""Tests for simulate_visual_motion_prior.py."""

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "simulate_visual_motion_prior.py"
)
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("simulate_visual_motion_prior", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_tum(path: Path, rows):
    with path.open("w", encoding="utf-8") as fp:
        for row in rows:
            fp.write(" ".join(f"{value:.9f}" for value in row) + "\n")


def tum_row(stamp_s: float, xyz):
    return [stamp_s, xyz[0], xyz[1], xyz[2], 0.0, 0.0, 0.0, 1.0]


def test_simulate_prior_replaces_bad_direction_step():
    module = load_module()
    times = np.array([0.0, 1.0, 2.0])
    visual = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
    ])
    prior = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [2.0, 0.0, 0.0],
    ])

    corrected, rows = module.simulate_prior(
        times,
        visual,
        prior,
        mode="replace-outliers",
        blend_alpha=0.5,
        min_reference_step_m=0.0,
        min_length_ratio=0.5,
        max_length_ratio=1.5,
        min_direction_cosine=0.5,
    )

    np.testing.assert_allclose(corrected, prior)
    assert rows[0].used_prior is False
    assert rows[1].used_prior is True
    assert rows[1].reason == "direction mismatch"


def test_blend_all_mixes_every_step():
    module = load_module()
    times = np.array([0.0, 1.0])
    visual = np.array([[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
    prior = np.array([[0.0, 0.0, 0.0], [0.0, 2.0, 0.0]])

    corrected, rows = module.simulate_prior(
        times,
        visual,
        prior,
        mode="blend-all",
        blend_alpha=0.25,
        min_reference_step_m=0.0,
        min_length_ratio=0.5,
        max_length_ratio=1.5,
        min_direction_cosine=0.0,
    )

    np.testing.assert_allclose(corrected[1], [1.5, 0.5, 0.0])
    assert rows[0].used_prior is True


def test_run_simulation_writes_corrected_trajectory(tmp_path):
    module = load_module()
    ref = tmp_path / "ref.tum"
    est = tmp_path / "est.tum"
    rows_ref = [tum_row(float(i), [float(i), 0.0, 0.0]) for i in range(5)]
    rows_est = [
        tum_row(0.0, [0.0, 0.0, 0.0]),
        tum_row(1.0, [1.0, 0.0, 0.0]),
        tum_row(2.0, [1.0, 1.0, 0.0]),
        tum_row(3.0, [2.0, 1.0, 0.0]),
        tum_row(4.0, [3.0, 1.0, 0.0]),
    ]
    write_tum(ref, rows_ref)
    write_tum(est, rows_est)
    args = module.parse_args([
        str(ref),
        str(est),
        "--out-dir",
        str(tmp_path / "out"),
        "--min-direction-cosine",
        "0.5",
        "--min-length-ratio",
        "0.5",
        "--max-length-ratio",
        "1.5",
    ])
    args.summary_out = args.out_dir / "summary.md"
    args.csv_out = args.out_dir / "steps.csv"
    args.corrected_out = args.out_dir / "corrected.tum"

    result, rows = module.run_simulation(args)

    assert result.corrected_tum.exists()
    assert result.prior_steps > 0
    assert len(rows) == 4


def test_main_writes_outputs(tmp_path, capsys):
    module = load_module()
    ref = tmp_path / "ref.tum"
    est = tmp_path / "est.tum"
    write_tum(ref, [tum_row(float(i), [float(i), 0.0, 0.0]) for i in range(5)])
    write_tum(est, [tum_row(float(i), [float(i), 0.2 * float(i), 0.0]) for i in range(5)])

    rc = module.main([
        str(ref),
        str(est),
        "--out-dir",
        str(tmp_path / "out"),
        "--mode",
        "blend-all",
        "--blend-alpha",
        "0.5",
    ])

    assert rc == 0
    assert (tmp_path / "out" / "visual_motion_prior_sim.md").exists()
    assert (tmp_path / "out" / "visual_motion_prior_steps.csv").exists()
    assert "Visual Motion Prior Simulation" in capsys.readouterr().out


def test_invalid_thresholds_raise():
    module = load_module()

    with pytest.raises(ValueError, match="max_length_ratio"):
        module.simulate_prior(
            np.array([0.0, 1.0]),
            np.zeros((2, 3)),
            np.zeros((2, 3)),
            mode="replace-outliers",
            blend_alpha=0.5,
            min_reference_step_m=0.0,
            min_length_ratio=2.0,
            max_length_ratio=1.0,
            min_direction_cosine=0.0,
        )

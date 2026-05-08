"""Pure-function tests for the scalar->pressure conversion logic.

Importing the script tries to load `rosbags`; if it is unavailable in the test
environment we skip. The conversion math itself is tested without any bag I/O.
"""

import importlib.util
import math
from pathlib import Path

import pytest


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "convert_scalar_pressure_bag.py"
)


def load_module():
    pytest.importorskip("rosbags.rosbag2")
    spec = importlib.util.spec_from_file_location("convert_scalar_pressure_bag", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_pressure_pa_mode_passes_through_with_offset():
    module = load_module()
    pa = module.scalar_to_pressure_pa(99000.0, mode="pressure_pa", pressure_offset_pa=2325.0)
    assert pa == pytest.approx(101325.0)


def test_depth_mode_uses_hydrostatic_law():
    module = load_module()
    pa = module.scalar_to_pressure_pa(
        2.0,
        mode="depth_m",
        reference_pressure_pa=101325.0,
        water_density_kg_m3=1000.0,
        gravity_mps2=9.81,
    )
    # 2 m of fresh water above a sensor adds rho*g*h = 1000*9.81*2 = 19620 Pa.
    assert pa == pytest.approx(101325.0 + 19620.0, rel=1e-9)


def test_ntnu_barometer_mode_matches_dataset_card_formula():
    module = load_module()
    # depth_m = -((scalar - offset) / scale)
    pa = module.scalar_to_pressure_pa(
        scalar=4500.0,
        mode="ntnu_barometer",
        barometer_pressure_offset=4000.0,
        barometer_pressure_scale=100.0,  # so depth_m = -(500/100) = -5 (above water)
        water_density_kg_m3=1025.0,
        gravity_mps2=9.80665,
        reference_pressure_pa=101325.0,
        min_depth_m=-10.0,
        max_depth_m=200.0,
    )
    expected_depth = -5.0
    expected_pa = 101325.0 + 1025.0 * 9.80665 * expected_depth
    assert pa == pytest.approx(expected_pa, rel=1e-9)


def test_ntnu_barometer_zero_scale_returns_nan():
    module = load_module()
    pa = module.scalar_to_pressure_pa(
        scalar=4500.0,
        mode="ntnu_barometer",
        barometer_pressure_offset=4000.0,
        barometer_pressure_scale=0.0,
    )
    assert math.isnan(pa)


def test_non_finite_scalar_returns_nan():
    module = load_module()
    assert math.isnan(module.scalar_to_pressure_pa(float("nan"), mode="pressure_pa"))
    assert math.isnan(module.scalar_to_pressure_pa(float("inf"), mode="depth_m"))


def test_depth_outside_window_is_rejected():
    module = load_module()
    pa = module.scalar_to_pressure_pa(
        scalar=10000.0,
        mode="depth_m",
        min_depth_m=-1.0,
        max_depth_m=200.0,
    )
    assert math.isnan(pa)


def test_unknown_mode_raises():
    module = load_module()
    with pytest.raises(ValueError):
        module.scalar_to_pressure_pa(1.0, mode="not_a_mode")

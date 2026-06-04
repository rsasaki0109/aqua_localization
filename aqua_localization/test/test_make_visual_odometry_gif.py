"""Pure helper tests for make_visual_odometry_gif.py."""

import importlib.util
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "make_visual_odometry_gif.py"


def load_module():
    spec = importlib.util.spec_from_file_location("make_visual_odometry_gif", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _texture(size=256):
    # Smooth random texture with trackable gradients (good for Shi-Tomasi + LK).
    rng = np.random.default_rng(0)
    noise = rng.integers(0, 256, (size, size), dtype=np.uint8)
    return cv2.GaussianBlur(noise, (0, 0), 3)


def _shifted_frames(base, sx, sy, count):
    # Shift content by (sx, sy) per frame; +sx moves content right, +sy down.
    return [np.roll(np.roll(base, sy * i, axis=0), sx * i, axis=1) for i in range(count)]


def test_frame_translation_recovers_known_shift():
    module = load_module()
    base = _texture()
    prev, cur = _shifted_frames(base, 3, -2, 2)
    flow = module.frame_translation(prev, cur, module.FlowParams())
    assert flow is not None
    np.testing.assert_allclose(flow, [3.0, -2.0], atol=1.0)


def test_frame_translation_returns_none_without_features():
    module = load_module()
    blank = np.zeros((128, 128), dtype=np.uint8)
    assert module.frame_translation(blank, blank, module.FlowParams()) is None


def test_integrate_camera_path_opposes_image_flow():
    module = load_module()
    base = _texture()
    frames = _shifted_frames(base, 3, -2, 6)
    path = module.integrate_camera_path(frames, module.FlowParams())
    assert path.shape == (6, 2)
    np.testing.assert_allclose(path[0], [0.0, 0.0])
    # Camera moves opposite to the image flow in x, with image y down: after five
    # steps of (+3, -2) image flow the camera path is (-15, -10), scale-relative.
    np.testing.assert_allclose(path[-1], [-15.0, -10.0], atol=3.0)


def test_integrate_camera_path_holds_on_dropped_step():
    module = load_module()
    blank = np.zeros((128, 128), dtype=np.uint8)
    path = module.integrate_camera_path([blank, blank, blank], module.FlowParams())
    # No features anywhere -> every step holds the origin.
    np.testing.assert_allclose(path, np.zeros((3, 2)))

"""Tests for MBES loop status CSV and summary helpers."""

import csv
import importlib.util
import math
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "export_mbes_loop_status.py"


def load_module():
    spec = importlib.util.spec_from_file_location("export_mbes_loop_status", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    import sys
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class _Stamp:
    def __init__(self, sec, nanosec):
        self.sec = sec
        self.nanosec = nanosec


class _Header:
    def __init__(self, stamp, frame_id="map"):
        self.stamp = stamp
        self.frame_id = frame_id


class _LoopStatus:
    def __init__(self, **kwargs):
        self.header = _Header(kwargs.get("stamp", _Stamp(0, 0)))
        self.current_id = kwargs.get("current_id", 1)
        self.candidate_id = kwargs.get("candidate_id", 0)
        self.accepted = kwargs.get("accepted", False)
        self.converged = kwargs.get("converged", False)
        self.fitness_score = kwargs.get("fitness_score", math.nan)
        self.correction_translation_m = kwargs.get("translation", math.nan)
        self.correction_rotation_rad = kwargs.get("rotation", math.nan)
        if "descriptor_centroid_distance_m" in kwargs:
            self.descriptor_centroid_distance_m = kwargs["descriptor_centroid_distance_m"]
        if "descriptor_extent_ratio" in kwargs:
            self.descriptor_extent_ratio = kwargs["descriptor_extent_ratio"]
        if "descriptor_point_count_ratio" in kwargs:
            self.descriptor_point_count_ratio = kwargs["descriptor_point_count_ratio"]
        self.status = kwargs.get("status", "no candidate submaps")


def test_sample_from_msg_uses_fallback_for_zero_stamp():
    module = load_module()
    msg = _LoopStatus(
        accepted=True,
        converged=True,
        fitness_score=0.25,
        translation=0.3,
        rotation=0.04,
        status="accepted",
    )

    sample = module.sample_from_msg(msg, fallback_time=123.5)

    assert sample.timestamp == 123.5
    assert sample.accepted is True
    assert sample.converged is True
    assert sample.fitness_score == 0.25
    assert math.isnan(sample.descriptor_centroid_distance_m)
    assert math.isnan(sample.descriptor_extent_ratio)
    assert math.isnan(sample.descriptor_point_count_ratio)
    assert sample.status == "accepted"


def test_sample_from_msg_reads_descriptor_fields_when_available():
    module = load_module()
    msg = _LoopStatus(
        descriptor_centroid_distance_m=1.5,
        descriptor_extent_ratio=2.5,
        descriptor_point_count_ratio=0.75,
    )

    sample = module.sample_from_msg(msg, fallback_time=123.5)

    assert sample.descriptor_centroid_distance_m == 1.5
    assert sample.descriptor_extent_ratio == 2.5
    assert sample.descriptor_point_count_ratio == 0.75


def test_summarize_counts_reasons_and_quantiles():
    module = load_module()
    samples = [
        module.LoopStatusSample(
            1.0, "map", 1, 0, True, True, 0.1, 0.2, 0.01,
            0.3, 1.1, 0.9, "accepted"),
        module.LoopStatusSample(
            2.0, "map", 2, 0, False, True, 3.0, 0.4, 0.02,
            2.0, 4.0, 0.4,
            "fitness score exceeds gate"),
        module.LoopStatusSample(
            3.0, "map", 3, module.NO_CANDIDATE_ID, False, False,
            math.nan, math.nan, math.nan, math.nan, math.nan, math.nan,
            "no candidate submaps"),
    ]

    summary = module.summarize(samples)

    assert summary["total"] == 3
    assert summary["accepted"] == 1
    assert summary["rejected"] == 1
    assert summary["no_candidate"] == 1
    assert summary["converged"] == 2
    assert summary["rejection_counts"]["fitness score exceeds gate"] == 1
    assert summary["fitness"]["count"] == 2
    assert math.isclose(summary["fitness"]["median"], 1.55)
    assert summary["accepted_fitness"]["max"] == 0.1
    assert summary["descriptor_centroid_distance_m"]["count"] == 2
    assert summary["descriptor_extent_ratio"]["max"] == 4.0
    assert summary["descriptor_point_count_ratio"]["min"] == 0.4


def test_write_csv_quotes_status_and_preserves_numeric_fields(tmp_path):
    module = load_module()
    samples = [
        module.LoopStatusSample(
            10.25, "map", 4, 2, False, True, 1.2, 0.5, 0.12,
            1.5, 2.5, 0.75,
            "fitness score exceeds gate, tune threshold"),
    ]
    out = tmp_path / "status.csv"

    module.write_csv(out, samples)

    with out.open(newline="", encoding="utf-8") as fp:
        rows = list(csv.DictReader(fp))
    assert len(rows) == 1
    assert rows[0]["timestamp"] == "10.250000000"
    assert rows[0]["accepted"] == "0"
    assert rows[0]["converged"] == "1"
    assert rows[0]["descriptor_centroid_distance_m"] == "1.500000000"
    assert rows[0]["descriptor_extent_ratio"] == "2.500000000"
    assert rows[0]["descriptor_point_count_ratio"] == "0.750000000"
    assert rows[0]["status"] == "fitness score exceeds gate, tune threshold"


def test_format_summary_markdown_contains_key_sections():
    module = load_module()
    summary = module.summarize([
        module.LoopStatusSample(
            1.0, "map", 1, 0, True, True, 0.2, 0.3, 0.04,
            0.5, 1.2, 0.8, "accepted")
    ])

    text = module.format_summary_markdown(summary, "/mbes_loop_closure/status")

    assert "# MBES Loop Closure Status Summary" in text
    assert "Accepted: 1" in text
    assert "`/mbes_loop_closure/status`" in text
    assert "| fitness_score |" in text
    assert "| descriptor_centroid_distance_m |" in text
    assert "| descriptor_extent_ratio |" in text
    assert "| descriptor_point_count_ratio |" in text

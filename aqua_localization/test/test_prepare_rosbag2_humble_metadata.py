"""Tests for prepare_rosbag2_humble_metadata.py."""

import importlib.util
import sys
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "prepare_rosbag2_humble_metadata.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location(
        "prepare_rosbag2_humble_metadata", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_prepare_metadata_view_rewrites_new_rosbags_fields(tmp_path):
    module = load_module()
    src = tmp_path / "src"
    dst = tmp_path / "dst"
    src.mkdir()
    (src / "sample.mcap").write_bytes(b"mcap")
    (src / "metadata.yaml").write_text(
        "\n".join(
            [
                "rosbag2_bagfile_information:",
                "  compression_format: ''",
                "  storage_identifier: mcap",
                "  topics_with_message_count:",
                "  - message_count: 1",
                "    topic_metadata:",
                "      name: /tf_static",
                "      type: tf2_msgs/msg/TFMessage",
                "      serialization_format: cdr",
                "      type_description_hash: RIHS01_deadbeef",
                "      offered_qos_profiles:",
                "      - reliability: reliable",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    bag_files, rewrites = module.prepare_metadata_view(src, dst)

    text = (dst / "metadata.yaml").read_text(encoding="utf-8")
    assert bag_files == 1
    assert rewrites == 3
    assert (dst / "sample.mcap").exists()
    assert "version: 5" in text
    assert "type_description_hash" not in text
    assert "offered_qos_profiles: ''" in text


def test_prepare_metadata_view_refuses_existing_destination(tmp_path):
    module = load_module()
    src = tmp_path / "src"
    dst = tmp_path / "dst"
    src.mkdir()
    dst.mkdir()
    (src / "metadata.yaml").write_text(
        "rosbag2_bagfile_information:\n  topics_with_message_count: []\n",
        encoding="utf-8",
    )

    try:
        module.prepare_metadata_view(src, dst)
    except FileExistsError as exc:
        assert "destination already exists" in str(exc)
    else:
        raise AssertionError("expected FileExistsError")


def test_rewrite_metadata_in_place(tmp_path):
    module = load_module()
    bag = tmp_path / "bag"
    bag.mkdir()
    metadata = bag / "metadata.yaml"
    metadata.write_text(
        "\n".join(
            [
                "rosbag2_bagfile_information:",
                "  version: 9",
                "  topics_with_message_count:",
                "  - message_count: 1",
                "    topic_metadata:",
                "      name: /clock",
                "      type: rosgraph_msgs/msg/Clock",
                "      offered_qos_profiles: []",
                "      type_description_hash: RIHS01_deadbeef",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    rewrites = module.rewrite_metadata_in_place(bag)

    text = metadata.read_text(encoding="utf-8")
    assert rewrites == 3
    assert "version: 5" in text
    assert "type_description_hash" not in text
    assert "offered_qos_profiles: ''" in text

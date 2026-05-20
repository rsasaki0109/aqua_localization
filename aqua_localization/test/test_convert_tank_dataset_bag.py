"""Pure helper tests for convert_tank_dataset_bag.py."""

import importlib.util
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "convert_tank_dataset_bag.py"


def load_module():
    spec = importlib.util.spec_from_file_location("convert_tank_dataset_bag", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_rewrite_metadata_for_humble(tmp_path):
    module = load_module()
    metadata = tmp_path / "metadata.yaml"
    metadata.write_text(
        "\n".join(
            [
                "rosbag2_bagfile_information:",
                "  topics_with_message_count:",
                "  - topic_metadata:",
                "      offered_qos_profiles: []",
                "      type: nav_msgs/msg/Odometry",
                "      type_description_hash: RIHS01_deadbeef",
                "  version: 9",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    module.rewrite_metadata_for_humble(tmp_path)

    text = metadata.read_text(encoding="utf-8")
    assert 'offered_qos_profiles: ""' in text
    assert "type_description_hash" not in text
    assert "version: 5" in text

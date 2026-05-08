import importlib.util
from pathlib import Path


def load_module():
    script_path = Path(__file__).resolve().parents[1] / "scripts" / "inspect_bag_topics.py"
    spec = importlib.util.spec_from_file_location("inspect_bag_topics", script_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_metadata(tmp_path, topics):
    metadata = tmp_path / "metadata.yaml"
    lines = ["rosbag2_bagfile_information:", "  topics_with_message_count:"]
    for name, msg_type in topics:
        lines.extend(
            [
                "    - topic_metadata:",
                f"        name: {name}",
                f"        type: {msg_type}",
            ]
        )
    metadata.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return metadata


def test_ntnu_style_barometer_enables_scalar_adapter(tmp_path):
    module = load_module()
    metadata = write_metadata(
        tmp_path,
        [
            ("/imu/data", "sensor_msgs/msg/Imu"),
            ("/barometer", "std_msgs/msg/Float64"),
        ],
    )

    topics = module.parse_metadata(metadata)
    launch_args, selection = module.build_launch_args(tmp_path, topics, "default")

    assert selection["imu"]["name"] == "/imu/data"
    assert selection["scalar_pressure"]["name"] == "/barometer"
    assert selection["depth"] is None
    assert "enable_scalar_to_pressure:=true" in launch_args
    assert "bag_scalar_pressure_topic:=/barometer" in launch_args
    assert any("scalar_to_pressure_ntnu.yaml" in arg for arg in launch_args)
    assert "enable_sonar_loc:=false" in launch_args
    assert "enable_fusion:=false" in launch_args


def test_depth_topic_uses_depth_adapter_not_scalar_adapter(tmp_path):
    module = load_module()
    metadata = write_metadata(
        tmp_path,
        [
            ("/rexrov/imu", "sensor_msgs/msg/Imu"),
            ("/rexrov/depth", "std_msgs/msg/Float64"),
        ],
    )

    topics = module.parse_metadata(metadata)
    launch_args, selection = module.build_launch_args(tmp_path, topics, "uuv_simulator")

    assert selection["depth"]["name"] == "/rexrov/depth"
    assert selection["scalar_pressure"] is None
    assert "enable_depth_to_pressure:=true" in launch_args
    assert "bag_depth_topic:=/rexrov/depth" in launch_args
    assert "enable_scalar_to_pressure:=true" not in launch_args


def test_fluid_pressure_wins_over_scalar_pressure(tmp_path):
    module = load_module()
    metadata = write_metadata(
        tmp_path,
        [
            ("/imu/data", "sensor_msgs/msg/Imu"),
            ("/pressure", "sensor_msgs/msg/FluidPressure"),
            ("/barometer", "std_msgs/msg/Float64"),
        ],
    )

    topics = module.parse_metadata(metadata)
    launch_args, selection = module.build_launch_args(tmp_path, topics, "default")

    assert selection["pressure"]["name"] == "/pressure"
    assert selection["scalar_pressure"]["name"] == "/barometer"
    assert "bag_pressure_topic:=/pressure" in launch_args
    assert "enable_scalar_to_pressure:=true" not in launch_args

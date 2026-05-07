from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare("aqua_imu_loc"), "config", "depth_to_pressure.yaml"]
    )

    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to the depth_to_pressure parameter YAML file.",
            ),
            Node(
                package="aqua_imu_loc",
                executable="depth_to_pressure_node",
                name="depth_to_pressure",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )

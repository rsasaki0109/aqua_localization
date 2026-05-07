from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    default_params = PathJoinSubstitution(
        [FindPackageShare("aqua_imu_loc"), "config", "scalar_to_pressure.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to the scalar_to_pressure parameter YAML file.",
            ),
            Node(
                package="aqua_imu_loc",
                executable="scalar_to_pressure_node",
                name="scalar_to_pressure",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )

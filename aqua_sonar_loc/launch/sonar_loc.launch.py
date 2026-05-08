from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare("aqua_sonar_loc"), "config", "params.yaml"]
    )
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to the aqua_sonar_loc parameter YAML file.",
            ),
            Node(
                package="aqua_sonar_loc",
                executable="sonar_loc_node",
                name="aqua_sonar_loc",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )

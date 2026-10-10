"""Launch the aqua_graph factor-graph estimator with a parameter profile."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("aqua_graph"), "config", "params.yaml"]),
            description="aqua_graph parameter profile (e.g. config/tank_dataset.yaml).",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="aqua_graph",
            executable="aqua_graph_node",
            name="aqua_graph",
            output="screen",
            parameters=[
                params_file,
                {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            ],
        ),
    ])

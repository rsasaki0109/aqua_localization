"""Standalone launch for the aqua_pose_graph node."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("aqua_pose_graph")
    default_params = f"{pkg_share}/config/params.yaml"

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file", default_value=default_params,
            description="YAML parameter file passed to aqua_pose_graph.",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Use sim time (set true when replaying a bag with --clock).",
        ),
        Node(
            package="aqua_pose_graph",
            executable="pose_graph_node",
            name="aqua_pose_graph",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),
    ])

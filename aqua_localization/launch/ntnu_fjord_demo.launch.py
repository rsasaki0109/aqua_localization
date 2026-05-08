"""Public-demo launch preset for the NTNU subset-fjord/fjord_1 sequence.

Wraps `replay.launch.py` with NTNU-specific defaults so the demo can be reproduced with
a single command. The bag path is taken from the workspace-relative location used by
`datasets/ntnu_demo.md` and can be overridden with `bag_path:=/abs/or/rel/path`.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bag_path = LaunchConfiguration("bag_path")
    enable_rviz = LaunchConfiguration("enable_rviz")
    bag_rate = LaunchConfiguration("bag_rate")
    loop_bag = LaunchConfiguration("loop_bag")

    imu_params = PathJoinSubstitution(
        [FindPackageShare("aqua_imu_loc"), "config", "ntnu_fjord.yaml"]
    )
    replay_launch = PathJoinSubstitution(
        [FindPackageShare("aqua_localization"), "launch", "replay.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_path",
                default_value=(
                    "aqua_localization/datasets/public/ntnu/"
                    "subset-fjord/fjord_1/fjord_1_ros2"
                ),
                description=(
                    "Path to the converted NTNU fjord_1 rosbag2 directory. "
                    "Default is workspace-relative; pass an absolute path when "
                    "launching from another working directory."
                ),
            ),
            DeclareLaunchArgument(
                "enable_rviz",
                default_value="true",
                description="Start RViz with the aqua_localization demo display config.",
            ),
            DeclareLaunchArgument(
                "bag_rate",
                default_value="1.0",
                description="Playback rate passed to ros2 bag play --rate.",
            ),
            DeclareLaunchArgument(
                "loop_bag",
                default_value="false",
                description="Loop the bag playback.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(replay_launch),
                launch_arguments={
                    "start_bag": "true",
                    "loop_bag": loop_bag,
                    "bag_path": bag_path,
                    "bag_rate": bag_rate,
                    "use_sim_time": "true",
                    "imu_params_file": imu_params,
                    "enable_sonar_loc": "false",
                    "enable_fusion": "false",
                    "enable_rviz": enable_rviz,
                }.items(),
            ),
        ]
    )

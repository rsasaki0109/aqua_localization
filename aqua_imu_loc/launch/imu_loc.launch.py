from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare("aqua_imu_loc"), "config", "params.yaml"]
    )

    params_file = LaunchConfiguration("params_file")
    current_velocity_topic = LaunchConfiguration("current_velocity_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to the aqua_imu_loc parameter YAML file.",
            ),
            DeclareLaunchArgument(
                "current_velocity_topic",
                default_value="",
                description=(
                    "Optional geometry_msgs/TwistStamped water-current velocity topic. "
                    "Leave empty to use the YAML static current_velocity_xyz."
                ),
            ),
            Node(
                package="aqua_imu_loc",
                executable="imu_loc_node",
                name="aqua_imu_loc",
                output="screen",
                parameters=[
                    params_file,
                    {"topics.current_velocity": current_velocity_topic},
                ],
            ),
        ]
    )

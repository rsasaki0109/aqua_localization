"""Live Tank Dataset VIO: stereo frontend + aqua_graph in one container.

The two components share a process (intra-process transport for the
feature stream) but run on a multi-threaded executor, so image processing
and graph updates overlap as they would on the vehicle. Pair with
`ros2 bag play --clock <short_test_ros2>` and use_sim_time:=true.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    sim_time = {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}
    graph_params = LaunchConfiguration("graph_params")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "graph_params",
            default_value=PathJoinSubstitution(
                [FindPackageShare("aqua_graph"), "config", "tank_dataset.yaml"])),
        DeclareLaunchArgument("max_features", default_value="100"),
        DeclareLaunchArgument("publish_every", default_value="2"),
        ComposableNodeContainer(
            name="aqua_vio_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            output="screen",
            composable_node_descriptions=[
                ComposableNode(
                    package="aqua_visual_frontend",
                    plugin="aqua_visual_frontend::StereoFrontendNode",
                    name="stereo_frontend",
                    parameters=[sim_time, {
                        "max_features": ParameterValue(
                            LaunchConfiguration("max_features"), value_type=int),
                        "publish_every": ParameterValue(
                            LaunchConfiguration("publish_every"), value_type=int),
                    }],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
                ComposableNode(
                    package="aqua_graph",
                    plugin="aqua_graph::AquaGraphNode",
                    name="aqua_graph",
                    parameters=[graph_params, sim_time, {
                        "topics.stereo_features": "/aqua_visual_frontend/stereo_features",
                    }],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
            ],
        ),
    ])

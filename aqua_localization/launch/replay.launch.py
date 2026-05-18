from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def package_params(package_name):
    return PathJoinSubstitution(
        [FindPackageShare(package_name), "config", "params.yaml"]
    )


def package_config(package_name, file_name):
    return PathJoinSubstitution(
        [FindPackageShare(package_name), "config", file_name]
    )


def package_file(package_name, directory, file_name):
    return PathJoinSubstitution(
        [FindPackageShare(package_name), directory, file_name]
    )


def and_condition(left, right):
    return IfCondition(PythonExpression(["'", left, "' == 'true' and '", right, "' == 'true'"]))


def and_not_condition(left, right):
    return IfCondition(PythonExpression(["'", left, "' == 'true' and '", right, "' != 'true'"]))


def generate_launch_description():
    enable_depth_to_pressure = LaunchConfiguration("enable_depth_to_pressure")
    enable_scalar_to_pressure = LaunchConfiguration("enable_scalar_to_pressure")
    enable_imu_loc = LaunchConfiguration("enable_imu_loc")
    enable_sonar_loc = LaunchConfiguration("enable_sonar_loc")
    enable_fusion = LaunchConfiguration("enable_fusion")
    enable_pose_graph = LaunchConfiguration("enable_pose_graph")
    enable_mbes_loop_closure = LaunchConfiguration("enable_mbes_loop_closure")
    enable_rviz = LaunchConfiguration("enable_rviz")
    start_bag = LaunchConfiguration("start_bag")
    loop_bag = LaunchConfiguration("loop_bag")
    bag_path = LaunchConfiguration("bag_path")
    bag_rate = LaunchConfiguration("bag_rate")
    use_sim_time = LaunchConfiguration("use_sim_time")
    current_velocity_topic = LaunchConfiguration("current_velocity_topic")
    bag_imu_topic = LaunchConfiguration("bag_imu_topic")
    bag_depth_topic = LaunchConfiguration("bag_depth_topic")
    bag_scalar_pressure_topic = LaunchConfiguration("bag_scalar_pressure_topic")
    bag_pressure_topic = LaunchConfiguration("bag_pressure_topic")
    bag_sonar_points_topic = LaunchConfiguration("bag_sonar_points_topic")
    stack_imu_topic = LaunchConfiguration("stack_imu_topic")
    stack_depth_topic = LaunchConfiguration("stack_depth_topic")
    stack_scalar_pressure_topic = LaunchConfiguration("stack_scalar_pressure_topic")
    stack_pressure_topic = LaunchConfiguration("stack_pressure_topic")
    stack_sonar_points_topic = LaunchConfiguration("stack_sonar_points_topic")

    imu_params_file = LaunchConfiguration("imu_params_file")
    depth_to_pressure_params_file = LaunchConfiguration("depth_to_pressure_params_file")
    scalar_to_pressure_params_file = LaunchConfiguration("scalar_to_pressure_params_file")
    sonar_params_file = LaunchConfiguration("sonar_params_file")
    fusion_params_file = LaunchConfiguration("fusion_params_file")
    pose_graph_params_file = LaunchConfiguration("pose_graph_params_file")
    mbes_loop_closure_params_file = LaunchConfiguration("mbes_loop_closure_params_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    common_node_parameters = [{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}]
    imu_tf_enabled = ParameterValue(
        PythonExpression(["'", enable_fusion, "' != 'true'"]),
        value_type=bool,
    )
    fusion_tf_enabled = ParameterValue(enable_fusion, value_type=bool)

    bag_play_cmd = ["ros2", "bag", "play", bag_path, "--rate", bag_rate]
    bag_play_loop_cmd = bag_play_cmd + ["--loop"]
    imu_remappings = [
        (stack_imu_topic, bag_imu_topic),
        (stack_pressure_topic, bag_pressure_topic),
    ]
    depth_to_pressure_remappings = [
        (stack_depth_topic, bag_depth_topic),
    ]
    scalar_to_pressure_remappings = [
        (stack_scalar_pressure_topic, bag_scalar_pressure_topic),
    ]
    sonar_remappings = [
        (stack_sonar_points_topic, bag_sonar_points_topic),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_path",
                default_value="",
                description="Path to a rosbag2 directory. Used only when start_bag is true.",
            ),
            DeclareLaunchArgument(
                "start_bag",
                default_value="false",
                description="Start ros2 bag play from this launch file.",
            ),
            DeclareLaunchArgument(
                "loop_bag",
                default_value="false",
                description="Pass --loop to ros2 bag play.",
            ),
            DeclareLaunchArgument(
                "bag_rate",
                default_value="1.0",
                description="Playback rate passed to ros2 bag play --rate.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Set use_sim_time on aqua_localization nodes.",
            ),
            DeclareLaunchArgument(
                "current_velocity_topic",
                default_value="",
                description=(
                    "Optional geometry_msgs/TwistStamped water-current velocity topic for aqua_imu_loc."
                ),
            ),
            DeclareLaunchArgument(
                "bag_imu_topic",
                default_value="/imu/data",
                description="IMU topic name recorded in the bag.",
            ),
            DeclareLaunchArgument(
                "bag_depth_topic",
                default_value="/depth",
                description="Positive-down depth topic recorded in the bag for depth_to_pressure_node.",
            ),
            DeclareLaunchArgument(
                "bag_scalar_pressure_topic",
                default_value="/scalar_pressure",
                description="Scalar pressure/depth/barometer topic recorded in the bag.",
            ),
            DeclareLaunchArgument(
                "bag_pressure_topic",
                default_value="/pressure",
                description="Fluid pressure topic name recorded in the bag.",
            ),
            DeclareLaunchArgument(
                "bag_sonar_points_topic",
                default_value="/sonar/points",
                description="Sonar PointCloud2 topic name recorded in the bag.",
            ),
            DeclareLaunchArgument(
                "stack_imu_topic",
                default_value="/imu/data",
                description="IMU topic expected by aqua_imu_loc params.",
            ),
            DeclareLaunchArgument(
                "stack_depth_topic",
                default_value="/depth",
                description="Depth topic expected by depth_to_pressure_node params.",
            ),
            DeclareLaunchArgument(
                "stack_scalar_pressure_topic",
                default_value="/scalar_pressure",
                description="Scalar topic expected by scalar_to_pressure_node params.",
            ),
            DeclareLaunchArgument(
                "stack_pressure_topic",
                default_value="/pressure",
                description="Fluid pressure topic expected by aqua_imu_loc params.",
            ),
            DeclareLaunchArgument(
                "stack_sonar_points_topic",
                default_value="/sonar/points",
                description="PointCloud2 topic expected by aqua_sonar_loc params.",
            ),
            DeclareLaunchArgument(
                "enable_depth_to_pressure",
                default_value="false",
                description="Start the std_msgs/Float64 depth to FluidPressure adapter node.",
            ),
            DeclareLaunchArgument(
                "enable_scalar_to_pressure",
                default_value="false",
                description="Start the scalar pressure/depth/barometer to FluidPressure adapter node.",
            ),
            DeclareLaunchArgument(
                "enable_imu_loc",
                default_value="true",
                description="Start the aqua_imu_loc IMU + pressure localization node.",
            ),
            DeclareLaunchArgument(
                "enable_sonar_loc",
                default_value="true",
                description="Start the aqua_sonar_loc sonar preprocessing and scan matching node.",
            ),
            DeclareLaunchArgument(
                "enable_fusion",
                default_value="true",
                description="Start the aqua_fusion loosely coupled fusion node.",
            ),
            DeclareLaunchArgument(
                "enable_pose_graph",
                default_value="false",
                description="Start the aqua_pose_graph SE(3) keyframe backend.",
            ),
            DeclareLaunchArgument(
                "enable_mbes_loop_closure",
                default_value="false",
                description=(
                    "Start the experimental MBES submap loop-closure front end. "
                    "Requires enable_pose_graph:=true and an MBES-style PointCloud2 stream."
                ),
            ),
            DeclareLaunchArgument(
                "enable_rviz",
                default_value="false",
                description="Start RViz with the aqua_localization demo display config.",
            ),
            DeclareLaunchArgument(
                "imu_params_file",
                default_value=package_params("aqua_imu_loc"),
                description="Parameter YAML for aqua_imu_loc.",
            ),
            DeclareLaunchArgument(
                "depth_to_pressure_params_file",
                default_value=package_config("aqua_imu_loc", "depth_to_pressure.yaml"),
                description="Parameter YAML for depth_to_pressure_node.",
            ),
            DeclareLaunchArgument(
                "scalar_to_pressure_params_file",
                default_value=package_config("aqua_imu_loc", "scalar_to_pressure.yaml"),
                description="Parameter YAML for scalar_to_pressure_node.",
            ),
            DeclareLaunchArgument(
                "sonar_params_file",
                default_value=package_params("aqua_sonar_loc"),
                description="Parameter YAML for aqua_sonar_loc.",
            ),
            DeclareLaunchArgument(
                "fusion_params_file",
                default_value=package_params("aqua_fusion"),
                description="Parameter YAML for aqua_fusion.",
            ),
            DeclareLaunchArgument(
                "pose_graph_params_file",
                default_value=package_params("aqua_pose_graph"),
                description="Parameter YAML for aqua_pose_graph.",
            ),
            DeclareLaunchArgument(
                "mbes_loop_closure_params_file",
                default_value=package_config("aqua_sonar_loc", "mbes_loop_closure.yaml"),
                description="Parameter YAML for the MBES loop-closure front end.",
            ),
            DeclareLaunchArgument(
                "rviz_config_file",
                default_value=package_file("aqua_localization", "rviz", "demo.rviz"),
                description="RViz config file for demo visualization.",
            ),
            Node(
                package="aqua_imu_loc",
                executable="depth_to_pressure_node",
                name="depth_to_pressure",
                output="screen",
                parameters=[depth_to_pressure_params_file, *common_node_parameters],
                remappings=depth_to_pressure_remappings,
                condition=IfCondition(enable_depth_to_pressure),
            ),
            Node(
                package="aqua_imu_loc",
                executable="scalar_to_pressure_node",
                name="scalar_to_pressure",
                output="screen",
                parameters=[scalar_to_pressure_params_file, *common_node_parameters],
                remappings=scalar_to_pressure_remappings,
                condition=IfCondition(enable_scalar_to_pressure),
            ),
            Node(
                package="aqua_imu_loc",
                executable="imu_loc_node",
                name="aqua_imu_loc",
                output="screen",
                parameters=[
                    imu_params_file,
                    {"topics.current_velocity": current_velocity_topic},
                    {"publish.tf": imu_tf_enabled},
                    *common_node_parameters,
                ],
                remappings=imu_remappings,
                condition=IfCondition(enable_imu_loc),
            ),
            Node(
                package="aqua_sonar_loc",
                executable="sonar_loc_node",
                name="aqua_sonar_loc",
                output="screen",
                parameters=[sonar_params_file, *common_node_parameters],
                remappings=sonar_remappings,
                condition=IfCondition(enable_sonar_loc),
            ),
            Node(
                package="aqua_fusion",
                executable="fusion_node",
                name="aqua_fusion",
                output="screen",
                parameters=[
                    fusion_params_file,
                    {"publish.tf": fusion_tf_enabled},
                    *common_node_parameters,
                ],
                condition=IfCondition(enable_fusion),
            ),
            Node(
                package="aqua_pose_graph",
                executable="pose_graph_node",
                name="aqua_pose_graph",
                output="screen",
                parameters=[pose_graph_params_file, *common_node_parameters],
                condition=IfCondition(enable_pose_graph),
            ),
            Node(
                package="aqua_sonar_loc",
                executable="mbes_loop_closure_node",
                name="mbes_loop_closure",
                output="screen",
                parameters=[mbes_loop_closure_params_file, *common_node_parameters],
                condition=IfCondition(enable_mbes_loop_closure),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="aqua_localization_rviz",
                output="screen",
                arguments=["-d", rviz_config_file],
                condition=IfCondition(enable_rviz),
            ),
            ExecuteProcess(
                cmd=bag_play_cmd,
                output="screen",
                condition=and_not_condition(start_bag, loop_bag),
            ),
            ExecuteProcess(
                cmd=bag_play_loop_cmd,
                output="screen",
                condition=and_condition(start_bag, loop_bag),
            ),
        ]
    )

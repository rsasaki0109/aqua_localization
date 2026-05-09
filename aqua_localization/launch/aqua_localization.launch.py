from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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


def generate_launch_description():
    enable_depth_to_pressure = LaunchConfiguration("enable_depth_to_pressure")
    enable_scalar_to_pressure = LaunchConfiguration("enable_scalar_to_pressure")
    enable_imu_loc = LaunchConfiguration("enable_imu_loc")
    enable_sonar_loc = LaunchConfiguration("enable_sonar_loc")
    enable_fusion = LaunchConfiguration("enable_fusion")
    enable_pose_graph = LaunchConfiguration("enable_pose_graph")
    enable_rviz = LaunchConfiguration("enable_rviz")
    current_velocity_topic = LaunchConfiguration("current_velocity_topic")

    imu_params_file = LaunchConfiguration("imu_params_file")
    depth_to_pressure_params_file = LaunchConfiguration("depth_to_pressure_params_file")
    scalar_to_pressure_params_file = LaunchConfiguration("scalar_to_pressure_params_file")
    sonar_params_file = LaunchConfiguration("sonar_params_file")
    fusion_params_file = LaunchConfiguration("fusion_params_file")
    pose_graph_params_file = LaunchConfiguration("pose_graph_params_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    imu_tf_enabled = ParameterValue(
        PythonExpression(["'", enable_fusion, "' != 'true'"]),
        value_type=bool,
    )
    fusion_tf_enabled = ParameterValue(enable_fusion, value_type=bool)

    return LaunchDescription(
        [
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
                description=(
                    "Start the aqua_pose_graph SE(3) keyframe backend. "
                    "Off by default since it does not yet generate loop "
                    "closure constraints — switch on to expose "
                    "/aqua_pose_graph/path."
                ),
            ),
            DeclareLaunchArgument(
                "enable_rviz",
                default_value="false",
                description="Start RViz with the aqua_localization demo display config.",
            ),
            DeclareLaunchArgument(
                "current_velocity_topic",
                default_value="",
                description=(
                    "Optional geometry_msgs/TwistStamped water-current velocity topic for aqua_imu_loc."
                ),
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
                "rviz_config_file",
                default_value=package_file("aqua_localization", "rviz", "demo.rviz"),
                description="RViz config file for demo visualization.",
            ),
            Node(
                package="aqua_imu_loc",
                executable="depth_to_pressure_node",
                name="depth_to_pressure",
                output="screen",
                parameters=[depth_to_pressure_params_file],
                condition=IfCondition(enable_depth_to_pressure),
            ),
            Node(
                package="aqua_imu_loc",
                executable="scalar_to_pressure_node",
                name="scalar_to_pressure",
                output="screen",
                parameters=[scalar_to_pressure_params_file],
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
                ],
                condition=IfCondition(enable_imu_loc),
            ),
            Node(
                package="aqua_sonar_loc",
                executable="sonar_loc_node",
                name="aqua_sonar_loc",
                output="screen",
                parameters=[sonar_params_file],
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
                ],
                condition=IfCondition(enable_fusion),
            ),
            Node(
                package="aqua_pose_graph",
                executable="pose_graph_node",
                name="aqua_pose_graph",
                output="screen",
                parameters=[pose_graph_params_file],
                condition=IfCondition(enable_pose_graph),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="aqua_localization_rviz",
                output="screen",
                arguments=["-d", rviz_config_file],
                condition=IfCondition(enable_rviz),
            ),
        ]
    )

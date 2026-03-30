from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

launch_args = [
    DeclareLaunchArgument(name="namespace",    default_value="ov_msckf", description="namespace"),
    DeclareLaunchArgument(name="ov_enable",    default_value="true",     description="enable OpenVINS node"),
    DeclareLaunchArgument(name="rviz_enable",  default_value="false",    description="enable rviz node"),
    DeclareLaunchArgument(
        name="config",
        default_value="euroc_mav",
        description="euroc_mav, tum_vi, rpng_aruco...",
    ),
    DeclareLaunchArgument(
        name="config_path",
        default_value="",
        description="path to estimator_config.yaml. If not given, determined from 'config' above",
    ),
    DeclareLaunchArgument(
        name="verbosity",
        default_value="INFO",
        description="ALL, DEBUG, INFO, WARNING, ERROR, SILENT",
    ),
    DeclareLaunchArgument(
        name="use_stereo",
        default_value="true",
        description="if we have more than 1 camera, track stereo constraints between pairs",
    ),
    DeclareLaunchArgument(
        name="max_cameras",
        default_value="2",
        description="1 = mono, 2 = stereo, >2 = binocular (all mono tracking)",
    ),
    DeclareLaunchArgument(
        name="save_total_state",
        default_value="false",
        description="record total state with calibration and features to a txt file",
    ),
    # Bag-specific arguments
    DeclareLaunchArgument(
        name="path_bag",
        default_value="",
        description="path to ROS2 bag directory (required)",
    ),
    DeclareLaunchArgument(
        name="bag_start",
        default_value="0.0",
        description="seconds to skip from the beginning of the bag",
    ),
    DeclareLaunchArgument(
        name="bag_durr",
        default_value="-1.0",
        description="duration to process in seconds (-1 = full bag)",
    ),
    DeclareLaunchArgument(
        name="topic_imu",
        default_value="/imu0",
        description="IMU topic override (default read from config YAML)",
    ),
    DeclareLaunchArgument(
        name="topic_camera0",
        default_value="/cam0/image_raw",
        description="camera 0 topic override",
    ),
    DeclareLaunchArgument(
        name="topic_camera1",
        default_value="/cam1/image_raw",
        description="camera 1 topic override",
    ),
    DeclareLaunchArgument(
        name="path_gt",
        default_value="",
        description="optional path to ground-truth ASL CSV file for GT initialisation",
    ),
]


def launch_setup(context):
    # Resolve config path
    config_path = LaunchConfiguration("config_path").perform(context)
    if not config_path:
        configs_dir = os.path.join(get_package_share_directory("ov_msckf"), "config")
        config = LaunchConfiguration("config").perform(context)
        available_configs = os.listdir(configs_dir)
        if config in available_configs:
            config_path = os.path.join(configs_dir, config, "estimator_config.yaml")
        else:
            return [
                LogInfo(
                    msg="ERROR: unknown config: '{}' - Available configs are: {} - not starting OpenVINS".format(
                        config, ", ".join(available_configs)
                    )
                )
            ]
    else:
        if not os.path.isfile(config_path):
            return [
                LogInfo(
                    msg="ERROR: config_path '{}' does not exist - not starting OpenVINS".format(config_path)
                )
            ]

    # Validate bag path
    path_bag = LaunchConfiguration("path_bag").perform(context)
    if not path_bag:
        return [LogInfo(msg="ERROR: path_bag is required - not starting OpenVINS")]
    if not os.path.isdir(path_bag):
        return [LogInfo(msg="ERROR: path_bag '{}' is not a directory - not starting OpenVINS".format(path_bag))]

    node1 = Node(
        package="ov_msckf",
        executable="ros2_serial_msckf",
        condition=IfCondition(LaunchConfiguration("ov_enable")),
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        on_exit=Shutdown(),
        parameters=[
            {"config_path":      config_path},
            {"verbosity":        LaunchConfiguration("verbosity")},
            {"use_stereo":       LaunchConfiguration("use_stereo")},
            {"max_cameras":      LaunchConfiguration("max_cameras")},
            {"save_total_state": LaunchConfiguration("save_total_state")},
            {"path_bag":         path_bag},
            {"bag_start":        LaunchConfiguration("bag_start")},
            {"bag_durr":         LaunchConfiguration("bag_durr")},
            {"topic_imu":        LaunchConfiguration("topic_imu")},
            {"topic_camera0":    LaunchConfiguration("topic_camera0")},
            {"topic_camera1":    LaunchConfiguration("topic_camera1")},
            {"path_gt":          LaunchConfiguration("path_gt")},
        ],
    )

    node2 = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(LaunchConfiguration("rviz_enable")),
        arguments=[
            "-d"
            + os.path.join(
                get_package_share_directory("ov_msckf"), "launch", "display_ros2.rviz"
            ),
            "--ros-args",
            "--log-level",
            "warn",
        ],
    )

    return [node1, node2]


def generate_launch_description():
    ld = LaunchDescription(launch_args)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld

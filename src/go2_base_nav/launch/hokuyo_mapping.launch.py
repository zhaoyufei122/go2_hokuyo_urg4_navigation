import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from go2_base_nav.mapping_bag import build_slamtec_bag_record_command


def _start_bag_recording(context):
    output_root = LaunchConfiguration("bag_output_root").perform(context)
    return [
        ExecuteProcess(
            cmd=build_slamtec_bag_record_command(output_root),
            output="screen",
        )
    ]


def _sensor_arguments(use_sim_time):
    names = (
        "robot_odom_topic",
        "laser_parent_frame",
        "laser_frame",
        "laser_x",
        "laser_y",
        "laser_z",
        "laser_roll",
        "laser_pitch",
        "laser_yaw",
    )
    arguments = {name: LaunchConfiguration(name) for name in names}
    arguments["scan_topic"] = "/scan"
    arguments["use_sim_time"] = use_sim_time
    return arguments


def generate_launch_description():
    package_root = Path(__file__).resolve().parents[1]
    sensors_launch = package_root / "launch" / "hokuyo_sensors.launch.py"
    slam_config = package_root / "config" / "hokuyo_slam_toolbox.yaml"
    rviz_config = package_root / "rviz" / "slamtec_mapping.rviz"
    slam_launch = os.path.join(
        get_package_share_directory("slam_toolbox"),
        "launch",
        "online_async_launch.py",
    )

    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    record_bag = LaunchConfiguration("record_bag")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("record_bag", default_value="false"),
            DeclareLaunchArgument(
                "bag_output_root",
                default_value="~/go2_hokuyo_bags",
            ),
            DeclareLaunchArgument(
                "robot_odom_topic",
                default_value="/utlidar/robot_odom",
            ),
            DeclareLaunchArgument(
                "laser_parent_frame",
                default_value="base_link",
            ),
            DeclareLaunchArgument("laser_frame", default_value="laser"),
            DeclareLaunchArgument("laser_x", default_value="0.20"),
            DeclareLaunchArgument("laser_y", default_value="0.0"),
            DeclareLaunchArgument("laser_z", default_value="0.10"),
            DeclareLaunchArgument("laser_roll", default_value="0.0"),
            DeclareLaunchArgument("laser_pitch", default_value="0.0"),
            DeclareLaunchArgument("laser_yaw", default_value="0.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(sensors_launch)),
                launch_arguments=_sensor_arguments(use_sim_time).items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch),
                launch_arguments={
                    "slam_params_file": str(slam_config),
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
            OpaqueFunction(
                function=_start_bag_recording,
                condition=IfCondition(record_bag),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", str(rviz_config)],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(use_rviz),
            ),
        ]
    )

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

from go2_base_nav.mapping_bag import build_bag_record_command


def _start_bag_recording(context):
    output_root = LaunchConfiguration("bag_output_root").perform(context)
    return [
        ExecuteProcess(
            cmd=build_bag_record_command(output_root),
            output="screen",
        )
    ]


def generate_launch_description():
    package_root = Path(__file__).resolve().parents[1]
    sensors_launch = package_root / "launch" / "sensors.launch.py"
    slam_config = package_root / "config" / "slam_toolbox.yaml"
    rviz_config = package_root / "rviz" / "mapping.rviz"
    slam_launch = os.path.join(
        get_package_share_directory("slam_toolbox"),
        "launch",
        "online_async_launch.py",
    )

    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    record_bag = LaunchConfiguration("record_bag")
    use_accumulator = LaunchConfiguration("use_accumulator")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("record_bag", default_value="true"),
            DeclareLaunchArgument(
                "use_accumulator",
                default_value="false",
                description="Forward to sensors.launch.py",
            ),
            DeclareLaunchArgument(
                "bag_output_root",
                default_value="~/go2_mapping_bags",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(sensors_launch)),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "use_accumulator": use_accumulator,
                }.items(),
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

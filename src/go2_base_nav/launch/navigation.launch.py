import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package_root = Path(__file__).resolve().parents[1]
    sensors_launch = package_root / "launch" / "sensors.launch.py"
    nav2_params = package_root / "config" / "nav2_params.yaml"
    rviz_config = package_root / "rviz" / "navigation.rviz"
    nav2_launch = os.path.join(
        get_package_share_directory("nav2_bringup"),
        "launch",
        "bringup_launch.py",
    )
    nav_to_pose_bt = (
        package_root / "behavior_trees" / "navigate_to_pose_go2.xml"
    )
    nav_through_poses_bt = (
        package_root / "behavior_trees" / "navigate_through_poses_go2.xml"
    )
    configured_nav2_params = RewrittenYaml(
        source_file=str(nav2_params),
        param_rewrites={
            "bt_navigator.ros__parameters.default_nav_to_pose_bt_xml": str(
                nav_to_pose_bt
            ),
            "bt_navigator.ros__parameters.default_nav_through_poses_bt_xml": str(
                nav_through_poses_bt
            ),
        },
        convert_types=True,
    )

    map_yaml = LaunchConfiguration("map")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                description="Absolute path to a saved map YAML file",
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(sensors_launch)),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    "map": map_yaml,
                    "params_file": configured_nav2_params,
                    "slam": "False",
                    "use_sim_time": use_sim_time,
                    "autostart": "True",
                    "use_composition": "False",
                    "use_respawn": "False",
                }.items(),
            ),
            Node(
                package="go2_base_nav",
                executable="go2_cmd_vel_bridge",
                name="go2_cmd_vel_bridge",
                output="screen",
                parameters=[
                    {
                        "max_linear_x": 0.4,
                        "max_linear_y": 0.0,
                        "max_angular_z": 0.4,
                        "cmd_timeout": 0.5,
                        "use_sim_time": use_sim_time,
                    }
                ],
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

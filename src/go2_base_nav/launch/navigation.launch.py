import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package_root = Path(__file__).resolve().parents[1]
    sensors_launch = package_root / "launch" / "sensors.launch.py"
    nav2_params = package_root / "config" / "nav2_params.yaml"
    slam_localization_params = (
        package_root / "config" / "slam_toolbox_localization.yaml"
    )
    rviz_config = package_root / "rviz" / "navigation.rviz"
    slam_localization_launch = os.path.join(
        get_package_share_directory("slam_toolbox"),
        "launch",
        "localization_launch.py",
    )
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
    # Only used with localization:=slam_toolbox; map_file_name is rewritten
    # with the slam_posegraph argument (path without .posegraph extension).
    configured_slam_params = RewrittenYaml(
        source_file=str(slam_localization_params),
        param_rewrites={
            "map_file_name": LaunchConfiguration("slam_posegraph"),
        },
        convert_types=True,
    )

    def _include_nav2(context):
        localization = LaunchConfiguration("localization").perform(context)
        use_slam_toolbox = localization == "slam_toolbox"
        # See hokuyo_navigation.launch.py: nav2_bringup's slam:=True path
        # cannot be given our slam params, so it would start slam_toolbox in
        # mapping mode and ignore the posegraph. Bring up the navigation
        # stack only and launch slam_toolbox ourselves.
        launch_arguments = {
            "map": LaunchConfiguration("map"),
            "params_file": configured_nav2_params,
            "slam": "False",
            "use_localization": "False" if use_slam_toolbox else "True",
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "autostart": "True",
            "use_composition": "False",
            "use_respawn": "False",
        }
        actions = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments=launch_arguments.items(),
            )
        ]
        if use_slam_toolbox:
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(slam_localization_launch),
                    launch_arguments={
                        "slam_params_file": configured_slam_params,
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "autostart": "true",
                    }.items(),
                )
            )
        return actions

    map_yaml = LaunchConfiguration("map")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_accumulator = LaunchConfiguration("use_accumulator")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value="",
                description=(
                    "Absolute path to a saved occupancy map YAML file "
                    "(used with the default localization:=amcl)"
                ),
            ),
            DeclareLaunchArgument(
                "localization",
                default_value="amcl",
                description=(
                    "Localization source: 'amcl' (default, known-good) or "
                    "'slam_toolbox' (experimental, loads slam_posegraph)"
                ),
            ),
            DeclareLaunchArgument(
                "slam_posegraph",
                default_value="",
                description=(
                    "Absolute path to the serialized slam_toolbox posegraph "
                    "WITHOUT the .posegraph extension (only when "
                    "localization:=slam_toolbox)"
                ),
            ),
            DeclareLaunchArgument(
                "use_accumulator",
                default_value="false",
                description="Forward to sensors.launch.py",
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(sensors_launch)),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "use_accumulator": use_accumulator,
                }.items(),
            ),
            OpaqueFunction(function=_include_nav2),
            Node(
                package="go2_base_nav",
                executable="go2_cmd_vel_bridge",
                name="go2_cmd_vel_bridge",
                output="screen",
                parameters=[
                    {
                        "max_linear_x": 0.4,
                        "max_linear_y": 0.0,
                        "max_angular_z": 0.6,
                        "min_angular_z": 0.4,
                        "angular_deadband": 0.1,
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

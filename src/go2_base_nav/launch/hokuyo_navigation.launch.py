import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from nav2_common.launch import RewrittenYaml


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
    nav2_params = package_root / "config" / "nav2_params.yaml"
    slam_localization_params = (
        package_root / "config" / "hokuyo_slam_toolbox_localization.yaml"
    )
    rviz_config = package_root / "rviz" / "slamtec_navigation.rviz"
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
        # Do NOT route slam_toolbox through nav2_bringup's slam:=True. Its
        # bringup_launch.py declares no slam_params_file argument, so ours is
        # silently dropped; slam_launch.py then forwards its own params_file
        # (nav2_params.yaml) only if that file has a slam_toolbox section,
        # which it does not. The result is slam_toolbox started on package
        # defaults -- mode: mapping, no map_file_name -- quietly building a
        # fresh map instead of localising against the serialized posegraph.
        # That failure looks like success in RViz, because the scan of course
        # matches a map being drawn from that same scan.
        # Instead: ask nav2 for the navigation stack only, and start
        # slam_toolbox's own localization_launch.py alongside it.
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
            # slam_toolbox localization only provides the map->odom TF; its
            # own /map is a local crop regenerated from the posegraph, so it
            # is remapped aside and the full occupancy map comes from a
            # dedicated map_server (activated by its own lifecycle manager).
            slam_params_path = configured_slam_params.perform(context)
            # map_start_pose is a double array, which RewrittenYaml cannot
            # express (its convert() only produces int/float/bool/str), so it
            # is applied as an explicit parameter override here.
            # Left empty, the value in hokuyo_slam_toolbox_localization.yaml
            # stands -- that is where a fixed launch spot belongs, so it only
            # has to be measured once. The arguments are for one-off starts
            # from somewhere else.
            start_pose_arguments = [
                LaunchConfiguration(name).perform(context)
                for name in ("map_start_x", "map_start_y", "map_start_yaw")
            ]
            slam_overrides = {
                "use_lifecycle_manager": False,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
            if any(value != "" for value in start_pose_arguments):
                if not all(value != "" for value in start_pose_arguments):
                    raise RuntimeError(
                        "map_start_x, map_start_y and map_start_yaw must be "
                        "given together (or all left unset to use the value "
                        "in hokuyo_slam_toolbox_localization.yaml)"
                    )
                slam_overrides["map_start_pose"] = [
                    float(value) for value in start_pose_arguments
                ]
            # localization_slam_toolbox_node is a lifecycle node: it only
            # loads the posegraph after configure+activate transitions.
            slam_node = LifecycleNode(
                package="slam_toolbox",
                executable="localization_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                namespace="",
                parameters=[slam_params_path, slam_overrides],
                remappings=[
                    ("map", "/map_slam"),
                    ("map_metadata", "/map_slam_metadata"),
                ],
            )
            actions.extend(
                [
                    slam_node,
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(slam_node),
                            transition_id=Transition.TRANSITION_CONFIGURE,
                        )
                    ),
                    RegisterEventHandler(
                        OnStateTransition(
                            target_lifecycle_node=slam_node,
                            start_state="configuring",
                            goal_state="inactive",
                            entities=[
                                EmitEvent(
                                    event=ChangeState(
                                        lifecycle_node_matcher=matches_action(
                                            slam_node
                                        ),
                                        transition_id=(
                                            Transition.TRANSITION_ACTIVATE
                                        ),
                                    )
                                )
                            ],
                        )
                    ),
                    Node(
                        package="nav2_map_server",
                        executable="map_server",
                        name="map_server",
                        output="screen",
                        parameters=[
                            {
                                "yaml_filename": LaunchConfiguration("map"),
                                "topic_name": "/map",
                                "frame_id": "map",
                                "use_sim_time": LaunchConfiguration(
                                    "use_sim_time"
                                ),
                            }
                        ],
                    ),
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_map_server",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": LaunchConfiguration(
                                    "use_sim_time"
                                ),
                                "autostart": True,
                                "node_names": ["map_server"],
                                "bond_timeout": 0.0,
                            }
                        ],
                    ),
                ]
            )
        return actions

    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

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
                    "Localization source: 'amcl' (default) or 'slam_toolbox' "
                    "(loads slam_posegraph, publishes map->odom continuously)"
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
            # slam_toolbox REFUSES to deserialize the posegraph unless it is
            # also told where the robot is standing when it loads it -- see
            # SlamToolbox::shouldStartWithPoseGraph(), which logs
            #   "Map starting pose not specified. Set either map_start_pose
            #    or map_start_at_dock."
            # and then returns false, leaving the graph EMPTY. Localization
            # then runs against nothing and looks exactly like fresh mapping.
            # A 2D Pose Estimate does NOT rescue this: it only relocalizes
            # inside an already-loaded graph.
            # The default 0,0,0 is the slam_toolbox map origin, i.e. wherever
            # the dog was standing when the mapping run started. Park it back
            # there, or override these with the real start pose.
            # Unset by default: the fixed launch spot lives in
            # hokuyo_slam_toolbox_localization.yaml so it is measured once.
            # Pass all three together to start from somewhere else instead.
            DeclareLaunchArgument("map_start_x", default_value=""),
            DeclareLaunchArgument("map_start_y", default_value=""),
            DeclareLaunchArgument("map_start_yaw", default_value=""),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
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
                        "max_angular_z": 0.8,
                        "min_angular_z": 0.4,
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

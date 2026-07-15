from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


DEFAULT_DATABASE_PATH = (
    "/home/yufei/Desktop/go2_base_navi/maps/room_3d.db"
)


def _rtabmap_node(
    config_file,
    database_path,
    filtered_cloud_topic,
    use_sim_time,
    condition,
    arguments=None,
):
    return Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        emulate_tty=True,
        parameters=[
            config_file,
            {
                "database_path": database_path,
                "use_sim_time": use_sim_time,
            },
        ],
        remappings=[
            ("scan_cloud", filtered_cloud_topic),
            ("odom", "/rtabmap_unused_odom"),
        ],
        arguments=arguments or [],
        condition=condition,
    )


def generate_launch_description():
    package_root = Path(__file__).resolve().parents[1]
    config_file = str(package_root / "config" / "rtabmap_3d.yaml")
    rviz_config = str(package_root / "rviz" / "mapping_3d.rviz")

    cloud_topic = LaunchConfiguration("cloud_topic")
    robot_odom_topic = LaunchConfiguration("robot_odom_topic")
    database_path = LaunchConfiguration("database_path")
    new_map = LaunchConfiguration("new_map")
    use_rtabmap_viz = LaunchConfiguration("use_rtabmap_viz")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    cropped_cloud_topic = "/cloud_3d_cropped"
    filtered_cloud_topic = "/cloud_3d_filtered"

    cloud_filters = ComposableNodeContainer(
        name="cloud_3d_filters",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="pcl_ros",
                plugin="pcl_ros::CropBox",
                name="cloud_self_filter",
                parameters=[
                    config_file,
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[
                    ("input", cloud_topic),
                    ("output", cropped_cloud_topic),
                ],
            ),
            ComposableNode(
                package="pcl_ros",
                plugin="pcl_ros::VoxelGrid",
                name="cloud_voxel_filter",
                parameters=[
                    config_file,
                    {"use_sim_time": use_sim_time},
                ],
                remappings=[
                    ("input", cropped_cloud_topic),
                    ("output", filtered_cloud_topic),
                ],
            ),
        ],
    )

    planar_odom = Node(
        package="go2_base_nav",
        executable="planar_odom",
        name="planar_odom",
        output="screen",
        parameters=[
            {
                "input_odom_topic": robot_odom_topic,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    fresh_mapping = _rtabmap_node(
        config_file=config_file,
        database_path=database_path,
        filtered_cloud_topic=filtered_cloud_topic,
        use_sim_time=use_sim_time,
        arguments=["--delete_db_on_start"],
        condition=IfCondition(new_map),
    )
    resumed_mapping = _rtabmap_node(
        config_file=config_file,
        database_path=database_path,
        filtered_cloud_topic=filtered_cloud_topic,
        use_sim_time=use_sim_time,
        condition=UnlessCondition(new_map),
    )

    rtabmap_viz = Node(
        package="rtabmap_viz",
        executable="rtabmap_viz",
        name="rtabmap_viz",
        output="screen",
        emulate_tty=True,
        parameters=[
            config_file,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("scan_cloud", filtered_cloud_topic),
            ("odom", "/rtabmap_unused_odom"),
        ],
        condition=IfCondition(use_rtabmap_viz),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "cloud_topic",
                default_value="/utlidar/cloud_deskewed",
            ),
            DeclareLaunchArgument(
                "robot_odom_topic",
                default_value="/utlidar/robot_odom",
            ),
            DeclareLaunchArgument(
                "database_path",
                default_value=DEFAULT_DATABASE_PATH,
            ),
            DeclareLaunchArgument("new_map", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_rtabmap_viz", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            planar_odom,
            cloud_filters,
            fresh_mapping,
            resumed_mapping,
            rtabmap_viz,
            rviz,
        ]
    )

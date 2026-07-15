from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    package_root = Path(__file__).resolve().parents[1]
    projection_config = str(
        package_root / "config" / "pointcloud_to_laserscan.yaml"
    )

    cloud_topic = LaunchConfiguration("cloud_topic")
    robot_odom_topic = LaunchConfiguration("robot_odom_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")
    filtered_cloud_topic = "/cloud_self_filtered"

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
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
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
            ),
            ComposableNodeContainer(
                name="cloud_2d_filters",
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
                            projection_config,
                            {"use_sim_time": use_sim_time},
                        ],
                        remappings=[
                            ("input", cloud_topic),
                            ("output", filtered_cloud_topic),
                        ],
                    )
                ],
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                output="screen",
                parameters=[projection_config, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("cloud_in", filtered_cloud_topic),
                    ("scan", scan_topic),
                ],
            ),
        ]
    )

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_root = Path(__file__).resolve().parents[1]
    projection_config = str(
        package_root / "config" / "pointcloud_to_laserscan.yaml"
    )

    cloud_topic = LaunchConfiguration("cloud_topic")
    robot_odom_topic = LaunchConfiguration("robot_odom_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")

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
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                output="screen",
                parameters=[projection_config, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("cloud_in", cloud_topic),
                    ("scan", scan_topic),
                ],
            ),
        ]
    )

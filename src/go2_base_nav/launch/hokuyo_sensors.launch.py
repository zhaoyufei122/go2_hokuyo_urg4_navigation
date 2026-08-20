from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_odom_topic = LaunchConfiguration("robot_odom_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    laser_parent_frame = LaunchConfiguration("laser_parent_frame")
    laser_frame = LaunchConfiguration("laser_frame")
    laser_x = LaunchConfiguration("laser_x")
    laser_y = LaunchConfiguration("laser_y")
    laser_z = LaunchConfiguration("laser_z")
    laser_roll = LaunchConfiguration("laser_roll")
    laser_pitch = LaunchConfiguration("laser_pitch")
    laser_yaw = LaunchConfiguration("laser_yaw")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_odom_topic",
                default_value="/utlidar/robot_odom",
            ),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
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
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            LogInfo(
                msg=[
                    "Using direct Hokuyo LaserScan ",
                    scan_topic,
                    " in frame ",
                    laser_frame,
                ]
            ),
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
                package="go2_base_nav",
                executable="scan_filter",
                name="hokuyo_scan_filter",
                output="screen",
                # URG-04LX-UG01 leaks error codes as 0.007--0.019 m fake
                # ranges; they would land on the robot cell in the costmap
                # and make Nav2 abort. Replace them with +inf.
                parameters=[
                    {
                        "input_topic": "/scan_raw",
                        "output_topic": "/scan",
                        "min_range": 0.06,
                        "max_range": 4.0,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="hokuyo_laser_static_tf",
                output="screen",
                arguments=[
                    "--x",
                    laser_x,
                    "--y",
                    laser_y,
                    "--z",
                    laser_z,
                    "--roll",
                    laser_roll,
                    "--pitch",
                    laser_pitch,
                    "--yaw",
                    laser_yaw,
                    "--frame-id",
                    laser_parent_frame,
                    "--child-frame-id",
                    laser_frame,
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )

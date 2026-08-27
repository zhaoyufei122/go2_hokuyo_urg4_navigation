from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_package_declares_runtime_dependencies():
    root = ET.parse(PACKAGE_ROOT / "package.xml").getroot()
    dependencies = {item.text for item in root.findall("exec_depend")}
    assert {
        "ament_index_python",
        "geometry_msgs",
        "launch",
        "launch_ros",
        "nav_msgs",
        "nav2_bringup",
        "nav2_collision_monitor",
        "nav2_common",
        "nav2_map_server",
        "nav2_msgs",
        "nav2_regulated_pure_pursuit_controller",
        "nav2_rviz_plugins",
        "nav2_smac_planner",
        "nav2_velocity_smoother",
        "pcl_ros",
        "pointcloud_to_laserscan",
        "rclcpp_components",
        "rclpy",
        "ros2bag",
        "rtabmap_slam",
        "rtabmap_viz",
        "rviz2",
        "sensor_msgs",
        "slam_toolbox",
        "std_msgs",
        "tf2_ros",
        "unitree_api",
    } <= dependencies

    description = root.findtext("description", default="")
    assert "2D navigation" in description
    assert "3D mapping" in description


def test_setup_installs_launch_config_and_rviz_assets():
    setup_text = (PACKAGE_ROOT / "setup.py").read_text()
    assert "planar_odom = go2_base_nav.planar_odom:main" in setup_text
    assert "go2_cmd_vel_bridge = go2_base_nav.cmd_vel_bridge:main" in setup_text
    assert 'glob("launch/*.launch.py")' in setup_text
    assert 'glob("config/*.yaml")' in setup_text
    assert 'glob("rviz/*.rviz")' in setup_text
    assert "2D navigation and 3D mapping" in setup_text

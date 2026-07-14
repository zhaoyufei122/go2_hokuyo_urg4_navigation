from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_package_declares_runtime_dependencies():
    root = ET.parse(PACKAGE_ROOT / "package.xml").getroot()
    dependencies = {item.text for item in root.findall("exec_depend")}
    assert {
        "geometry_msgs",
        "launch",
        "launch_ros",
        "nav_msgs",
        "nav2_bringup",
        "nav2_map_server",
        "pointcloud_to_laserscan",
        "rclpy",
        "rviz2",
        "sensor_msgs",
        "slam_toolbox",
        "tf2_ros",
        "unitree_api",
    } <= dependencies


def test_setup_installs_launch_config_and_rviz_assets():
    setup_text = (PACKAGE_ROOT / "setup.py").read_text()
    assert "planar_odom = go2_base_nav.planar_odom:main" in setup_text
    assert "go2_cmd_vel_bridge = go2_base_nav.cmd_vel_bridge:main" in setup_text
    assert 'glob("launch/*.launch.py")' in setup_text
    assert 'glob("config/*.yaml")' in setup_text
    assert 'glob("rviz/*.rviz")' in setup_text

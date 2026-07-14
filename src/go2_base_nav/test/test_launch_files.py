import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _load_launch_description(filename):
    path = PACKAGE_ROOT / "launch" / filename
    spec = importlib.util.spec_from_file_location(filename.replace(".", "_"), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def test_sensors_launch_starts_planar_odom_and_projection():
    description = _load_launch_description("sensors.launch.py")
    node_pairs = {
        (action.node_package, action.node_executable)
        for action in description.entities
        if isinstance(action, Node)
    }
    assert ("go2_base_nav", "planar_odom") in node_pairs
    assert (
        "pointcloud_to_laserscan",
        "pointcloud_to_laserscan_node",
    ) in node_pairs

    argument_names = {
        action.name
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    assert {
        "cloud_topic",
        "robot_odom_topic",
        "scan_topic",
        "use_sim_time",
    } <= argument_names


def test_mapping_launch_includes_sensors_and_async_slam_without_control():
    path = PACKAGE_ROOT / "launch" / "mapping.launch.py"
    launch_text = path.read_text()
    assert "sensors.launch.py" in launch_text
    assert "online_async_launch.py" in launch_text
    assert "cmd_vel_bridge" not in launch_text

    description = _load_launch_description("mapping.launch.py")
    includes = [
        action
        for action in description.entities
        if isinstance(action, IncludeLaunchDescription)
    ]
    assert len(includes) == 2
    node_pairs = {
        (action.node_package, action.node_executable)
        for action in description.entities
        if isinstance(action, Node)
    }
    assert ("rviz2", "rviz2") in node_pairs
    assert ("go2_base_nav", "go2_cmd_vel_bridge") not in node_pairs

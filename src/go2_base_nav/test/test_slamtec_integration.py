import importlib.util
from pathlib import Path

import yaml
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]


def _load_launch_description(filename):
    path = PACKAGE_ROOT / "launch" / filename
    spec = importlib.util.spec_from_file_location(filename.replace(".", "_"), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _argument_names(description):
    return {
        action.name
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }


def _node_pairs(description):
    return {
        (action.node_package, action.node_executable)
        for action in description.entities
        if isinstance(action, Node)
    }


def test_slamtec_sensors_uses_direct_scan_without_l2_projection():
    path = PACKAGE_ROOT / "launch" / "slamtec_sensors.launch.py"
    launch_text = path.read_text()
    description = _load_launch_description(path.name)

    assert ("go2_base_nav", "planar_odom") in _node_pairs(description)
    assert ("tf2_ros", "static_transform_publisher") in _node_pairs(description)
    assert "pointcloud_to_laserscan" not in launch_text
    assert "pcl_ros" not in launch_text
    assert {
        "robot_odom_topic",
        "scan_topic",
        "laser_parent_frame",
        "laser_frame",
        "laser_x",
        "laser_y",
        "laser_z",
        "laser_roll",
        "laser_pitch",
        "laser_yaw",
        "use_sim_time",
    } <= _argument_names(description)

    for expected_default in (
        'default_value="/scan"',
        'default_value="base_link"',
        'default_value="laser"',
        'default_value="0.20"',
        'default_value="0.25"',
    ):
        assert expected_default in launch_text


def test_slamtec_mapping_uses_direct_scan_slam_and_never_starts_control():
    path = PACKAGE_ROOT / "launch" / "slamtec_mapping.launch.py"
    launch_text = path.read_text()
    description = _load_launch_description(path.name)

    assert "slamtec_sensors.launch.py" in launch_text
    assert "online_async_launch.py" in launch_text
    assert "slamtec_slam_toolbox.yaml" in launch_text
    assert "cmd_vel_bridge" not in launch_text
    assert "build_slamtec_bag_record_command" in launch_text
    assert 'DeclareLaunchArgument("record_bag", default_value="false")' in launch_text

    includes = [
        action
        for action in description.entities
        if isinstance(action, IncludeLaunchDescription)
    ]
    assert len(includes) == 2
    assert ("rviz2", "rviz2") in _node_pairs(description)


def test_slamtec_navigation_reuses_safe_nav2_motion_limits():
    path = PACKAGE_ROOT / "launch" / "slamtec_navigation.launch.py"
    launch_text = path.read_text()
    description = _load_launch_description(path.name)

    assert "slamtec_sensors.launch.py" in launch_text
    assert "bringup_launch.py" in launch_text
    assert '"slam": "False"' in launch_text
    assert ("go2_base_nav", "go2_cmd_vel_bridge") in _node_pairs(description)
    assert ("rviz2", "rviz2") in _node_pairs(description)
    assert '"max_linear_x": 0.4' in launch_text
    assert '"max_linear_y": 0.0' in launch_text
    assert '"max_angular_z": 0.6' in launch_text
    assert '"min_angular_z": 0.4' in launch_text
    assert '"cmd_timeout": 0.5' in launch_text

    map_arguments = [
        action
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument) and action.name == "map"
    ]
    assert len(map_arguments) == 1
    assert map_arguments[0].default_value is None


def test_slamtec_slam_config_accepts_every_a3_scan_and_uses_long_walls():
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "slamtec_slam_toolbox.yaml").read_text()
    )
    params = config["slam_toolbox"]["ros__parameters"]

    assert params["odom_frame"] == "odom"
    assert params["map_frame"] == "map"
    assert params["base_frame"] == "base_footprint"
    assert params["scan_topic"] == "/scan"
    assert params["mode"] == "mapping"
    assert params["resolution"] == 0.03
    assert params["max_laser_range"] == 12.0
    assert params["minimum_time_interval"] <= 0.08
    assert params["minimum_travel_distance"] == 0.03
    assert params["minimum_travel_heading"] == 0.03
    assert params["do_loop_closing"] is True


def test_slamtec_rviz_requests_best_effort_scan_qos():
    mapping = (PACKAGE_ROOT / "rviz" / "slamtec_mapping.rviz").read_text()
    navigation = (PACKAGE_ROOT / "rviz" / "slamtec_navigation.rviz").read_text()

    for rviz_text in (mapping, navigation):
        assert "Class: rviz_default_plugins/LaserScan" in rviz_text
        assert "Value: /scan" in rviz_text
        assert "Reliability Policy: Best Effort" in rviz_text

    assert "/cloud_self_filtered" not in mapping
    assert "Class: nav2_rviz_plugins/GoalTool" in navigation
    assert "Class: nav2_rviz_plugins/Navigation 2" in navigation


def test_slamtec_scripts_are_safe_foreground_entrypoints():
    scripts = REPOSITORY_ROOT / "scripts"
    lidar = scripts / "start_slamtec_lidar.sh"
    mapping = scripts / "start_slamtec_mapping.sh"
    navigation = scripts / "start_slamtec_navigation.sh"

    for script in (lidar, mapping, navigation):
        assert script.exists()
        assert script.stat().st_mode & 0o111

    lidar_text = lidar.read_text()
    assert "sllidar_a3_launch.py" in lidar_text
    assert "serial_port:=" in lidar_text
    assert "sshpass" not in lidar_text
    assert "SSHPASS" not in lidar_text
    assert "password=" not in lidar_text
    assert "exec ros2 launch" in lidar_text

    mapping_text = mapping.read_text()
    navigation_text = navigation.read_text()
    assert "slamtec_mapping.launch.py" in mapping_text
    assert "slamtec_navigation.launch.py" in navigation_text
    for local_script in (mapping_text, navigation_text):
        assert local_script.index("set +u") < local_script.index(
            'source "${unitree_setup}"'
        )
        assert local_script.index('source "${workspace_setup}"') < (
            local_script.index("set -u", local_script.index("set +u"))
        )


def test_readme_documents_slamtec_start_map_save_and_navigation_order():
    readme = (REPOSITORY_ROOT / "README.md").read_text()
    commands = (
        "./scripts/start_slamtec_lidar.sh",
        "./scripts/start_slamtec_mapping.sh",
        "slamtec_room",
        "./scripts/start_slamtec_navigation.sh",
        "ros2 topic hz /scan",
        "sensor_msgs/msg/LaserScan",
    )
    for command in commands:
        assert command in readme

    assert readme.index("./scripts/start_slamtec_lidar.sh") < readme.index(
        "./scripts/start_slamtec_mapping.sh"
    )

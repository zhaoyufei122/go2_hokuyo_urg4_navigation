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


def test_hokuyo_sensors_uses_direct_scan_without_l2_projection():
    path = PACKAGE_ROOT / "launch" / "hokuyo_sensors.launch.py"
    launch_text = path.read_text()
    description = _load_launch_description(path.name)

    assert ("go2_base_nav", "planar_odom") in _node_pairs(description)
    assert ("tf2_ros", "static_transform_publisher") in _node_pairs(description)
    assert ("go2_base_nav", "scan_filter") in _node_pairs(description)
    assert "pointcloud_to_laserscan" not in launch_text
    assert "pcl_ros" not in launch_text
    assert "urg_node" not in launch_text
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
        'default_value="0.10"',
    ):
        assert expected_default in launch_text


def test_hokuyo_mapping_uses_direct_scan_slam_and_never_starts_control():
    path = PACKAGE_ROOT / "launch" / "hokuyo_mapping.launch.py"
    launch_text = path.read_text()
    description = _load_launch_description(path.name)

    assert "hokuyo_sensors.launch.py" in launch_text
    assert "online_async_launch.py" in launch_text
    assert "hokuyo_slam_toolbox.yaml" in launch_text
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


def test_hokuyo_navigation_reuses_safe_nav2_motion_limits():
    path = PACKAGE_ROOT / "launch" / "hokuyo_navigation.launch.py"
    launch_text = path.read_text()
    description = _load_launch_description(path.name)

    assert "hokuyo_sensors.launch.py" in launch_text
    assert "bringup_launch.py" in launch_text
    assert 'default_value="amcl"' in launch_text
    assert "hokuyo_slam_toolbox_localization.yaml" in launch_text
    assert "localization_slam_toolbox_node" in launch_text
    assert "lifecycle_manager_map_server" in launch_text
    assert '"/map_slam"' in launch_text
    # nav2_bringup cannot be handed our slam params (bringup_launch.py has no
    # slam_params_file argument), so slam:=True would start slam_toolbox on
    # package defaults and quietly map instead of localise. We must ask nav2
    # for the navigation stack only and run slam_toolbox ourselves.
    assert '"slam": "False"' in launch_text
    assert '"slam": "True"' not in launch_text
    assert "localization_launch.py" in launch_text
    assert '"use_localization": "False" if use_slam_toolbox else "True"' in launch_text
    assert ("go2_base_nav", "go2_cmd_vel_bridge") in _node_pairs(description)
    assert ("rviz2", "rviz2") in _node_pairs(description)
    assert '"max_linear_x": 0.4' in launch_text
    assert '"max_linear_y": 0.0' in launch_text
    assert '"max_angular_z": 0.8' in launch_text
    assert '"min_angular_z": 0.4' in launch_text
    assert '"cmd_timeout": 0.5' in launch_text

    map_arguments = [
        action
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument) and action.name == "map"
    ]
    assert len(map_arguments) == 1
    assert map_arguments[0].default_value[0].text == ""

    assert {"localization", "slam_posegraph"} <= _argument_names(description)


def test_hokuyo_slam_config_matches_urg04lx_short_range():
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "hokuyo_slam_toolbox.yaml").read_text()
    )
    params = config["slam_toolbox"]["ros__parameters"]

    assert params["odom_frame"] == "odom"
    assert params["map_frame"] == "map"
    assert params["base_frame"] == "base_footprint"
    assert params["scan_topic"] == "/scan"
    assert params["mode"] == "mapping"
    assert params["resolution"] == 0.05
    assert params["max_laser_range"] == 4.0
    assert params["scan_buffer_maximum_scan_distance"] == 4.0
    assert params["minimum_time_interval"] <= 0.2
    assert params["minimum_travel_distance"] == 0.05
    assert params["minimum_travel_heading"] == 0.087
    assert params["do_loop_closing"] is True


def test_hokuyo_scripts_are_safe_foreground_entrypoints():
    scripts = REPOSITORY_ROOT / "scripts"
    lidar = scripts / "start_hokuyo_lidar.sh"
    mapping = scripts / "start_hokuyo_mapping.sh"
    navigation = scripts / "start_hokuyo_navigation.sh"

    for script in (lidar, mapping, navigation):
        assert script.exists()
        assert script.stat().st_mode & 0o111

    lidar_text = lidar.read_text()
    assert "urg_node" in lidar_text
    assert "serial_port:=" in lidar_text
    assert "laser_frame_id:=laser" in lidar_text
    assert "scan:=/scan_raw" in lidar_text
    assert "GO2_HOKUYO_SSH_TARGET" in lidar_text
    assert "ssh -tt" in lidar_text
    assert "rmw_cyclonedds_cpp" in lidar_text
    assert "sudo date -s" in lidar_text
    assert "sshpass" not in lidar_text
    assert "SSHPASS" not in lidar_text
    assert "password=" not in lidar_text
    assert "exec ros2" in lidar_text

    mapping_text = mapping.read_text()
    navigation_text = navigation.read_text()
    assert "hokuyo_mapping.launch.py" in mapping_text
    assert "hokuyo_navigation.launch.py" in navigation_text
    for local_script in (mapping_text, navigation_text):
        assert local_script.index("set +u") < local_script.index(
            'source "${unitree_setup}"'
        )
        assert local_script.index('source "${workspace_setup}"') < (
            local_script.index("set -u", local_script.index("set +u"))
        )


def test_readme_documents_hokuyo_start_map_save_and_navigation_order():
    readme = (REPOSITORY_ROOT / "README.md").read_text()
    commands = (
        "sudo apt install ros-jazzy-urg-node",
        "./scripts/start_hokuyo_lidar.sh",
        "./scripts/start_hokuyo_mapping.sh",
        "hokuyo_room",
        "./scripts/start_hokuyo_navigation.sh",
    )
    for command in commands:
        assert command in readme

    assert readme.index("./scripts/start_hokuyo_lidar.sh") < readme.index(
        "./scripts/start_hokuyo_mapping.sh"
    )

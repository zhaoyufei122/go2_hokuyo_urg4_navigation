import importlib.util
from pathlib import Path

import yaml
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer, Node


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PACKAGE_ROOT / "config" / "rtabmap_3d.yaml"
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "mapping_3d.launch.py"
RVIZ_PATH = PACKAGE_ROOT / "rviz" / "mapping_3d.rviz"
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]


def _load_parameters(node_name):
    config = yaml.safe_load(CONFIG_PATH.read_text())
    return config[node_name]["ros__parameters"]


def _load_launch_description():
    spec = importlib.util.spec_from_file_location("mapping_3d_launch", LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def test_3d_cloud_filters_remove_self_and_preserve_environment():
    crop = _load_parameters("cloud_self_filter")
    assert crop["input_frame"] == "base_link"
    assert crop["output_frame"] == "base_link"
    assert crop["negative"] is True
    assert crop["keep_organized"] is False
    assert crop["min_x"] == -0.45
    assert crop["max_x"] == 0.45
    assert crop["min_y"] == -0.32
    assert crop["max_y"] == 0.32
    assert crop["min_z"] == -0.45
    assert crop["max_z"] == 0.30

    voxel = _load_parameters("cloud_voxel_filter")
    assert voxel["input_frame"] == "base_link"
    assert voxel["output_frame"] == "base_link"
    assert voxel["leaf_size"] == 0.08


def test_rtabmap_uses_3d_lidar_and_full_tf_odometry():
    parameters = _load_parameters("rtabmap")

    for disabled_input in (
        "subscribe_depth",
        "subscribe_rgb",
        "subscribe_rgbd",
        "subscribe_stereo",
        "subscribe_scan",
        "subscribe_odom_info",
    ):
        assert parameters[disabled_input] is False
    assert parameters["subscribe_scan_cloud"] is True

    assert parameters["frame_id"] == "base_link"
    assert parameters["odom_frame_id"] == "odom"
    assert parameters["map_frame_id"] == "map_3d"
    assert parameters["publish_tf"] is True
    assert parameters["qos_scan"] == 2


def test_rtabmap_3d_icp_loop_closure_and_grid_parameters():
    parameters = _load_parameters("rtabmap")

    assert parameters["Reg/Strategy"] == "1"
    assert parameters["Reg/Force3DoF"] == "false"
    assert parameters["Icp/PointToPlane"] == "true"
    assert parameters["Icp/VoxelSize"] == "0.08"
    assert parameters["Icp/MaxCorrespondenceDistance"] == "0.30"
    assert parameters["RGBD/ProximityBySpace"] == "true"
    assert parameters["RGBD/NeighborLinkRefining"] == "true"
    assert parameters["Rtabmap/DetectionRate"] == "1.0"

    assert parameters["RGBD/CreateOccupancyGrid"] == "true"
    assert parameters["Grid/3D"] == "true"
    assert parameters["Grid/Sensor"] == "0"
    assert parameters["Grid/CellSize"] == "0.08"
    assert parameters["Grid/RangeMin"] == "0.25"
    assert parameters["Grid/RangeMax"] == "8.0"
    assert parameters["Grid/NormalsSegmentation"] == "false"
    assert parameters["Grid/MaxGroundHeight"] == "0.08"
    assert parameters["Grid/RayTracing"] == "true"


def test_rtabmap_viz_matches_cloud_and_tf_inputs():
    parameters = _load_parameters("rtabmap_viz")
    assert parameters["subscribe_scan_cloud"] is True
    assert parameters["subscribe_scan"] is False
    assert parameters["subscribe_depth"] is False
    assert parameters["subscribe_rgb"] is False
    assert parameters["frame_id"] == "base_link"
    assert parameters["odom_frame_id"] == "odom"
    assert parameters["qos_scan"] == 2


def test_3d_mapping_rviz_shows_live_accumulated_projected_and_path_data():
    config = yaml.safe_load(RVIZ_PATH.read_text())
    manager = config["Visualization Manager"]
    displays = {display["Name"]: display for display in manager["Displays"]}

    assert manager["Global Options"]["Fixed Frame"] == "map_3d"
    assert displays["Ground Grid"]["Class"] == "rviz_default_plugins/Grid"
    assert displays["TF"]["Class"] == "rviz_default_plugins/TF"

    live_cloud = displays["Live Filtered Cloud"]
    assert live_cloud["Topic"]["Value"] == "/cloud_3d_filtered"
    assert live_cloud["Topic"]["Reliability Policy"] == "Best Effort"
    assert live_cloud["Color"] == "255; 170; 0"
    assert live_cloud["Size (Pixels)"] == 4

    accumulated_cloud = displays["Accumulated 3D Map"]
    assert accumulated_cloud["Topic"]["Value"] == "/cloud_map"
    assert accumulated_cloud["Topic"]["Reliability Policy"] == "Reliable"
    assert accumulated_cloud["Color"] == "120; 220; 255"
    assert accumulated_cloud["Size (Pixels)"] == 2

    projected_map = displays["Projected 2D Map"]
    assert projected_map["Topic"]["Value"] == "/map"
    assert projected_map["Topic"]["Durability Policy"] == "Transient Local"

    assert displays["Mapping Path"]["Topic"]["Value"] == "/mapPath"


def test_3d_mapping_launch_declares_operator_inputs_and_nodes():
    description = _load_launch_description()
    argument_names = {
        action.name
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    assert argument_names == {
        "cloud_topic",
        "database_path",
        "new_map",
        "robot_odom_topic",
        "use_rtabmap_viz",
        "use_rviz",
        "use_sim_time",
    }

    node_pairs = [
        (action.node_package, action.node_executable)
        for action in description.entities
        if isinstance(action, Node)
    ]
    assert node_pairs.count(("go2_base_nav", "planar_odom")) == 1
    assert node_pairs.count(("rclcpp_components", "component_container_mt")) == 1
    assert node_pairs.count(("rtabmap_slam", "rtabmap")) == 2
    assert node_pairs.count(("rtabmap_viz", "rtabmap_viz")) == 1
    assert node_pairs.count(("rviz2", "rviz2")) == 1

    containers = [
        action
        for action in description.entities
        if isinstance(action, ComposableNodeContainer)
    ]
    assert len(containers) == 1


def test_3d_mapping_launch_defaults_to_rviz_operator_view():
    launch_text = LAUNCH_PATH.read_text()
    assert 'DeclareLaunchArgument("use_rviz", default_value="true")' in launch_text
    assert (
        'DeclareLaunchArgument("use_rtabmap_viz", default_value="false")'
        in launch_text
    )
    assert '"-d", rviz_config' in launch_text
    assert "condition=IfCondition(use_rviz)" in launch_text


def test_3d_mapping_launch_wires_filter_chain_and_database_modes():
    launch_text = LAUNCH_PATH.read_text()
    for required_text in (
        "pcl_ros::CropBox",
        "pcl_ros::VoxelGrid",
        "/utlidar/cloud_deskewed",
        "/cloud_3d_cropped",
        "/cloud_3d_filtered",
        '"scan_cloud", filtered_cloud_topic',
        "--delete_db_on_start",
        "IfCondition(new_map)",
        "UnlessCondition(new_map)",
        "/home/yufei/Desktop/go2_base_navi/maps/room_3d.db",
    ):
        assert required_text in launch_text


def test_3d_mapping_launch_contains_no_motion_or_2d_navigation_stack():
    launch_text = LAUNCH_PATH.read_text()
    for forbidden_text in (
        "sensors.launch.py",
        "pointcloud_to_laserscan",
        "nav2_bringup",
        "cmd_vel",
        "go2_cmd_vel_bridge",
        "/api/sport/request",
    ):
        assert forbidden_text not in launch_text


def test_3d_database_artifacts_are_ignored():
    gitignore = (REPOSITORY_ROOT / ".gitignore").read_text()
    assert "maps/*.db" in gitignore

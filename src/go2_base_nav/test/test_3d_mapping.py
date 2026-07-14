from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PACKAGE_ROOT / "config" / "rtabmap_3d.yaml"


def _load_parameters(node_name):
    config = yaml.safe_load(CONFIG_PATH.read_text())
    return config[node_name]["ros__parameters"]


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

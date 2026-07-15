from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _load_sensor_config():
    return yaml.safe_load(
        (PACKAGE_ROOT / "config" / "pointcloud_to_laserscan.yaml").read_text()
    )


def test_cloud_self_filter_removes_go2_body_in_base_link():
    params = _load_sensor_config()["cloud_self_filter"]["ros__parameters"]
    assert params["input_frame"] == "base_link"
    assert params["output_frame"] == "base_link"
    assert params["negative"] is True
    assert params["keep_organized"] is False
    assert params["min_x"] == -0.45
    assert params["max_x"] == 0.45
    assert params["min_y"] == -0.32
    assert params["max_y"] == 0.32
    assert params["min_z"] == -0.45
    assert params["max_z"] == 0.30


def test_pointcloud_projection_uses_stable_indoor_structure_window():
    params = _load_sensor_config()["pointcloud_to_laserscan"][
        "ros__parameters"
    ]
    assert params["target_frame"] == "base_footprint"
    assert params["min_height"] == 0.12
    assert params["max_height"] == 0.45
    assert params["range_min"] == 0.25
    assert params["range_max"] == 6.0
    assert params["angle_min"] == -3.141592653589793
    assert params["angle_max"] == 3.141592653589793
    assert params["angle_increment"] == 0.008726646259971648
    assert params["queue_size"] == 1
    assert params["use_inf"] is True

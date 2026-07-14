from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_pointcloud_projection_uses_indoor_navigation_window():
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "pointcloud_to_laserscan.yaml").read_text()
    )
    params = config["pointcloud_to_laserscan"]["ros__parameters"]
    assert params["target_frame"] == "base_footprint"
    assert params["min_height"] == 0.05
    assert params["max_height"] == 0.55
    assert params["range_min"] == 0.25
    assert params["range_max"] == 8.0
    assert params["angle_increment"] == 0.008726646259971648
    assert params["queue_size"] == 1
    assert params["use_inf"] is True

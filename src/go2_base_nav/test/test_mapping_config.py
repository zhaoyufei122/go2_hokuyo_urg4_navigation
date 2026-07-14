from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_slam_toolbox_uses_planar_real_robot_frames():
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "slam_toolbox.yaml").read_text()
    )
    params = config["slam_toolbox"]["ros__parameters"]
    assert params["use_sim_time"] is False
    assert params["odom_frame"] == "odom"
    assert params["map_frame"] == "map"
    assert params["base_frame"] == "base_footprint"
    assert params["scan_topic"] == "/scan"
    assert params["mode"] == "mapping"
    assert params["resolution"] == 0.05
    assert params["max_laser_range"] == 8.0
    assert params["transform_publish_period"] == 0.02
    assert params["minimum_travel_distance"] == 0.10
    assert params["minimum_travel_heading"] == 0.10
    assert params["do_loop_closing"] is True

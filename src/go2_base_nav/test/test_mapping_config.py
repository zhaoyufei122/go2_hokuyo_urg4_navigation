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
    assert params["max_laser_range"] == 6.0
    assert params["transform_publish_period"] == 0.02
    assert params["map_update_interval"] == 1.0
    assert params["minimum_time_interval"] == 0.15
    assert params["minimum_travel_distance"] == 0.05
    assert params["minimum_travel_heading"] == 0.05
    assert params["check_min_dist_and_heading_precisely"] is True
    assert params["scan_buffer_size"] == 10
    assert params["scan_buffer_maximum_scan_distance"] == 6.0
    assert params["link_match_minimum_response_fine"] == 0.20
    assert params["link_scan_maximum_distance"] == 1.5
    assert params["loop_search_maximum_distance"] == 2.0
    assert params["do_loop_closing"] is True
    assert params["loop_match_minimum_chain_size"] == 10
    assert params["loop_match_minimum_response_coarse"] == 0.45
    assert params["loop_match_minimum_response_fine"] == 0.55

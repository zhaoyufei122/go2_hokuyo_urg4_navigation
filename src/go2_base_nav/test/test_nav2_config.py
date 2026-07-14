from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
FOOTPRINT = "[[0.40, 0.25], [0.40, -0.25], [-0.40, -0.25], [-0.40, 0.25]]"


def _config():
    return yaml.safe_load((PACKAGE_ROOT / "config" / "nav2_params.yaml").read_text())


def test_navigation_frames_controller_and_speed_limits():
    config = _config()
    amcl = config["amcl"]["ros__parameters"]
    controller = config["controller_server"]["ros__parameters"]
    smoother = config["velocity_smoother"]["ros__parameters"]
    collision = config["collision_monitor"]["ros__parameters"]

    assert amcl["base_frame_id"] == "base_footprint"
    assert amcl["odom_frame_id"] == "odom"
    assert amcl["global_frame_id"] == "map"
    assert amcl["set_initial_pose"] is False
    assert amcl["always_reset_initial_pose"] is False
    assert controller["FollowPath"]["plugin"] == (
        "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
    )
    assert controller["FollowPath"]["desired_linear_vel"] == 0.4
    assert controller["FollowPath"]["min_approach_linear_velocity"] == 0.3
    assert controller["FollowPath"]["regulated_linear_scaling_min_speed"] == 0.3
    assert controller["FollowPath"]["rotate_to_heading_angular_vel"] == 0.6
    assert controller["FollowPath"]["use_cost_regulated_linear_velocity_scaling"] is False
    assert controller["FollowPath"]["allow_reversing"] is False
    assert controller["general_goal_checker"]["xy_goal_tolerance"] == 0.25
    assert controller["general_goal_checker"]["yaw_goal_tolerance"] == 0.25
    assert smoother["max_velocity"] == [0.4, 0.0, 0.6]
    assert smoother["min_velocity"] == [-0.4, 0.0, -0.6]
    assert smoother["velocity_timeout"] == 0.5
    assert collision["source_timeout"] == 0.5
    assert collision["StopZone"]["action_type"] == "stop"
    assert collision["StopZone"]["min_points"] == 4
    assert collision["StopZone"]["points"] == (
        "[[0.60, 0.35], [0.60, -0.35], [-0.50, -0.35], [-0.50, 0.35]]"
    )
    assert collision["scan"]["topic"] == "/scan"


def test_costmaps_use_exact_footprint_and_scan_obstacles():
    config = _config()
    for costmap_name in ("local_costmap", "global_costmap"):
        params = config[costmap_name][costmap_name]["ros__parameters"]
        assert params["robot_base_frame"] == "base_footprint"
        assert params["footprint"] == FOOTPRINT
        assert params["resolution"] == 0.05
        obstacle = params["obstacle_layer"]
        assert obstacle["observation_sources"] == "scan"
        assert obstacle["scan"]["topic"] == "/scan"
        assert obstacle["scan"]["data_type"] == "LaserScan"
        assert params["inflation_layer"]["inflation_radius"] == 0.35
        assert params["inflation_layer"]["cost_scaling_factor"] == 5.0


def test_recovery_rotation_respects_go2_limit():
    behavior = _config()["behavior_server"]["ros__parameters"]
    assert behavior["robot_base_frame"] == "base_footprint"
    assert behavior["max_rotational_vel"] == 0.6
    assert behavior["min_rotational_vel"] == 0.4

from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
FOOTPRINT = "[[0.35, 0.20], [0.35, -0.20], [-0.35, -0.20], [-0.35, 0.20]]"
# scan_filter publishes range_max = 4.0 and replaces everything outside
# (0.06, 4.0) m with +inf.
SCAN_RANGE_MAX = 4.0


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
    assert controller["FollowPath"]["rotate_to_heading_angular_vel"] == 0.8
    # 2026-08-25: 开启障碍减速——切弯扫进膨胀区（转角半径 v/w 太大）的修复
    assert controller["FollowPath"]["use_cost_regulated_linear_velocity_scaling"] is True
    # 2026-08-19: 巡逻必须正向（点2→点3曾被倒开）；倒开拖行走 task_planner
    # 自己的倒车控制器（NAVIGATE_TO_POINT reverse 模式），不依赖 Nav2 倒开。
    # 要恢复 Nav2 倒开（拐弯倒车实验）把配置改回 true 即可。
    assert controller["FollowPath"]["allow_reversing"] is False
    assert controller["general_goal_checker"]["xy_goal_tolerance"] == 0.25
    assert controller["general_goal_checker"]["yaw_goal_tolerance"] == 0.25
    assert smoother["max_velocity"] == [0.4, 0.0, 0.8]
    assert smoother["min_velocity"] == [-0.4, 0.0, -0.8]
    assert smoother["velocity_timeout"] == 0.5
    assert collision["source_timeout"] == 0.5
    assert collision["StopZone"]["action_type"] == "slowdown"  # 2026-08-25 pic30：stop 把 BackUp 恢复也卡死
    assert collision["StopZone"]["min_points"] == 4
    assert collision["StopZone"]["points"] == (
        "[[0.45, 0.28], [0.45, -0.28], [-0.40, -0.28], [-0.40, 0.28]]"
    )
    assert collision["scan"]["topic"] == "/scan"


def test_costmaps_use_exact_footprint_and_scan_obstacles():
    config = _config()
    for costmap_name in ("local_costmap", "global_costmap"):
        params = config[costmap_name][costmap_name]["ros__parameters"]
        assert params["robot_base_frame"] == "base_footprint"
        # 2026-08-25 pic31：圆形 footprint 太保守堵死缝隙，改回矩形。
        # 旋转扫膨胀区不是致命的（膨胀=软代价），防撞由 collision_monitor 底底
        assert params["footprint"] == FOOTPRINT
        assert params["resolution"] == 0.05
        obstacle = params["obstacle_layer"]
        assert obstacle["observation_sources"] == "scan"
        assert obstacle["scan"]["topic"] == "/scan"
        assert obstacle["scan"]["data_type"] == "LaserScan"
        # 2026-08-24：0.30→0.20（walker 旁有障碍时 0.30 膨胀吃掉可通行空间；
        # 用户评：0.15 太激进，取 0.20 折中）
        assert params["inflation_layer"]["inflation_radius"] == 0.30
        # 2026-08-25：5.0→2.5，代价衰减更慢 → 路径被推离墙更远（贴墙问题）
        assert params["inflation_layer"]["cost_scaling_factor"] == 1.5


def test_costmaps_clear_the_scan_filter_infinities():
    """+inf beams must clear the ray without marking a ring at range_max."""
    config = _config()
    for costmap_name in ("local_costmap", "global_costmap"):
        params = config[costmap_name][costmap_name]["ros__parameters"]
        scan = params["obstacle_layer"]["scan"]
        # Without this the ObstacleLayer drops every +inf beam, so phantom
        # obstacles are never ray-cleared.
        assert scan["inf_is_valid"] is True
        assert scan["clearing"] is True
        assert scan["marking"] is True
        # inf becomes range_max - 1e-4, so clearing must reach past it and
        # marking must stop short of it.
        assert scan["raytrace_max_range"] >= SCAN_RANGE_MAX
        assert scan["obstacle_max_range"] < SCAN_RANGE_MAX
        assert scan["obstacle_min_range"] < scan["obstacle_max_range"]
        assert scan["raytrace_min_range"] < scan["raytrace_max_range"]


def test_amcl_does_not_inject_random_particles():
    """Augmented-MCL recovery never converges on a trotting quadruped."""
    amcl = _config()["amcl"]["ros__parameters"]
    assert amcl["recovery_alpha_fast"] == 0.0
    assert amcl["recovery_alpha_slow"] == 0.0
    # AMCL uses min(laser_max_range, scan.range_max); a larger value here just
    # hides the real limit.
    assert amcl["laser_max_range"] == SCAN_RANGE_MAX


def test_recovery_rotation_respects_go2_limit():
    behavior = _config()["behavior_server"]["ros__parameters"]
    assert behavior["robot_base_frame"] == "base_footprint"
    assert behavior["max_rotational_vel"] == 0.6
    assert behavior["min_rotational_vel"] == 0.4

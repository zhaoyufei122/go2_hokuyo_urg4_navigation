from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]


def test_readme_contains_complete_operator_commands_in_source_order():
    readme = (REPOSITORY_ROOT / "README.md").read_text()
    commands = (
        "sudo apt install ros-jazzy-pointcloud-to-laserscan",
        "colcon build --symlink-install",
        "ip -brief address show enp130s0",
        "ros2 launch go2_base_nav mapping.launch.py",
        "ros2 run nav2_map_server map_saver_cli",
        "ros2 launch go2_base_nav navigation.launch.py",
        "ros2 topic hz /scan",
        "ros2 run tf2_ros tf2_echo odom base_footprint",
    )
    for command in commands:
        assert command in readme

    unitree_source = "source /home/yufei/Desktop/unitree_ros2/setup.sh"
    workspace_source = (
        "source /home/yufei/Desktop/go2_base_navi/install/setup.bash"
    )
    assert unitree_source in readme
    assert workspace_source in readme
    assert readme.index(unitree_source) < readme.index(workspace_source)


def test_testing_runbook_covers_real_robot_safety_cases():
    testing = (REPOSITORY_ROOT / "docs" / "TESTING.md").read_text()
    for required_text in (
        "0.4 m/s",
        "0.4 rad/s",
        "实体遥控器",
        "椅子",
        "/scan",
        "StopMove",
    ):
        assert required_text in testing

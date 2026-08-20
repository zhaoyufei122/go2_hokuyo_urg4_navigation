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
        "0.4-0.8 rad/s",
        "实体遥控器",
        "椅子",
        "/scan",
        "StopMove",
    ):
        assert required_text in testing


def test_readme_documents_complete_3d_mapping_workflow():
    readme = (REPOSITORY_ROOT / "README.md").read_text()
    for required_text in (
        "sudo apt install ros-jazzy-rtabmap-ros ros-jazzy-pcl-ros",
        "mapping_3d.launch.py",
        "/home/yufei/Desktop/go2_base_navi/maps/room_3d.db",
        "new_map:=true",
        "new_map:=false",
        "/cloud_3d_filtered",
        "[-0.45, 0.45]",
        "[-0.32, 0.32]",
        "[-0.45, 0.30]",
        "先调整机身裁剪框",
        "实体遥控器",
        "Ctrl-C",
        "本版不做 3D 定位或自主导航",
        "默认打开配置好的 RViz",
        "/cloud_map",
        "/mapPath",
        "Fixed Frame",
        "map_3d",
        "use_rviz:=false",
        "use_rtabmap_viz:=true",
    ):
        assert required_text in readme


def test_testing_runbook_covers_3d_mapping_acceptance():
    testing = (REPOSITORY_ROOT / "docs" / "TESTING.md").read_text()
    for required_text in (
        "3D 建图验收",
        "/cloud_3d_filtered",
        "机身点簇",
        "桌面和椅腿",
        "双墙",
        "room_3d.db",
        "没有软件运动指令",
        "Accumulated 3D Map",
        "Live Filtered Cloud",
        "Projected 2D Map",
        "Mapping Path",
    ):
        assert required_text in testing


def test_docs_cover_accurate_planar_mapping_and_offline_replay():
    readme = (REPOSITORY_ROOT / "README.md").read_text()
    testing = (REPOSITORY_ROOT / "docs" / "TESTING.md").read_text()

    for required_text in (
        "record_bag:=true",
        "~/go2_mapping_bags",
        "/cloud_self_filtered",
        "0.12--0.45 m",
        "0.25--6.0 m",
        "ros2 bag play",
        "--clock",
        "record_bag:=false",
        "/tf_static",
        "room_map",
    ):
        assert required_text in readme

    for required_text in (
        "静止 30 秒",
        "1--2 个 5 cm 栅格",
        "0.10 m",
        "扇形拖影",
        "/cloud_self_filtered",
        "rosbag",
        "实体遥控器",
        "没有软件运动指令",
    ):
        assert required_text in testing

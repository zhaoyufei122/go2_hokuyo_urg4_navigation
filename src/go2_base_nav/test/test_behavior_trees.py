from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
TREE_NAMES = (
    "navigate_to_pose_go2.xml",
    "navigate_through_poses_go2.xml",
)


@pytest.mark.parametrize("tree_name", TREE_NAMES)
def test_recovery_backup_uses_go2_minimum_walking_speed(tree_name):
    root = ET.parse(PACKAGE_ROOT / "behavior_trees" / tree_name).getroot()
    backups = root.findall(".//BackUp")

    assert len(backups) == 1
    assert backups[0].attrib["backup_dist"] == "0.20"
    assert backups[0].attrib["backup_speed"] == "0.30"


def test_navigation_launch_rewrites_both_default_behavior_trees():
    launch_text = (PACKAGE_ROOT / "launch" / "navigation.launch.py").read_text()

    assert "RewrittenYaml" in launch_text
    assert "bt_navigator.ros__parameters.default_nav_to_pose_bt_xml" in launch_text
    assert (
        "bt_navigator.ros__parameters.default_nav_through_poses_bt_xml"
        in launch_text
    )
    for tree_name in TREE_NAMES:
        assert tree_name in launch_text


def test_setup_installs_behavior_trees():
    setup_text = (PACKAGE_ROOT / "setup.py").read_text()

    assert 'glob("behavior_trees/*.xml")' in setup_text

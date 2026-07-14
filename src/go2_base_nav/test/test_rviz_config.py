from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_navigation_rviz_connects_goal_tool_to_nav2_action_panel():
    rviz_text = (PACKAGE_ROOT / "rviz" / "navigation.rviz").read_text()

    assert "Class: nav2_rviz_plugins/GoalTool" in rviz_text
    assert "Class: nav2_rviz_plugins/Navigation 2" in rviz_text

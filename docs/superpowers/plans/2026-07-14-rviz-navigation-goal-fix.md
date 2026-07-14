# GO2 RViz Navigation Goal Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the GO2 navigation RViz goal tool submit a real Nav2 `NavigateToPose` action.

**Architecture:** Keep the existing GO2-specific RViz displays and add Nav2's `Navigation 2` panel beside the existing goal tool. The panel owns the action client that forwards a goal-tool pose to `/navigate_to_pose`; a focused text-based regression test prevents either half of that pair from being removed.

**Tech Stack:** ROS 2 Jazzy, Nav2 RViz plugins, RViz YAML configuration, pytest, colcon

## Global Constraints

- Do not change controller gains, velocity limits, collision monitoring, mapping, localization, or the Unitree command bridge.
- Do not send an automated navigation goal during offline verification.
- Preserve the existing GO2 map, scan, costmap, path, and footprint displays.
- Stop all old mapping, sensors, and navigation launch instances before the later live test so only one sensor pipeline remains.

---

### Task 1: Connect the RViz Goal Tool to Nav2

**Files:**
- Create: `src/go2_base_nav/test/test_rviz_config.py`
- Modify: `src/go2_base_nav/rviz/navigation.rviz:1-4`

**Interfaces:**
- Consumes: RViz plugin classes `nav2_rviz_plugins/GoalTool` and `nav2_rviz_plugins/Navigation 2` supplied by the installed `nav2_rviz_plugins` package.
- Produces: `navigation.rviz` with a goal-selection tool and a panel that submits `nav2_msgs/action/NavigateToPose` goals.

- [ ] **Step 1: Write the failing RViz regression test**

```python
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_navigation_rviz_connects_goal_tool_to_nav2_action_panel():
    rviz_text = (PACKAGE_ROOT / "rviz" / "navigation.rviz").read_text()

    assert "Class: nav2_rviz_plugins/GoalTool" in rviz_text
    assert "Class: nav2_rviz_plugins/Navigation 2" in rviz_text
```

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pytest src/go2_base_nav/test/test_rviz_config.py -q
```

Expected: one failure on the missing `Class: nav2_rviz_plugins/Navigation 2` assertion.

- [ ] **Step 3: Add the minimal Navigation 2 panel**

Change the opening `Panels` block to exactly:

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: nav2_rviz_plugins/Navigation 2
    Name: Navigation 2
```

Do not replace the rest of the RViz file with Nav2's default configuration.

- [ ] **Step 4: Run the focused test and verify GREEN**

Run:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pytest src/go2_base_nav/test/test_rviz_config.py -q
```

Expected: `1 passed`.

- [ ] **Step 5: Run the complete package test suite**

Run:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pytest src/go2_base_nav/test -q
```

Expected: all tests pass with no failures or errors.

- [ ] **Step 6: Build the package**

Run:

```bash
source /home/yufei/Desktop/unitree_ros2/setup.sh
colcon build --symlink-install --packages-select go2_base_nav
```

Expected: `go2_base_nav` finishes successfully.

- [ ] **Step 7: Verify the installed RViz asset**

Run:

```bash
rg -n "nav2_rviz_plugins/Navigation 2|nav2_rviz_plugins/GoalTool" \
  install/go2_base_nav/share/go2_base_nav/rviz/navigation.rviz
```

Expected: one match for the Navigation 2 panel and one match for the GoalTool.

- [ ] **Step 8: Commit the verified fix**

```bash
git add src/go2_base_nav/test/test_rviz_config.py \
  src/go2_base_nav/rviz/navigation.rviz
git commit -m "fix: connect RViz goals to Nav2"
```

### Task 2: Perform the Guarded Live Handoff

**Files:**
- No repository files change.

**Interfaces:**
- Consumes: the rebuilt `navigation.rviz`, saved map YAML, `/scan`, AMCL pose, and the running Nav2 action servers.
- Produces: evidence that a clicked RViz goal reaches `/navigate_to_pose`, followed by downstream velocity evidence only if planning succeeds.

- [ ] **Step 1: Stop duplicate launch instances**

In every existing mapping, sensors, and navigation terminal, press `Ctrl-C`.

Expected: a subsequent navigation launch creates exactly one `/planar_odom` and one `/pointcloud_to_laserscan` node.

- [ ] **Step 2: Start one navigation launch with the saved map**

```bash
source /home/yufei/Desktop/unitree_ros2/setup.sh
source /home/yufei/Desktop/go2_base_navi/install/setup.bash
ros2 launch go2_base_nav navigation.launch.py \
  map:=/home/yufei/Desktop/go2_base_navi/maps/room_map.yaml
```

Expected: RViz shows a `Navigation 2` panel and the localization/navigation indicators become active.

- [ ] **Step 3: Initialize localization without commanding motion**

Use `2D Pose Estimate` at the robot's physical pose and heading. Confirm `/scan` overlays the saved walls and furniture before continuing.

Expected: `map -> base_footprint` resolves continuously and the displayed scan agrees with the map.

- [ ] **Step 4: Submit one guarded short goal**

With the physical controller in hand, use `Nav2 Goal` for a roughly 1 m target in open space.

Expected: `/navigate_to_pose/_action/status` reports an active goal and `/plan` publishes a path. If either is absent, do not test motion; collect the navigation terminal error instead.

- [ ] **Step 5: Trace downstream commands and stop on the first fault**

Read `/cmd_vel_nav`, `/cmd_vel_smoothed`, `/cmd_vel`, and `/api/sport/request` while the short goal is active.

Expected: nonzero controller output reaches each safety stage and becomes a Unitree Move request. If localization, scan alignment, planned direction, or physical motion is wrong, use the controller to stop immediately.

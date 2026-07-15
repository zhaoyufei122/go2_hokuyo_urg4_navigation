# GO2 Accurate Planar Mapping Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a repeatable high-accuracy 2D mapping pipeline that removes GO2 self/ground returns, constrains motion to `x/y/yaw`, uses stricter SLAM Toolbox matching, records one reusable mapping bag, and exposes every relevant stage in RViz.

**Architecture:** The shared `sensors.launch.py` pipeline will transform raw Unitree input into planar odometry and a body-cropped 360-degree LaserScan, so both mapping and navigation consume identical clean sensor data. SLAM Toolbox remains the 2D mapper with denser local scan matching and conservative loop closure. A small pure-Python utility will select a unique timestamped rosbag path, while `mapping.launch.py` conditionally starts recording without adding any motion process.

**Tech Stack:** ROS 2 Jazzy, Python launch, `pcl_ros::CropBox`, `pointcloud_to_laserscan`, SLAM Toolbox, rosbag2 CLI, RViz2, pytest, YAML, colcon.

## Global Constraints

- The physical environment is one finite flat floor with walls, fixed cabinets, tables, and chairs; walls and fixed cabinets are the primary localization anchors.
- Mapping pose is strictly `x`, `y`, and `yaw`; `z`, `roll`, and `pitch` must not enter the SLAM Toolbox pose graph.
- Keep the existing planar odometry chain `/utlidar/robot_odom -> /odom -> odom -> base_footprint`.
- Keep 360-degree environment coverage; do not switch to a front-only scan.
- Remove the body box in `base_link`: x `[-0.45, 0.45]` m, y `[-0.32, 0.32]` m, z `[-0.45, 0.30]` m, `negative=true`.
- Project only height `0.12--0.45` m and range `0.25--6.0` m in `base_footprint`.
- Keep map resolution at `0.05` m and LaserScan angular increment at `0.5` degrees.
- Mapping must never start Nav2, `go2_cmd_vel_bridge`, or publish `/api/sport/request`; physical movement is by the handheld remote only.
- Operator motion remains about `0.3--0.4 m/s` linear and `0.4--0.6 rad/s` angular.
- Do not change AMCL, Nav2 controller/costmap parameters, or the existing RTAB-Map 3D pipeline in this plan.
- `record_bag` defaults to `true`; `bag_output_root` defaults to `~/go2_mapping_bags`; each recording uses a unique `YYYYMMDD_HHMMSS` directory.

---

## File Structure

- Modify `src/go2_base_nav/config/pointcloud_to_laserscan.yaml`: own both the 2D CropBox parameters and the clean LaserScan projection window.
- Modify `src/go2_base_nav/launch/sensors.launch.py`: insert one CropBox component between the raw point cloud and `pointcloud_to_laserscan`.
- Modify `src/go2_base_nav/config/slam_toolbox.yaml`: apply the approved scan density and conservative loop-closure parameters.
- Create `src/go2_base_nav/go2_base_nav/mapping_bag.py`: hold the exact topic tuple and deterministic unique bag-output selection.
- Modify `src/go2_base_nav/launch/mapping.launch.py`: declare recording arguments and conditionally launch `ros2 bag record`.
- Modify `src/go2_base_nav/package.xml`: declare the local `ros2bag` runtime dependency.
- Modify `src/go2_base_nav/rviz/mapping.rviz`: show the body-filtered cloud instead of presenting the raw cloud as the mapping input.
- Modify focused tests in `src/go2_base_nav/test/`: verify each config, launch, utility, dependency, RViz, and documentation contract.
- Modify `README.md` and `docs/TESTING.md`: document one-command recording, offline replay, route order, and measurable acceptance checks.

---

### Task 1: Shared GO2 Self-Filter and Stable LaserScan Band

**Files:**
- Modify: `src/go2_base_nav/test/test_sensor_config.py`
- Modify: `src/go2_base_nav/test/test_launch_files.py`
- Modify: `src/go2_base_nav/config/pointcloud_to_laserscan.yaml`
- Modify: `src/go2_base_nav/launch/sensors.launch.py`

**Interfaces:**
- Consumes: `/utlidar/cloud_deskewed`, `/utlidar/robot_odom`, and the existing `planar_odom` executable.
- Produces: `/cloud_self_filtered` in `base_link`, `/scan` in `base_footprint`, `/odom`, and the existing odometry TF chain.

- [ ] **Step 1: Write failing configuration tests**

Replace `src/go2_base_nav/test/test_sensor_config.py` with:

```python
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _load_sensor_config():
    return yaml.safe_load(
        (PACKAGE_ROOT / "config" / "pointcloud_to_laserscan.yaml").read_text()
    )


def test_cloud_self_filter_removes_go2_body_in_base_link():
    params = _load_sensor_config()["cloud_self_filter"]["ros__parameters"]
    assert params["input_frame"] == "base_link"
    assert params["output_frame"] == "base_link"
    assert params["negative"] is True
    assert params["keep_organized"] is False
    assert params["min_x"] == -0.45
    assert params["max_x"] == 0.45
    assert params["min_y"] == -0.32
    assert params["max_y"] == 0.32
    assert params["min_z"] == -0.45
    assert params["max_z"] == 0.30


def test_pointcloud_projection_uses_stable_indoor_structure_window():
    params = _load_sensor_config()["pointcloud_to_laserscan"][
        "ros__parameters"
    ]
    assert params["target_frame"] == "base_footprint"
    assert params["min_height"] == 0.12
    assert params["max_height"] == 0.45
    assert params["range_min"] == 0.25
    assert params["range_max"] == 6.0
    assert params["angle_min"] == -3.141592653589793
    assert params["angle_max"] == 3.141592653589793
    assert params["angle_increment"] == 0.008726646259971648
    assert params["queue_size"] == 1
    assert params["use_inf"] is True
```

Extend `test_sensors_launch_starts_planar_odom_and_projection()` in
`src/go2_base_nav/test/test_launch_files.py` with these assertions:

```python
    launch_text = (PACKAGE_ROOT / "launch" / "sensors.launch.py").read_text()
    for required_text in (
        "pcl_ros::CropBox",
        "/cloud_self_filtered",
        '("input", cloud_topic)',
        '("output", filtered_cloud_topic)',
        '("cloud_in", filtered_cloud_topic)',
    ):
        assert required_text in launch_text
```

- [ ] **Step 2: Run the focused tests and verify failure**

Run:

```bash
python3 -m pytest \
  src/go2_base_nav/test/test_sensor_config.py \
  src/go2_base_nav/test/test_launch_files.py::test_sensors_launch_starts_planar_odom_and_projection \
  -q
```

Expected: FAIL because `cloud_self_filter` is missing and `sensors.launch.py` does not contain the CropBox chain.

- [ ] **Step 3: Add the filter parameters and projection window**

Replace `src/go2_base_nav/config/pointcloud_to_laserscan.yaml` with:

```yaml
cloud_self_filter:
  ros__parameters:
    input_frame: base_link
    output_frame: base_link
    max_queue_size: 5
    use_indices: false
    negative: true
    keep_organized: false
    min_x: -0.45
    max_x: 0.45
    min_y: -0.32
    max_y: 0.32
    min_z: -0.45
    max_z: 0.30

pointcloud_to_laserscan:
  ros__parameters:
    target_frame: base_footprint
    transform_tolerance: 0.10
    min_height: 0.12
    max_height: 0.45
    angle_min: -3.141592653589793
    angle_max: 3.141592653589793
    angle_increment: 0.008726646259971648
    queue_size: 1
    scan_time: 0.06666666666666667
    range_min: 0.25
    range_max: 6.0
    use_inf: true
```

- [ ] **Step 4: Wire CropBox before projection**

In `src/go2_base_nav/launch/sensors.launch.py`, add these imports:

```python
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
```

Define the fixed filtered output next to the launch configurations:

```python
    filtered_cloud_topic = "/cloud_self_filtered"
```

Insert this action after `planar_odom` and before `pointcloud_to_laserscan`:

```python
            ComposableNodeContainer(
                name="cloud_2d_filters",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                output="screen",
                composable_node_descriptions=[
                    ComposableNode(
                        package="pcl_ros",
                        plugin="pcl_ros::CropBox",
                        name="cloud_self_filter",
                        parameters=[
                            projection_config,
                            {"use_sim_time": use_sim_time},
                        ],
                        remappings=[
                            ("input", cloud_topic),
                            ("output", filtered_cloud_topic),
                        ],
                    )
                ],
            ),
```

Change the projection input remapping to:

```python
                    ("cloud_in", filtered_cloud_topic),
```

- [ ] **Step 5: Run the focused tests and verify pass**

Run the Step 2 command again.

Expected: `3 passed` (the sensor file contains two tests and the selected launch test is the third).

- [ ] **Step 6: Commit the shared sensor pipeline**

```bash
git add src/go2_base_nav/config/pointcloud_to_laserscan.yaml \
  src/go2_base_nav/launch/sensors.launch.py \
  src/go2_base_nav/test/test_sensor_config.py \
  src/go2_base_nav/test/test_launch_files.py
git commit -m "feat: filter GO2 self points from planar scan"
```

---

### Task 2: Denser Local Matching and Conservative Loop Closure

**Files:**
- Modify: `src/go2_base_nav/test/test_mapping_config.py`
- Modify: `src/go2_base_nav/config/slam_toolbox.yaml`

**Interfaces:**
- Consumes: `/scan`, `odom -> base_footprint`, and SLAM Toolbox's existing Ceres solver.
- Produces: `map -> odom` and `/map` at 0.05 m resolution.

- [ ] **Step 1: Write the failing SLAM parameter test**

Replace the final assertions in `test_slam_toolbox_uses_planar_real_robot_frames()` after `resolution` with:

```python
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
```

- [ ] **Step 2: Run the test and verify failure**

Run:

```bash
python3 -m pytest src/go2_base_nav/test/test_mapping_config.py -q
```

Expected: FAIL first at `max_laser_range`, which is still `8.0`.

- [ ] **Step 3: Apply the approved SLAM values**

In `src/go2_base_nav/config/slam_toolbox.yaml`, set these exact values and leave all unlisted matcher parameters unchanged:

```yaml
    map_update_interval: 1.0
    resolution: 0.05
    max_laser_range: 6.0
    minimum_time_interval: 0.15

    minimum_travel_distance: 0.05
    minimum_travel_heading: 0.05
    check_min_dist_and_heading_precisely: true
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 6.0
    link_match_minimum_response_fine: 0.20
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 2.0
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_minimum_response_coarse: 0.45
    loop_match_minimum_response_fine: 0.55
```

- [ ] **Step 4: Run the mapping configuration test and verify pass**

Run the Step 2 command again.

Expected: `1 passed`.

- [ ] **Step 5: Commit the SLAM tuning**

```bash
git add src/go2_base_nav/config/slam_toolbox.yaml \
  src/go2_base_nav/test/test_mapping_config.py
git commit -m "tune: tighten planar SLAM matching"
```

---

### Task 3: Timestamped Mapping Bag Recording

**Files:**
- Create: `src/go2_base_nav/test/test_mapping_bag.py`
- Create: `src/go2_base_nav/go2_base_nav/mapping_bag.py`
- Modify: `src/go2_base_nav/test/test_launch_files.py`
- Modify: `src/go2_base_nav/test/test_package_metadata.py`
- Modify: `src/go2_base_nav/launch/mapping.launch.py`
- Modify: `src/go2_base_nav/package.xml`

**Interfaces:**
- Consumes: `bag_output_root: str`, optional `datetime`, and the mapping pipeline's raw/processed topics.
- Produces: `BAG_TOPICS: tuple[str, ...]`, `select_bag_output(root, now) -> Path`, and `build_bag_record_command(root, now) -> list[str]`.

- [ ] **Step 1: Write failing pure-Python recording tests**

Create `src/go2_base_nav/test/test_mapping_bag.py`:

```python
from datetime import datetime

from go2_base_nav.mapping_bag import (
    BAG_TOPICS,
    build_bag_record_command,
    select_bag_output,
)


EXPECTED_TOPICS = (
    "/utlidar/cloud_deskewed",
    "/utlidar/robot_odom",
    "/cloud_self_filtered",
    "/scan",
    "/map",
    "/tf",
    "/tf_static",
)


def test_select_bag_output_expands_root_and_avoids_existing_directory(tmp_path):
    now = datetime(2026, 7, 15, 12, 34, 56)
    first = tmp_path / "20260715_123456"
    first.mkdir()

    assert select_bag_output(tmp_path, now) == tmp_path / "20260715_123456_01"


def test_build_bag_record_command_contains_exact_topics_and_unique_output(tmp_path):
    now = datetime(2026, 7, 15, 12, 34, 56)

    command = build_bag_record_command(tmp_path, now)

    assert BAG_TOPICS == EXPECTED_TOPICS
    assert command == [
        "ros2",
        "bag",
        "record",
        "-o",
        str(tmp_path / "20260715_123456"),
        *EXPECTED_TOPICS,
    ]
```

- [ ] **Step 2: Run the utility tests and verify import failure**

Run:

```bash
python3 -m pytest src/go2_base_nav/test/test_mapping_bag.py -q
```

Expected: collection ERROR with `ModuleNotFoundError: No module named 'go2_base_nav.mapping_bag'`.

- [ ] **Step 3: Implement the bag utility**

Create `src/go2_base_nav/go2_base_nav/mapping_bag.py`:

```python
from datetime import datetime
from pathlib import Path


BAG_TOPICS = (
    "/utlidar/cloud_deskewed",
    "/utlidar/robot_odom",
    "/cloud_self_filtered",
    "/scan",
    "/map",
    "/tf",
    "/tf_static",
)


def select_bag_output(
    root: str | Path,
    now: datetime | None = None,
) -> Path:
    root_path = Path(root).expanduser()
    root_path.mkdir(parents=True, exist_ok=True)
    stem = (now or datetime.now()).strftime("%Y%m%d_%H%M%S")
    candidate = root_path / stem
    suffix = 1
    while candidate.exists():
        candidate = root_path / f"{stem}_{suffix:02d}"
        suffix += 1
    return candidate


def build_bag_record_command(
    root: str | Path,
    now: datetime | None = None,
) -> list[str]:
    output = select_bag_output(root, now)
    return ["ros2", "bag", "record", "-o", str(output), *BAG_TOPICS]
```

- [ ] **Step 4: Run the utility tests and verify pass**

Run the Step 2 command again.

Expected: `2 passed`.

- [ ] **Step 5: Add failing launch and package contracts**

In `test_mapping_launch_includes_sensors_and_async_slam_without_control()` in
`src/go2_base_nav/test/test_launch_files.py`, add:

```python
    argument_names = {
        action.name
        for action in description.entities
        if isinstance(action, DeclareLaunchArgument)
    }
    assert {"record_bag", "bag_output_root"} <= argument_names
    for required_text in (
        "OpaqueFunction",
        "ExecuteProcess",
        "build_bag_record_command",
        'DeclareLaunchArgument("record_bag", default_value="true")',
        'default_value="~/go2_mapping_bags"',
    ):
        assert required_text in launch_text
```

Add `"ros2bag",` to the expected runtime dependency set in
`src/go2_base_nav/test/test_package_metadata.py`.

- [ ] **Step 6: Run launch and package tests and verify failure**

Run:

```bash
python3 -m pytest \
  src/go2_base_nav/test/test_launch_files.py::test_mapping_launch_includes_sensors_and_async_slam_without_control \
  src/go2_base_nav/test/test_package_metadata.py::test_package_declares_runtime_dependencies \
  -q
```

Expected: both tests FAIL because mapping recording arguments/actions and `ros2bag` are absent.

- [ ] **Step 7: Wire conditional bag recording into mapping launch**

In `src/go2_base_nav/launch/mapping.launch.py`, extend imports and add the helper:

```python
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)

from go2_base_nav.mapping_bag import build_bag_record_command


def _start_bag_recording(context):
    output_root = LaunchConfiguration("bag_output_root").perform(context)
    return [
        ExecuteProcess(
            cmd=build_bag_record_command(output_root),
            output="screen",
        )
    ]
```

Inside `generate_launch_description()`, define:

```python
    record_bag = LaunchConfiguration("record_bag")
```

Add these declarations after the existing launch arguments:

```python
            DeclareLaunchArgument("record_bag", default_value="true"),
            DeclareLaunchArgument(
                "bag_output_root",
                default_value="~/go2_mapping_bags",
            ),
```

Add this action after the sensor and SLAM includes and before RViz:

```python
            OpaqueFunction(
                function=_start_bag_recording,
                condition=IfCondition(record_bag),
            ),
```

Add this line to `src/go2_base_nav/package.xml` beside the other runtime tools:

```xml
  <exec_depend>ros2bag</exec_depend>
```

- [ ] **Step 8: Run all Task 3 tests and verify pass**

Run:

```bash
python3 -m pytest \
  src/go2_base_nav/test/test_mapping_bag.py \
  src/go2_base_nav/test/test_launch_files.py::test_mapping_launch_includes_sensors_and_async_slam_without_control \
  src/go2_base_nav/test/test_package_metadata.py::test_package_declares_runtime_dependencies \
  -q
```

Expected: `4 passed`.

- [ ] **Step 9: Commit timestamped bag recording**

```bash
git add src/go2_base_nav/go2_base_nav/mapping_bag.py \
  src/go2_base_nav/launch/mapping.launch.py \
  src/go2_base_nav/package.xml \
  src/go2_base_nav/test/test_mapping_bag.py \
  src/go2_base_nav/test/test_launch_files.py \
  src/go2_base_nav/test/test_package_metadata.py
git commit -m "feat: record timestamped planar mapping bags"
```

---

### Task 4: RViz View of the Actual Filtered Mapping Input

**Files:**
- Create: `src/go2_base_nav/test/test_mapping_rviz.py`
- Modify: `src/go2_base_nav/rviz/mapping.rviz`

**Interfaces:**
- Consumes: `/cloud_self_filtered`, `/scan`, `/map`, `/map_updates`, and TF.
- Produces: a mapping RViz view whose enabled displays correspond to the actual SLAM input chain.

- [ ] **Step 1: Write the failing RViz contract**

Create `src/go2_base_nav/test/test_mapping_rviz.py`:

```python
from pathlib import Path

import yaml


RVIZ_PATH = Path(__file__).resolve().parents[1] / "rviz" / "mapping.rviz"


def test_mapping_rviz_shows_filtered_cloud_scan_map_and_tf():
    config = yaml.safe_load(RVIZ_PATH.read_text())
    manager = config["Visualization Manager"]
    displays = {display["Name"]: display for display in manager["Displays"]}

    assert manager["Global Options"]["Fixed Frame"] == "map"
    assert displays["TF"]["Class"] == "rviz_default_plugins/TF"
    assert displays["Filtered Cloud"]["Topic"]["Value"] == "/cloud_self_filtered"
    assert (
        displays["Filtered Cloud"]["Topic"]["Reliability Policy"]
        == "Best Effort"
    )
    assert displays["Scan"]["Topic"]["Value"] == "/scan"
    assert displays["Map"]["Topic"]["Value"] == "/map"
    assert "Deskewed Cloud" not in displays
```

- [ ] **Step 2: Run the RViz test and verify failure**

Run:

```bash
python3 -m pytest src/go2_base_nav/test/test_mapping_rviz.py -q
```

Expected: FAIL with `KeyError: 'Filtered Cloud'`.

- [ ] **Step 3: Point the cloud display at the filter output**

In `src/go2_base_nav/rviz/mapping.rviz`, replace the `Deskewed Cloud` display name and topic block with:

```yaml
    - Class: rviz_default_plugins/PointCloud2
      Name: Filtered Cloud
      Enabled: true
      Topic:
        Depth: 1
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /cloud_self_filtered
      Style: Points
      Size (Pixels): 2
      Decay Time: 0.2
```

- [ ] **Step 4: Run the RViz test and verify pass**

Run the Step 2 command again.

Expected: `1 passed`.

- [ ] **Step 5: Commit the operator view**

```bash
git add src/go2_base_nav/rviz/mapping.rviz \
  src/go2_base_nav/test/test_mapping_rviz.py
git commit -m "feat: show filtered cloud in planar mapping RViz"
```

---

### Task 5: Accurate Mapping and Offline Replay Runbook

**Files:**
- Modify: `src/go2_base_nav/test/test_documentation.py`
- Modify: `README.md`
- Modify: `docs/TESTING.md`

**Interfaces:**
- Consumes: the mapping launch arguments, bag topic list, filter output, and SLAM acceptance thresholds from Tasks 1--4.
- Produces: exact live mapping, save, stop, bag discovery, and offline replay commands for the operator.

- [ ] **Step 1: Add failing documentation assertions**

Append this test to `src/go2_base_nav/test/test_documentation.py`:

```python
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
```

- [ ] **Step 2: Run the documentation test and verify failure**

Run:

```bash
python3 -m pytest \
  src/go2_base_nav/test/test_documentation.py::test_docs_cover_accurate_planar_mapping_and_offline_replay \
  -q
```

Expected: FAIL because the current 2D mapping section does not describe filtered cloud recording or replay.

- [ ] **Step 3: Replace the README 2D mapping workflow**

Update README section `## 3. 建图` so it includes these exact commands and facts:

```bash
ros2 launch go2_base_nav mapping.launch.py record_bag:=true
```

Explain that raw input is body-cropped to `/cloud_self_filtered`, then projected from height `0.12--0.45 m` and range `0.25--6.0 m`; 360-degree wall/cabinet structure is preserved. State that each bag is under `~/go2_mapping_bags/YYYYMMDD_HHMMSS`, mapping sends no software motion command, and the route order is stationary check, outer closed loop, inner aisles, then return to the start.

Keep the existing map save command unchanged:

```bash
ros2 run nav2_map_server map_saver_cli -f /home/yufei/Desktop/go2_base_navi/maps/room_map
```

Add the exact offline replay workflow:

```bash
ros2 launch go2_base_nav mapping.launch.py \
  use_sim_time:=true use_rviz:=true record_bag:=false
```

In another sourced terminal:

```bash
ros2 bag play ~/go2_mapping_bags/YYYYMMDD_HHMMSS \
  --clock \
  --topics /utlidar/cloud_deskewed /utlidar/robot_odom /tf_static
```

Explain that `/tf` is deliberately excluded during reprocessing because `planar_odom` and SLAM Toolbox regenerate dynamic TF.

- [ ] **Step 4: Extend real-robot mapping acceptance**

In `docs/TESTING.md`, update sections A and B to require `/cloud_self_filtered` inspection before movement, verify the bag directory is created, and state these exact pass criteria:

- static for `静止 30 秒` without a growing wall;
- no body cluster in `/cloud_self_filtered`;
- no `扇形拖影` after a controlled rotation;
- principal walls remain `1--2 个 5 cm 栅格` thick after loop closure;
- no persistent double wall separated by more than `0.10 m`;
- rosbag metadata is finalized after Ctrl-C;
- the process contains `没有软件运动指令` and movement uses the `实体遥控器`.

- [ ] **Step 5: Run documentation tests and verify pass**

Run:

```bash
python3 -m pytest src/go2_base_nav/test/test_documentation.py -q
```

Expected: all documentation tests PASS.

- [ ] **Step 6: Commit the operator runbook**

```bash
git add README.md docs/TESTING.md \
  src/go2_base_nav/test/test_documentation.py
git commit -m "docs: add accurate planar mapping workflow"
```

---

### Task 6: Full Static Verification and Launch Smoke Test

**Files:**
- Verify only; modify a prior task's files only if a failing check exposes a defect in that task.

**Interfaces:**
- Consumes: all deliverables from Tasks 1--5.
- Produces: a clean build, passing test suite, valid installed launch arguments, and evidence that mapping starts without a motion process.

- [ ] **Step 1: Run source-tree tests**

```bash
python3 -m pytest src/go2_base_nav/test -q
```

Expected: all tests PASS with no failure or collection error.

- [ ] **Step 2: Check formatting and patch integrity**

```bash
python3 -m compileall -q \
  src/go2_base_nav/go2_base_nav \
  src/go2_base_nav/launch \
  src/go2_base_nav/test
git diff --check
```

Expected: both commands exit 0 with no output.

- [ ] **Step 3: Build the workspace**

```bash
source /home/yufei/Desktop/unitree_ros2/setup.sh
cd /home/yufei/Desktop/go2_base_navi
colcon build --symlink-install
source /home/yufei/Desktop/go2_base_navi/install/setup.bash
```

Expected: `go2_base_nav` finishes successfully.

- [ ] **Step 4: Run colcon tests**

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Expected: summary contains zero failures and zero errors.

- [ ] **Step 5: Verify installed launch arguments**

```bash
ros2 launch go2_base_nav mapping.launch.py --show-args
```

Expected: output lists `use_rviz`, `use_sim_time`, `record_bag`, and `bag_output_root`; `record_bag` defaults to `true` and `bag_output_root` to `~/go2_mapping_bags`.

- [ ] **Step 6: Run a no-hardware launch smoke test with recording disabled**

```bash
timeout 8s ros2 launch go2_base_nav mapping.launch.py \
  use_rviz:=false record_bag:=false
```

Expected: the process reaches the wait-for-sensor state; logs show `planar_odom`, `cloud_2d_filters`, `pointcloud_to_laserscan`, and SLAM Toolbox. Logs must not contain `go2_cmd_vel_bridge`, Nav2, or `/api/sport/request`. Timeout exit 124 is acceptable because the launch is intentionally long-running.

- [ ] **Step 7: Confirm repository state and recent commits**

```bash
git status --short
git log --oneline -6
```

Expected: no uncommitted implementation files; recent commits correspond to Tasks 1--5 and the plan/design documents.

---

## Real-Hardware Handoff

After static verification, do not start the GO2 motion bridge. The operator will start:

```bash
ros2 launch go2_base_nav mapping.launch.py record_bag:=true
```

The first real run stops immediately if `/cloud_self_filtered` contains a robot-following cluster, if `/scan` forms a near-body ring, or if stationary walls move. Only after the 30-second stationary and controlled-turn checks pass should the operator drive the outer loop with the handheld remote.

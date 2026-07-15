# GO2 3D Mapping RViz Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the existing motion-free GO2 3D mapping command open a preconfigured RViz view that shows the live scan, accumulated 3D map, projected 2D map, TF, and mapping path.

**Architecture:** Add one installed RViz asset and start it conditionally from the existing `mapping_3d.launch.py`. RViz becomes the default operator interface while `rtabmap_viz` remains an opt-in graph-debugging tool; RTAB-Map, filtering, database, and safety behavior stay unchanged.

**Tech Stack:** ROS 2 Jazzy launch, RViz2, RTAB-Map ROS 0.22.1, Python/pytest, YAML

## Global Constraints

- RViz fixed frame is exactly `map_3d`.
- Display `/cloud_3d_filtered`, `/cloud_map`, `/map`, and `/mapPath`.
- `use_rviz` defaults to `true`; `use_rtabmap_viz` defaults to `false`.
- Mapping remains motion-free: no Nav2, command bridge, teleoperation node, `/cmd_vel`, or `/api/sport/request`.
- Robot movement during mapping is performed only with the physical remote.
- Do not change RTAB-Map ICP, point-cloud crop, voxel, database, or fresh/resume behavior.
- Closing RViz does not stop RTAB-Map; Ctrl-C in the launch terminal remains the database-safe shutdown.

---

## File structure

- Create `src/go2_base_nav/rviz/mapping_3d.rviz`: the complete 3D mapping operator view.
- Modify `src/go2_base_nav/launch/mapping_3d.launch.py`: declare GUI arguments and conditionally launch RViz.
- Modify `src/go2_base_nav/test/test_3d_mapping.py`: test the RViz asset and launch integration.
- Modify `src/go2_base_nav/test/test_documentation.py`: test the one-command operator documentation.
- Modify `README.md`: describe the default RViz displays and optional specialist view.
- Modify `docs/TESTING.md`: define the live RViz acceptance check.

### Task 1: Add the dedicated 3D mapping RViz view

**Files:**
- Create: `src/go2_base_nav/rviz/mapping_3d.rviz`
- Modify: `src/go2_base_nav/test/test_3d_mapping.py`

**Interfaces:**
- Consumes: RTAB-Map topics `/cloud_map`, `/map`, `/mapPath`; filter topic `/cloud_3d_filtered`; TF frame `map_3d`.
- Produces: installed RViz config path `share/go2_base_nav/rviz/mapping_3d.rviz`, already covered by `setup.py`'s `glob("rviz/*.rviz")`.

- [ ] **Step 1: Write the failing RViz asset test**

Add this constant beside the existing path constants in
`src/go2_base_nav/test/test_3d_mapping.py`:

```python
RVIZ_PATH = PACKAGE_ROOT / "rviz" / "mapping_3d.rviz"
```

Add this test before the launch tests:

```python
def test_3d_mapping_rviz_shows_live_accumulated_projected_and_path_data():
    config = yaml.safe_load(RVIZ_PATH.read_text())
    manager = config["Visualization Manager"]
    displays = {display["Name"]: display for display in manager["Displays"]}

    assert manager["Global Options"]["Fixed Frame"] == "map_3d"
    assert displays["Ground Grid"]["Class"] == "rviz_default_plugins/Grid"
    assert displays["TF"]["Class"] == "rviz_default_plugins/TF"

    live_cloud = displays["Live Filtered Cloud"]
    assert live_cloud["Topic"]["Value"] == "/cloud_3d_filtered"
    assert live_cloud["Topic"]["Reliability Policy"] == "Best Effort"
    assert live_cloud["Color"] == "255; 170; 0"
    assert live_cloud["Size (Pixels)"] == 4

    accumulated_cloud = displays["Accumulated 3D Map"]
    assert accumulated_cloud["Topic"]["Value"] == "/cloud_map"
    assert accumulated_cloud["Topic"]["Reliability Policy"] == "Reliable"
    assert accumulated_cloud["Color"] == "120; 220; 255"
    assert accumulated_cloud["Size (Pixels)"] == 2

    projected_map = displays["Projected 2D Map"]
    assert projected_map["Topic"]["Value"] == "/map"
    assert projected_map["Topic"]["Durability Policy"] == "Transient Local"

    assert displays["Mapping Path"]["Topic"]["Value"] == "/mapPath"
```

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
python3 -m pytest -q src/go2_base_nav/test/test_3d_mapping.py::test_3d_mapping_rviz_shows_live_accumulated_projected_and_path_data
```

Expected: FAIL with `FileNotFoundError` for `rviz/mapping_3d.rviz`.

- [ ] **Step 3: Create the complete RViz config**

Create `src/go2_base_nav/rviz/mapping_3d.rviz` with:

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Ground Grid
      Enabled: true
      Plane: XY
      Cell Size: 1
      Plane Cell Count: 20
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
      Frame Timeout: 15
      Marker Scale: 0.5
      Show Arrows: true
      Show Axes: true
      Show Names: false
    - Class: rviz_default_plugins/PointCloud2
      Name: Live Filtered Cloud
      Enabled: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /cloud_3d_filtered
      Style: Points
      Size (Pixels): 4
      Decay Time: 0.3
      Color Transformer: FlatColor
      Color: 255; 170; 0
    - Class: rviz_default_plugins/PointCloud2
      Name: Accumulated 3D Map
      Enabled: true
      Topic:
        Depth: 1
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /cloud_map
      Style: Points
      Size (Pixels): 2
      Decay Time: 0
      Color Transformer: FlatColor
      Color: 120; 220; 255
    - Class: rviz_default_plugins/Map
      Name: Projected 2D Map
      Enabled: true
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map
      Alpha: 0.55
      Color Scheme: map
      Draw Behind: true
    - Class: rviz_default_plugins/Path
      Name: Mapping Path
      Enabled: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /mapPath
      Buffer Length: 1
      Color: 25; 255; 0
      Line Style: Lines
      Line Width: 0.03
  Enabled: true
  Global Options:
    Fixed Frame: map_3d
    Frame Rate: 30
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 8
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Pitch: 0.65
      Yaw: 0.8
Window Geometry:
  Height: 900
  Width: 1400
```

- [ ] **Step 4: Run the focused test and verify GREEN**

Run the focused test from Step 2.

Expected: `1 passed`.

- [ ] **Step 5: Commit the RViz asset**

```bash
git add src/go2_base_nav/rviz/mapping_3d.rviz src/go2_base_nav/test/test_3d_mapping.py
git commit -m "feat: add 3D mapping RViz view"
```

### Task 2: Launch RViz by default and keep RTAB-Map visualization optional

**Files:**
- Modify: `src/go2_base_nav/launch/mapping_3d.launch.py`
- Modify: `src/go2_base_nav/test/test_3d_mapping.py`

**Interfaces:**
- Consumes: `rviz/mapping_3d.rviz` from Task 1 and existing `use_sim_time`.
- Produces: launch argument `use_rviz: bool` default `true`; changes `use_rtabmap_viz: bool` default to `false`.

- [ ] **Step 1: Write failing launch integration assertions**

Update the expected launch argument set in
`test_3d_mapping_launch_declares_operator_inputs_and_nodes()` to:

```python
    assert argument_names == {
        "cloud_topic",
        "database_path",
        "new_map",
        "robot_odom_topic",
        "use_rtabmap_viz",
        "use_rviz",
        "use_sim_time",
    }
```

Add these node assertions:

```python
    assert node_pairs.count(("rviz2", "rviz2")) == 1
    assert node_pairs.count(("rtabmap_viz", "rtabmap_viz")) == 1
```

Add this separate defaults test:

```python
def test_3d_mapping_launch_defaults_to_rviz_operator_view():
    launch_text = LAUNCH_PATH.read_text()
    assert 'DeclareLaunchArgument("use_rviz", default_value="true")' in launch_text
    assert (
        'DeclareLaunchArgument("use_rtabmap_viz", default_value="false")'
        in launch_text
    )
    assert '"-d", rviz_config' in launch_text
    assert "condition=IfCondition(use_rviz)" in launch_text
```

- [ ] **Step 2: Run the launch tests and verify RED**

Run:

```bash
python3 -m pytest -q \
  src/go2_base_nav/test/test_3d_mapping.py::test_3d_mapping_launch_declares_operator_inputs_and_nodes \
  src/go2_base_nav/test/test_3d_mapping.py::test_3d_mapping_launch_defaults_to_rviz_operator_view
```

Expected: FAIL because `use_rviz` and the RViz node are absent and
`use_rtabmap_viz` still defaults to `true`.

- [ ] **Step 3: Add the conditional RViz node**

In `generate_launch_description()`, add:

```python
    rviz_config = str(package_root / "rviz" / "mapping_3d.rviz")
```

Add the launch configuration beside the other GUI configuration:

```python
    use_rviz = LaunchConfiguration("use_rviz")
```

Add this node after `rtabmap_viz`:

```python
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )
```

Replace the GUI argument declarations with:

```python
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "use_rtabmap_viz",
                default_value="false",
            ),
```

Append `rviz` to the returned `LaunchDescription` after
`rtabmap_viz`.

- [ ] **Step 4: Run all 3D mapping tests and verify GREEN**

Run:

```bash
python3 -m pytest -q src/go2_base_nav/test/test_3d_mapping.py
```

Expected: all tests pass, including the existing motion-stack exclusion test.

- [ ] **Step 5: Commit launch integration**

```bash
git add src/go2_base_nav/launch/mapping_3d.launch.py src/go2_base_nav/test/test_3d_mapping.py
git commit -m "feat: launch RViz for 3D mapping"
```

### Task 3: Document the one-command live mapping workflow

**Files:**
- Modify: `src/go2_base_nav/test/test_documentation.py`
- Modify: `README.md`
- Modify: `docs/TESTING.md`

**Interfaces:**
- Consumes: the launch arguments and display topics from Tasks 1 and 2.
- Produces: copy-paste mapping command and acceptance criteria for the operator.

- [ ] **Step 1: Add failing documentation assertions**

Append these requirements to
`test_readme_documents_complete_3d_mapping_workflow()`:

```python
        "默认打开配置好的 RViz",
        "/cloud_map",
        "/mapPath",
        "Fixed Frame",
        "map_3d",
        "use_rviz:=false",
        "use_rtabmap_viz:=true",
```

Append these requirements to
`test_testing_runbook_covers_3d_mapping_acceptance()`:

```python
        "Accumulated 3D Map",
        "Live Filtered Cloud",
        "Projected 2D Map",
        "Mapping Path",
```

- [ ] **Step 2: Run the documentation tests and verify RED**

Run:

```bash
python3 -m pytest -q \
  src/go2_base_nav/test/test_documentation.py::test_readme_documents_complete_3d_mapping_workflow \
  src/go2_base_nav/test/test_documentation.py::test_testing_runbook_covers_3d_mapping_acceptance
```

Expected: FAIL on the first newly required RViz text.

- [ ] **Step 3: Update the README**

Immediately after the existing `new_map:=true` launch command, add:

```markdown
这个命令默认打开配置好的 RViz。Fixed Frame 已设为 `map_3d`：

- `Live Filtered Cloud`（橙色）显示当前帧 `/cloud_3d_filtered`；
- `Accumulated 3D Map`（青色）显示不断增长和回环优化的 `/cloud_map`；
- `Projected 2D Map` 显示同步生成的 `/map`；
- `Mapping Path` 显示 `/mapPath` 建图轨迹。

只做无界面诊断时增加 `use_rviz:=false`。需要检查 RTAB-Map 节点图和
回环细节时增加 `use_rtabmap_viz:=true`；默认不打开这个专业调试窗口。
```

- [ ] **Step 4: Update the testing runbook**

In the 3D mapping acceptance section, add:

```markdown
默认 RViz 的 Fixed Frame 必须为 `map_3d`。确认四个预配置显示均正常：

- `Live Filtered Cloud` 随机器狗移动并只短暂保留当前扫描；
- `Accumulated 3D Map` 随行走不断增长；
- `Projected 2D Map` 同步显示平面占用区域；
- `Mapping Path` 记录已经走过的轨迹。

回环发生时累计地图和轨迹可能整体小幅重新对齐，这是正常的图优化结果。
```

- [ ] **Step 5: Run documentation and full source tests**

Run:

```bash
python3 -m pytest -q src/go2_base_nav/test/test_documentation.py
python3 -m pytest -q src/go2_base_nav/test
```

Expected: documentation tests pass, then the complete source suite passes.

- [ ] **Step 6: Commit operator documentation**

```bash
git add README.md docs/TESTING.md src/go2_base_nav/test/test_documentation.py
git commit -m "docs: explain live 3D mapping in RViz"
```

### Task 4: Build and verify the installed one-command workflow

**Files:**
- Verify: `install/go2_base_nav/share/go2_base_nav/rviz/mapping_3d.rviz`
- Verify: `install/go2_base_nav/share/go2_base_nav/launch/mapping_3d.launch.py`

**Interfaces:**
- Consumes: all deliverables from Tasks 1-3.
- Produces: a tested installed command ready for the supervised GO2 run.

- [ ] **Step 1: Build the package**

Run in a terminal with Unitree ROS 2 and this workspace sourced:

```bash
colcon build --symlink-install --packages-select go2_base_nav
```

Expected: `Summary: 1 package finished`.

- [ ] **Step 2: Verify installed assets and launch arguments**

Run:

```bash
test -f install/go2_base_nav/share/go2_base_nav/rviz/mapping_3d.rviz
ros2 launch go2_base_nav mapping_3d.launch.py --show-args
```

Expected: the file check exits 0; launch arguments show `use_rviz` default
`true` and `use_rtabmap_viz` default `false`.

- [ ] **Step 3: Run a no-GUI, no-motion launch smoke test**

For an offline desktop smoke test only, temporarily unset the Unitree
interface-specific CycloneDDS configuration, then run:

```bash
unset CYCLONEDDS_URI
timeout --signal=INT --kill-after=3s 8s \
  ros2 launch go2_base_nav mapping_3d.launch.py \
  use_rviz:=false \
  use_rtabmap_viz:=false \
  database_path:=/tmp/go2_3d_rviz_smoke.db \
  new_map:=true
```

Expected: `planar_odom`, both PCL filters, and `rtabmap` start; no RViz,
RTAB-Map visualization, Nav2, or motion node starts; SIGINT saves the temporary
database and all processes exit cleanly. A no-data warning is acceptable when
the GO2 is disconnected.

- [ ] **Step 4: Run ROS package tests**

Run:

```bash
colcon test --packages-select go2_base_nav --event-handlers console_direct+
colcon test-result --verbose
```

Expected: every test passes with 0 errors, 0 failures, and 0 skipped.

- [ ] **Step 5: Run final repository checks**

Run:

```bash
git diff --check
git status --short
pgrep -af "mapping_3d.launch.py|rtabmap|rviz2|planar_odom|component_container_mt"
```

Expected: no whitespace errors, no uncommitted implementation files, and no
mapping processes left running.

- [ ] **Step 6: Hand off the real-robot command**

Use the Unitree interface configuration on the connected GO2; do not unset
`CYCLONEDDS_URI` for the real run:

```bash
source /home/yufei/Desktop/unitree_ros2/setup.sh
source /home/yufei/Desktop/go2_base_navi/install/setup.bash
ros2 launch go2_base_nav mapping_3d.launch.py \
  database_path:=/home/yufei/Desktop/go2_base_navi/maps/room_3d.db \
  new_map:=true
```

Expected: one RViz window opens with the four named data displays. The
operator moves the GO2 only with the physical remote and presses Ctrl-C in the
launch terminal to save the database.

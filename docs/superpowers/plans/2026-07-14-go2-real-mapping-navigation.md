# GO2 Real-Robot 2D Mapping and Navigation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (- [ ]) syntax for tracking.

**Goal:** Build a ROS 2 Jazzy workspace that maps a flat furnished room from the GO2 deskewed 3D LiDAR cloud and navigates the real robot on the saved 2D map.

**Architecture:** A Python adapter decomposes the Unitree 3D odometry into a planar odom-to-base_footprint transform plus the residual base_footprint-to-base_link body transform. The standard pointcloud_to_laserscan node gravity-levels and height-filters the deskewed cloud into /scan. SLAM Toolbox handles mapping; AMCL and Nav2 handle saved-map navigation; Nav2 velocity smoothing and collision monitoring feed a tested Unitree Sport API bridge.

**Tech Stack:** ROS 2 Jazzy, rclpy, tf2_ros, pointcloud_to_laserscan, SLAM Toolbox, Nav2, RViz2, pytest, colcon, CycloneDDS, unitree_api messages.

## Global Constraints

- Runtime terminals source /home/yufei/Desktop/unitree_ros2/setup.sh before the workspace overlay.
- LiDAR input is /utlidar/cloud_deskewed, sensor_msgs/msg/PointCloud2, frame odom.
- Odometry input is /utlidar/robot_odom, nav_msgs/msg/Odometry, frame odom, child base_link.
- The navigation base frame is base_footprint; map and odom are the global/local frames.
- Projection defaults are 0.05-0.55 m height, 0.25-8.0 m range, and 0.5 degree angular bins.
- Nonzero forward commands are tuned for 0.30-0.40 m/s; absolute linear-x and angular-z limits are 0.40; linear-y is always zero.
- The GO2 bridge timeout and collision-monitor scan-source timeout are both 0.5 s.
- The costmap footprint is [[0.40, 0.25], [0.40, -0.25], [-0.40, -0.25], [-0.40, 0.25]].
- Mapping never starts autonomous motion.
- No node sends StandUp, gait changes, or advanced Unitree actions.

## File Map

- src/go2_base_nav/go2_base_nav/pose_math.py: pure quaternion decomposition.
- src/go2_base_nav/go2_base_nav/planar_odom.py: odometry conversion, /odom, and TF publication.
- src/go2_base_nav/go2_base_nav/command_safety.py: pure clamping and watchdog state.
- src/go2_base_nav/go2_base_nav/sport_client.py: Unitree Move and StopMove request creation.
- src/go2_base_nav/go2_base_nav/cmd_vel_bridge.py: safe /cmd_vel to /api/sport/request bridge.
- src/go2_base_nav/launch/sensors.launch.py: planar odometry and 3D-to-2D projection.
- src/go2_base_nav/launch/mapping.launch.py: sensors, SLAM Toolbox, and mapping RViz.
- src/go2_base_nav/launch/navigation.launch.py: sensors, Nav2/AMCL, bridge, and navigation RViz.
- src/go2_base_nav/config/pointcloud_to_laserscan.yaml: projection band and scan geometry.
- src/go2_base_nav/config/slam_toolbox.yaml: real-robot 2D mapping parameters.
- src/go2_base_nav/config/nav2_params.yaml: AMCL, planner, controller, costmaps, smoothing, and collision stop.
- src/go2_base_nav/rviz/mapping.rviz and navigation.rviz: operator displays.
- src/go2_base_nav/test/: unit, metadata, YAML, and launch tests.
- README.md and docs/TESTING.md: exact build, map, navigation, and supervised-test commands.

---

### Task 1: ROS Package Skeleton and Metadata

**Files:**
- Create: .gitignore
- Create: src/go2_base_nav/package.xml
- Create: src/go2_base_nav/setup.py
- Create: src/go2_base_nav/setup.cfg
- Create: src/go2_base_nav/resource/go2_base_nav
- Create: src/go2_base_nav/go2_base_nav/__init__.py
- Create: src/go2_base_nav/test/test_package_metadata.py

**Interfaces:**
- Produces: an ament_python package named go2_base_nav with console scripts planar_odom and go2_cmd_vel_bridge.
- Consumes: the Unitree message overlay and the ROS 2 Jazzy underlay.

- [ ] **Step 1: Write the failing metadata test**

~~~python
from pathlib import Path
import xml.etree.ElementTree as ET

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_package_declares_runtime_dependencies():
    root = ET.parse(PACKAGE_ROOT / "package.xml").getroot()
    dependencies = {item.text for item in root.findall("exec_depend")}
    assert {
        "geometry_msgs",
        "launch",
        "launch_ros",
        "nav_msgs",
        "nav2_bringup",
        "nav2_map_server",
        "pointcloud_to_laserscan",
        "rclpy",
        "rviz2",
        "sensor_msgs",
        "slam_toolbox",
        "tf2_ros",
        "unitree_api",
    } <= dependencies


def test_setup_installs_launch_config_and_rviz_assets():
    setup_text = (PACKAGE_ROOT / "setup.py").read_text()
    assert "planar_odom = go2_base_nav.planar_odom:main" in setup_text
    assert "go2_cmd_vel_bridge = go2_base_nav.cmd_vel_bridge:main" in setup_text
    assert 'glob("launch/*.launch.py")' in setup_text
    assert 'glob("config/*.yaml")' in setup_text
    assert 'glob("rviz/*.rviz")' in setup_text
~~~

- [ ] **Step 2: Run the test and verify RED**

Run:

~~~bash
python3 -m pytest src/go2_base_nav/test/test_package_metadata.py -v
~~~

Expected: FAIL because package.xml and setup.py do not exist.

- [ ] **Step 3: Create the minimal package metadata**

package.xml must declare ament_python as build type, the dependencies asserted above, python3-pytest as a test dependency, maintainer yufei, and MIT license.

setup.py must install package.xml, launch/*.launch.py, config/*.yaml, and rviz/*.rviz, and register exactly:

~~~python
entry_points={
    "console_scripts": [
        "planar_odom = go2_base_nav.planar_odom:main",
        "go2_cmd_vel_bridge = go2_base_nav.cmd_vel_bridge:main",
    ],
}
~~~

.gitignore must contain:

~~~gitignore
build/
install/
log/
__pycache__/
*.pyc
.pytest_cache/
maps/*.pgm
maps/*.yaml
~~~

- [ ] **Step 4: Run the metadata test and verify GREEN**

Run:

~~~bash
python3 -m pytest src/go2_base_nav/test/test_package_metadata.py -v
~~~

Expected: 2 passed.

- [ ] **Step 5: Commit**

~~~bash
git add .gitignore src/go2_base_nav
git commit -m "build: scaffold GO2 navigation package"
~~~

### Task 2: Planar Odometry and TF Adapter

**Files:**
- Create: src/go2_base_nav/go2_base_nav/pose_math.py
- Create: src/go2_base_nav/go2_base_nav/planar_odom.py
- Create: src/go2_base_nav/test/test_pose_math.py
- Create: src/go2_base_nav/test/test_planar_odom.py

**Interfaces:**
- Consumes: /utlidar/robot_odom as nav_msgs/msg/Odometry with reliable, volatile, keep-last-depth-1 QoS.
- Produces: /odom and dynamic TF odom -> base_footprint -> base_link.
- Produces pure API: split_planar_orientation(full: QuaternionValue) and
  planarize_odometry(message, odom_frame, footprint_frame, base_frame).

- [ ] **Step 1: Write failing quaternion decomposition tests**

~~~python
from math import isclose, pi

from go2_base_nav.pose_math import (
    QuaternionValue,
    multiply,
    split_planar_orientation,
)


def test_split_recomposes_full_orientation():
    full = QuaternionValue(
        x=0.0843056797421489,
        y=-0.07285182744658007,
        z=0.2506948010244541,
        w=0.961632611936709,
    )
    planar, residual = split_planar_orientation(full)
    recomposed = multiply(planar, residual)
    assert isclose(recomposed.x, full.x, abs_tol=1e-9)
    assert isclose(recomposed.y, full.y, abs_tol=1e-9)
    assert isclose(recomposed.z, full.z, abs_tol=1e-9)
    assert isclose(recomposed.w, full.w, abs_tol=1e-9)


def test_planar_part_contains_yaw_only():
    full = QuaternionValue.from_rpy(0.2, -0.1, pi / 3.0)
    planar, residual = split_planar_orientation(full)
    assert isclose(planar.yaw(), pi / 3.0, abs_tol=1e-9)
    assert isclose(planar.roll(), 0.0, abs_tol=1e-9)
    assert isclose(planar.pitch(), 0.0, abs_tol=1e-9)
    recomposed = multiply(planar, residual)
    assert recomposed.is_equivalent(full, abs_tol=1e-9)
~~~

Run with PYTHONPATH set to src/go2_base_nav. Expected: import failure.

- [ ] **Step 2: Implement pose_math.py minimally**

Implement an immutable QuaternionValue dataclass with normalized(), conjugate(), roll(), pitch(), yaw(), from_rpy(), and is_equivalent(). Implement multiply(a, b) with Hamilton multiplication. Implement split_planar_orientation(full) as:

~~~python
def split_planar_orientation(full):
    normalized = full.normalized()
    planar = QuaternionValue.from_rpy(0.0, 0.0, normalized.yaw())
    residual = multiply(planar.conjugate(), normalized).normalized()
    return planar, residual
~~~

Normalization must reject non-finite values and norms below 1e-12 with ValueError.

- [ ] **Step 3: Verify pose math GREEN**

Run:

~~~bash
PYTHONPATH=src/go2_base_nav python3 -m pytest \
  src/go2_base_nav/test/test_pose_math.py -v
~~~

Expected: 2 passed.

- [ ] **Step 4: Write the failing odometry conversion test**

Create an Odometry with stamp 123.456, frame odom, child base_link, position (1.2, -0.7, 0.31), a roll/pitch/yaw quaternion, planar twist, and covariance arrays. Assert:

- output odometry frame is odom and child is base_footprint;
- output z is zero and orientation contains yaw only;
- odom_to_footprint contains x/y, zero z, and the same planar orientation;
- footprint_to_base contains zero x/y, z=0.31, and the residual orientation;
- composing both transforms recovers the original orientation;
- timestamps and covariance arrays are preserved.

Expected: import failure for planarize_odometry.

- [ ] **Step 5: Implement planar_odom.py**

The pure conversion function must have this exact signature:

~~~python
def planarize_odometry(
    message: Odometry,
    odom_frame: str = "odom",
    footprint_frame: str = "base_footprint",
    base_frame: str = "base_link",
) -> tuple[Odometry, TransformStamped, TransformStamped]:
~~~

PlanarOdomNode must declare input_odom_topic, output_odom_topic, odom_frame, footprint_frame, and base_frame parameters; create the matching QoS subscription; publish /odom; broadcast both transforms; reject invalid messages without publishing; and use the incoming timestamp for every output.

- [ ] **Step 6: Verify conversion and regression tests GREEN**

~~~bash
PYTHONPATH=src/go2_base_nav python3 -m pytest \
  src/go2_base_nav/test/test_pose_math.py \
  src/go2_base_nav/test/test_planar_odom.py -v
~~~

Expected: all tests pass.

- [ ] **Step 7: Commit**

~~~bash
git add src/go2_base_nav/go2_base_nav src/go2_base_nav/test
git commit -m "feat: publish planar GO2 odometry and TF"
~~~

### Task 3: Safe Unitree Sport API Bridge

**Files:**
- Create: src/go2_base_nav/go2_base_nav/command_safety.py
- Create: src/go2_base_nav/go2_base_nav/sport_client.py
- Create: src/go2_base_nav/go2_base_nav/cmd_vel_bridge.py
- Create: src/go2_base_nav/test/test_command_safety.py
- Create: src/go2_base_nav/test/test_sport_client.py

**Interfaces:**
- Consumes: /cmd_vel geometry_msgs/msg/Twist.
- Produces: /api/sport/request unitree_api/msg/Request.
- API IDs: Move=1008 and StopMove=1003.
- Limits: abs(linear.x)<=0.4, linear.y=0.0, abs(angular.z)<=0.4, timeout=0.5 s.

- [ ] **Step 1: Write failing command-safety tests**

Test clamp_command with oversized positive and negative inputs, lateral input, and zero input. Test WatchdogState so that it requests a stop once after 0.5 s, does not repeat the stop while stopped, and becomes armed again after a new nonzero command.

~~~python
def test_clamp_enforces_real_robot_limits():
    command = clamp_command(0.9, 0.5, -0.8, MotionLimits())
    assert command.linear_x == 0.4
    assert command.linear_y == 0.0
    assert command.angular_z == -0.4
~~~

Expected: import failure.

- [ ] **Step 2: Implement command_safety.py**

Create immutable MotionLimits and MotionCommand dataclasses, clamp_command(), is_zero_command(), and WatchdogState. WatchdogState stores last_command_nanoseconds and stopped; expired(now_nanoseconds, timeout_seconds) returns true only for the first expired moving state.

- [ ] **Step 3: Verify command-safety GREEN**

~~~bash
PYTHONPATH=src/go2_base_nav python3 -m pytest \
  src/go2_base_nav/test/test_command_safety.py -v
~~~

Expected: all command-safety tests pass.

- [ ] **Step 4: Write failing Sport request tests**

Instantiate Go2SportClient through __new__ without a ROS publisher and test make_request():

~~~python
def test_move_request_uses_compact_json():
    client = Go2SportClient.__new__(Go2SportClient)
    request = client.make_request(1008, {"x": 0.4, "y": 0.0, "z": -0.4})
    assert request.header.identity.api_id == 1008
    assert request.parameter == '{"x":0.4,"y":0.0,"z":-0.4}'
~~~

Also assert StopMove has API ID 1003 and an empty parameter.

- [ ] **Step 5: Implement sport_client.py and cmd_vel_bridge.py**

Go2SportClient must expose make_request(), move(), and stop_move(). Go2CmdVelBridge must declare cmd_vel_topic, sport_request_topic, cmd_timeout, max_linear_x=0.4, max_linear_y=0.0, and max_angular_z=0.4. It must:

- clamp every command through command_safety;
- publish Move for nonzero commands;
- publish StopMove once for zero;
- publish StopMove once after 0.5 s without a command;
- always attempt StopMove during destroy_node();
- never publish stand, gait, or advanced actions.

- [ ] **Step 6: Verify bridge tests GREEN**

~~~bash
source /home/yufei/Desktop/unitree_ros2/setup.sh
PYTHONPATH=src/go2_base_nav python3 -m pytest \
  src/go2_base_nav/test/test_command_safety.py \
  src/go2_base_nav/test/test_sport_client.py -v
~~~

Expected: all tests pass without publishing to the robot because no node is spun.

- [ ] **Step 7: Commit**

~~~bash
git add src/go2_base_nav/go2_base_nav src/go2_base_nav/test
git commit -m "feat: add safe GO2 cmd_vel bridge"
~~~

### Task 4: Deskewed Point Cloud to 2D Scan Pipeline

**Files:**
- Create: src/go2_base_nav/config/pointcloud_to_laserscan.yaml
- Create: src/go2_base_nav/launch/sensors.launch.py
- Create: src/go2_base_nav/test/test_sensor_config.py
- Create: src/go2_base_nav/test/test_launch_files.py

**Interfaces:**
- Consumes: /utlidar/cloud_deskewed and /utlidar/robot_odom.
- Produces: /scan in base_footprint plus /odom and TF.
- Depends on executable pointcloud_to_laserscan_node.

- [ ] **Step 1: Write failing sensor-config tests**

Load the YAML with yaml.safe_load and assert:

~~~python
params = config["pointcloud_to_laserscan"]["ros__parameters"]
assert params["target_frame"] == "base_footprint"
assert params["min_height"] == 0.05
assert params["max_height"] == 0.55
assert params["range_min"] == 0.25
assert params["range_max"] == 8.0
assert params["angle_increment"] == 0.008726646259971648
assert params["queue_size"] == 1
assert params["use_inf"] is True
~~~

Load sensors.launch.py with importlib and assert generate_launch_description() returns a LaunchDescription containing a go2_base_nav/planar_odom node and pointcloud_to_laserscan/pointcloud_to_laserscan_node.

Expected: missing-file failures.

- [ ] **Step 2: Create the exact projection configuration**

~~~yaml
pointcloud_to_laserscan:
  ros__parameters:
    target_frame: base_footprint
    transform_tolerance: 0.10
    min_height: 0.05
    max_height: 0.55
    angle_min: -3.141592653589793
    angle_max: 3.141592653589793
    angle_increment: 0.008726646259971648
    queue_size: 1
    scan_time: 0.06666666666666667
    range_min: 0.25
    range_max: 8.0
    use_inf: true
~~~

- [ ] **Step 3: Create sensors.launch.py**

Declare cloud_topic, robot_odom_topic, scan_topic, and use_sim_time. Start planar_odom with the requested input topic. Start pointcloud_to_laserscan_node with the YAML file and remappings cloud_in -> cloud_topic and scan -> scan_topic. Set output to screen.

- [ ] **Step 4: Verify sensor tests GREEN**

~~~bash
source /opt/ros/jazzy/setup.bash
PYTHONPATH=src/go2_base_nav python3 -m pytest \
  src/go2_base_nav/test/test_sensor_config.py \
  src/go2_base_nav/test/test_launch_files.py -v
~~~

Expected: sensor/config launch tests pass.

- [ ] **Step 5: Commit**

~~~bash
git add src/go2_base_nav/config src/go2_base_nav/launch src/go2_base_nav/test
git commit -m "feat: project GO2 point cloud to laser scan"
~~~

### Task 5: SLAM Toolbox Mapping Mode

**Files:**
- Create: src/go2_base_nav/config/slam_toolbox.yaml
- Create: src/go2_base_nav/launch/mapping.launch.py
- Create: src/go2_base_nav/rviz/mapping.rviz
- Create: src/go2_base_nav/test/test_mapping_config.py
- Create: maps/.gitkeep

**Interfaces:**
- Consumes: /scan and odom -> base_footprint.
- Produces: /map and map -> odom while mapping.
- Saves: maps/<name>.yaml and maps/<name>.pgm via map_saver_cli.

- [ ] **Step 1: Write failing mapping tests**

Assert the SLAM YAML contains use_sim_time=false, odom_frame=odom, map_frame=map, base_frame=base_footprint, scan_topic=/scan, mode=mapping, resolution=0.05, max_laser_range=8.0, transform_publish_period=0.02, and do_loop_closing=true. Assert mapping.launch.py includes sensors.launch.py and the official slam_toolbox online_async_launch.py and never starts cmd_vel_bridge.

Expected: missing-file failures.

- [ ] **Step 2: Create slam_toolbox.yaml**

Use the existing proven solver configuration from /home/yufei/Desktop/go2_navigation/src/go2_navigation_bringup/config/slam_toolbox.yaml, changing use_sim_time to false, base_frame to base_footprint, minimum_travel_distance to 0.10, minimum_travel_heading to 0.10, and preserving 0.05 m map resolution, 8.0 m range, asynchronous mapping, and loop closure.

- [ ] **Step 3: Create mapping.launch.py**

Declare use_rviz and use_sim_time. Include sensors.launch.py. Include slam_toolbox/launch/online_async_launch.py with slam_params_file and use_sim_time. Conditionally start RViz with mapping.rviz. Do not start the control bridge.

- [ ] **Step 4: Reuse and adapt the RViz asset**

Copy /home/yufei/Desktop/go2_navigation/src/go2_navigation_bringup/rviz/go2_nav.rviz to mapping.rviz. Set Fixed Frame to map and ensure Map=/map, LaserScan=/scan, TF, and PointCloud2=/utlidar/cloud_deskewed displays exist.

- [ ] **Step 5: Verify mapping tests GREEN**

~~~bash
source /opt/ros/jazzy/setup.bash
PYTHONPATH=src/go2_base_nav python3 -m pytest \
  src/go2_base_nav/test/test_mapping_config.py \
  src/go2_base_nav/test/test_launch_files.py -v
~~~

Expected: all mapping and launch assertions pass.

- [ ] **Step 6: Commit**

~~~bash
git add maps src/go2_base_nav/config src/go2_base_nav/launch \
  src/go2_base_nav/rviz src/go2_base_nav/test
git commit -m "feat: add GO2 SLAM mapping launch"
~~~

### Task 6: AMCL and Nav2 Navigation Mode

**Files:**
- Create: src/go2_base_nav/config/nav2_params.yaml
- Create: src/go2_base_nav/launch/navigation.launch.py
- Create: src/go2_base_nav/rviz/navigation.rviz
- Create: src/go2_base_nav/test/test_nav2_config.py

**Interfaces:**
- Consumes: saved map YAML, /scan, /odom, and TF.
- Produces: /cmd_vel after velocity smoothing and collision monitoring.
- The GO2 bridge converts /cmd_vel to /api/sport/request.

- [ ] **Step 1: Write failing Nav2 configuration tests**

The tests must assert all of these exact values:

~~~python
assert amcl["base_frame_id"] == "base_footprint"
assert amcl["odom_frame_id"] == "odom"
assert amcl["global_frame_id"] == "map"
assert amcl["set_initial_pose"] is False
assert controller["FollowPath"]["plugin"] == (
    "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
)
assert controller["FollowPath"]["desired_linear_vel"] == 0.4
assert controller["FollowPath"]["min_approach_linear_velocity"] == 0.3
assert controller["FollowPath"]["regulated_linear_scaling_min_speed"] == 0.3
assert controller["FollowPath"]["rotate_to_heading_angular_vel"] == 0.4
assert smoother["max_velocity"] == [0.4, 0.0, 0.4]
assert smoother["velocity_timeout"] == 0.5
assert collision["source_timeout"] == 0.5
assert collision["StopZone"]["action_type"] == "stop"
assert collision["StopZone"]["points"] == (
    "[[0.60, 0.35], [0.60, -0.35], [-0.50, -0.35], [-0.50, 0.35]]"
)
~~~

For local_costmap and global_costmap, assert base_footprint, the exact rectangular footprint, /scan obstacle sources, 0.05 m resolution, 0.25 m inflation radius, and 5.0 cost-scaling factor.

Expected: missing-file failures.

- [ ] **Step 2: Create nav2_params.yaml from the Jazzy baseline**

Copy /home/yufei/Desktop/go2_navigation/src/go2_navigation_bringup/config/nav2_params.yaml and make these exact replacements:

- every navigation robot base frame becomes base_footprint;
- remove the simulated initial pose and set AMCL set_initial_pose=false and always_reset_initial_pose=false;
- replace MPPI FollowPath with Regulated Pure Pursuit;
- desired_linear_vel=0.4;
- min_approach_linear_velocity=0.3;
- regulated_linear_scaling_min_speed=0.3;
- rotate_to_heading_angular_vel=0.4;
- use_cost_regulated_linear_velocity_scaling=false;
- allow_reversing=false;
- xy_goal_tolerance=0.25 and yaw_goal_tolerance=0.25;
- both costmaps use the exact footprint string;
- inflation radius=0.25 and cost_scaling_factor=5.0;
- velocity smoother max=[0.4,0.0,0.4], min=[-0.4,0.0,-0.4], max_accel=[0.6,0.0,0.8], max_decel=[-0.8,0.0,-1.0], timeout=0.5;
- collision monitor uses StopZone polygon, action_type=stop, min_points=4, source_timeout=0.5, scan topic=/scan;
- behavior rotational max and min are both 0.4;
- use_sim_time=false throughout.

- [ ] **Step 3: Create navigation.launch.py**

Require a nonempty map launch argument. Include sensors.launch.py. Include nav2_bringup/launch/bringup_launch.py with slam=False, use_sim_time=False, autostart=True, use_composition=False, use_respawn=False, map, and params_file. Start go2_cmd_vel_bridge with max_linear_x=0.4, max_linear_y=0.0, max_angular_z=0.4, and cmd_timeout=0.5. Conditionally start RViz with navigation.rviz.

- [ ] **Step 4: Create the navigation RViz view**

Copy mapping.rviz to navigation.rviz and ensure Nav2 Goal, 2D Pose Estimate, global plan, local plan, global costmap, local costmap, robot footprint, /scan, map, and TF displays/tools are available.

- [ ] **Step 5: Verify Nav2 tests GREEN**

~~~bash
source /opt/ros/jazzy/setup.bash
PYTHONPATH=src/go2_base_nav python3 -m pytest \
  src/go2_base_nav/test/test_nav2_config.py \
  src/go2_base_nav/test/test_launch_files.py -v
~~~

Expected: all Nav2 speed, frame, footprint, collision, and launch assertions pass.

- [ ] **Step 6: Commit**

~~~bash
git add src/go2_base_nav/config src/go2_base_nav/launch \
  src/go2_base_nav/rviz src/go2_base_nav/test
git commit -m "feat: add safe GO2 Nav2 bringup"
~~~

### Task 7: Operator Documentation, Dependency, and Full Verification

**Files:**
- Create: README.md
- Create: docs/TESTING.md
- Modify: src/go2_base_nav/package.xml if dependency checks expose an omission.

**Interfaces:**
- Produces exact build, mapping, save-map, navigation, and emergency-stop instructions.
- Produces automated build/test evidence and supervised real-robot smoke-test evidence.

- [ ] **Step 1: Write the failing documentation test**

Add test_documentation.py that asserts README.md contains all exact commands:

~~~text
sudo apt install ros-jazzy-pointcloud-to-laserscan
colcon build --symlink-install
ros2 launch go2_base_nav mapping.launch.py
ros2 run nav2_map_server map_saver_cli
ros2 launch go2_base_nav navigation.launch.py
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo odom base_footprint
~~~

Also assert both source commands appear in the correct Unitree-then-workspace order and that docs/TESTING.md contains the 0.4 limits, physical remote emergency-stop instruction, chair-obstacle test, and scan-loss StopMove test.

Expected: missing-file failures.

- [ ] **Step 2: Write README.md and docs/TESTING.md**

README sequence:

1. install pointcloud_to_laserscan;
2. source Unitree environment;
3. build and source workspace;
4. run sensors smoke checks;
5. run mapping launch and move with the physical remote;
6. save maps/room_map;
7. stop mapping;
8. run navigation launch with the absolute map YAML;
9. set initial pose and a short RViz goal.

docs/TESTING.md must give pass/fail observations for /scan, TF, stationary scan alignment, rotating scan alignment, loop closure, map reload, AMCL overlay, short goal, chair obstacle, scan loss, launch Ctrl-C, and StopMove.

- [ ] **Step 3: Verify documentation GREEN**

~~~bash
python3 -m pytest src/go2_base_nav/test/test_documentation.py -v
~~~

Expected: documentation test passes.

- [ ] **Step 4: Install the missing runtime dependency**

~~~bash
sudo apt install ros-jazzy-pointcloud-to-laserscan
~~~

Expected: pointcloud_to_laserscan installs successfully.

- [ ] **Step 5: Build and run the complete automated suite**

~~~bash
source /home/yufei/Desktop/unitree_ros2/setup.sh
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
python3 -m compileall src/go2_base_nav
~~~

Expected: build succeeds; every test passes; compileall reports no syntax errors.

- [ ] **Step 6: Run read-only live sensor smoke verification**

With the robot stationary and no motion command publisher:

~~~bash
source /home/yufei/Desktop/unitree_ros2/setup.sh
source install/setup.bash
ros2 launch go2_base_nav sensors.launch.py
~~~

In another correctly sourced terminal:

~~~bash
timeout 10s ros2 topic hz /scan
timeout 10s ros2 run tf2_ros tf2_echo odom base_footprint
timeout 10s ros2 topic echo /odom --once --no-arr
~~~

Expected: /scan receives data, TF resolves, /odom is planar, and no /api/sport/request Move command is published.

- [ ] **Step 7: Commit**

~~~bash
git add README.md docs src/go2_base_nav
git commit -m "docs: add GO2 mapping and navigation runbook"
~~~

- [ ] **Step 8: Final clean-state verification**

~~~bash
git status --short
git log --oneline --decorate -8
~~~

Expected: clean status and one focused commit per completed task.


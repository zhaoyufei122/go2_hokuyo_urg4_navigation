# GO2 Real-Robot 2D Mapping and Navigation Design

## Goal

Build a ROS 2 Jazzy workspace that lets the connected Unitree GO2:

1. create and save a 2D occupancy map of a finite, single-floor furnished room; and
2. localize in that saved map and navigate to RViz goals through Nav2.

The first version deliberately excludes 3D map storage, automatic exploration,
multi-floor navigation, and unrelated robot behaviors.

## Confirmed Environment and Interfaces

- Host ROS distribution: ROS 2 Jazzy.
- Robot transport: CycloneDDS configured by
  `/home/yufei/Desktop/unitree_ros2/setup.sh` on interface `enp130s0`.
- LiDAR input: `/utlidar/cloud_deskewed`,
  `sensor_msgs/msg/PointCloud2`, frame `odom`.
- Robot odometry input: `/utlidar/robot_odom`,
  `nav_msgs/msg/Odometry`, frame `odom`, child frame `base_link`.
- Motion command output: `/api/sport/request`,
  `unitree_api/msg/Request`.
- The existing `/cmd_vel` to Unitree Sport API bridge has already been tested
  on this robot. Its API IDs are `1008` for Move and `1003` for StopMove.
- The room is a flat, finite, single-floor indoor area containing furniture such
  as tables and chairs, with no stairs or significant slopes.
- SLAM Toolbox, Nav2, robot_localization, RViz, and Unitree message packages are
  already available. `ros-jazzy-pointcloud-to-laserscan` is available from the
  configured apt repository but is not currently installed.

Every runtime terminal must source the Unitree setup before this workspace so
that all processes use the same CycloneDDS implementation and network interface:

```bash
source /home/yufei/Desktop/unitree_ros2/setup.sh
source /home/yufei/Desktop/go2_base_navi/install/setup.bash
```

## Selected Architecture

The implementation will be one focused `ament_python` package named
`go2_base_nav`. It will contain two small Python runtime nodes, launch files,
SLAM/Nav2/projector parameters, RViz configurations, tests, and operator
instructions.

```text
/utlidar/robot_odom
        |
        v
planar odometry adapter ----> /odom
        |                     odom -> base_footprint -> base_link TF
        |
/utlidar/cloud_deskewed
        |
        v
PointCloud2 height-band projection ----> /scan
        |
        +---- mapping: SLAM Toolbox ----> /map ----> saved YAML + PGM
        |
        +---- navigation: AMCL + Nav2
                              |
                              v
                    /cmd_vel_nav
                              |
                    velocity smoother
                              |
                    collision monitor
                              |
                         /cmd_vel
                              |
                    GO2 Sport API bridge
                              |
                    /api/sport/request
```

### 1. Planar odometry and TF adapter

The robot odometry contains full 3D body height, roll, pitch, and yaw, while
SLAM Toolbox and Nav2 require a stable planar base frame. The adapter will
consume each `/utlidar/robot_odom` message using a reliable, volatile,
keep-last-depth-1 QoS profile matching the Unitree publisher.

For every valid input pose it will publish:

- `odom -> base_footprint`: `(x, y, 0)` and yaw only;
- `base_footprint -> base_link`: `(0, 0, z)` and the residual roll/pitch
  orientation after removing yaw; and
- `/odom`: a planar `nav_msgs/msg/Odometry` message whose child frame is
  `base_footprint` and whose twist retains planar x/y and yaw-rate components.

Both transforms use the original sensor timestamp. Decomposing the pose this
way preserves the measured body tilt for LiDAR leveling without exposing that
tilt to the 2D navigation base frame. Invalid or non-finite poses are rejected
and logged; stale transforms are never invented.

### 2. Gravity-aligned 3D-to-2D projection

The standard `pointcloud_to_laserscan` component will subscribe to
`/utlidar/cloud_deskewed`, transform it from `odom` into `base_footprint` at the
cloud timestamp, and publish `/scan`.

Initial tunable projection values are:

- target frame: `base_footprint`;
- minimum height above the floor: `0.05 m`;
- maximum height above the floor: `0.55 m`;
- horizontal field of view: `-pi` to `+pi`;
- angular resolution: `0.5 degrees`;
- minimum range: `0.25 m`;
- maximum range: `8.0 m`;
- input queue depth: `1` to favor fresh data over backlog.

Within every angular bin, the closest surviving 3D point becomes the 2D range.
The height band is intended to retain table legs, chair legs, chair seats, and
objects that could hit the GO2 body while rejecting the floor, ceiling, and
most high overhangs. All values remain launch/configuration parameters so the
first RViz scan can be tuned without changing code.

`cloud_deskewed` is selected because motion compensation reduces bent walls and
duplicated furniture edges while the robot walks or turns. It is still a
single 3D cloud, not a map; height filtering and angular projection happen in
this workspace.

### 3. Mapping mode

`mapping.launch.py` will start:

- the planar odometry/TF adapter;
- the PointCloud2-to-LaserScan projector;
- SLAM Toolbox in asynchronous mapping mode; and
- an RViz mapping view.

SLAM Toolbox will use `map`, `odom`, and `base_footprint`, subscribe to `/scan`,
publish `map -> odom`, and create a `0.05 m/cell` occupancy grid with an `8.0 m`
usable scan range. Mapping motion is manual; the operator may use the physical
remote or the already-tested keyboard bridge. The mapping launch itself does
not autonomously move the robot.

The operator will traverse the accessible perimeter, pass both sides of large
furniture where possible, and return near the starting area to give loop
closure a strong observation. The completed map is saved with
`nav2_map_server/map_saver_cli` as a YAML metadata file plus PGM occupancy
image under `/home/yufei/Desktop/go2_base_navi/maps/`.

### 4. Navigation mode

`navigation.launch.py` will accept a map YAML path and start:

- the same planar odometry and scan pipeline used during mapping;
- Nav2 map server and AMCL;
- the required Nav2 planning, control, behavior, lifecycle, velocity smoother,
  and collision monitor nodes;
- the tested GO2 `/cmd_vel` bridge; and
- an RViz navigation view.

AMCL publishes `map -> odom`; the odometry adapter publishes the remaining TF
chain. The operator sets the initial pose in RViz and then sends a Nav2 goal.
The first controller is Nav2 Regulated Pure Pursuit using differential motion
with lateral velocity disabled, even though the GO2 can move sideways. This
reduces the initial tuning surface and matches the existing proven control
path.

The controller speed envelope follows the real-robot requirement:

- maximum forward linear velocity: `0.4 m/s`;
- commanded nonzero forward walking range: `0.30-0.40 m/s`, because the robot
  does not walk below `0.30 m/s`;
- maximum angular velocity: `0.4 rad/s`;
- lateral velocity: `0.0 m/s` for this first version.

The bridge does not raise arbitrary tiny commands to `0.30 m/s`, because doing so near a goal could create an unexpected jump.
Instead, Regulated Pure Pursuit uses a `0.30 m/s` minimum approach speed, a
`0.40 m/s` desired speed, and a `0.25 m` goal tolerance that permits a direct
stop. The velocity smoother may briefly
cross the lower range during acceleration and deceleration; the final zero
command still produces an explicit StopMove request.

The costmaps use a conservative rectangular footprint of exactly
`0.80 m x 0.50 m` (half extents `0.40 m x 0.25 m`) to cover the body and normal
standing leg envelope. The inflation layer uses a `0.25 m` radius and a `5.0`
cost-scaling factor. These dimensions are intentionally more conservative than
the simulated trunk collision box and can be narrowed only after observing the
real local costmap.

### 5. Motion-command safety

The command path is:

```text
Nav2 controller -> cmd_vel_nav -> velocity smoother
-> collision monitor -> /cmd_vel -> GO2 bridge -> Sport API
```

Safety behavior is deterministic:

- the bridge clamps linear x and angular z to `0.4` and clamps linear y to
  zero;
- a `0.5 s` bridge command timeout sends StopMove;
- receiving a zero command sends StopMove once;
- shutting down the bridge sends StopMove;
- the collision monitor consumes `/scan`, uses a `0.5 s` source timeout, and
  applies the stop polygon `[(0.60, 0.35), (0.60, -0.35), (-0.50, -0.35),
  (-0.50, 0.35)]` in `base_footprint`;
- a stale or missing scan causes the collision monitor command output to be
  zero, which the bridge converts to StopMove;
- the first version uses stop-or-pass behavior rather than a slowdown zone,
  because slowdown commands below `0.3 m/s` do not move this robot reliably;
- neither launch file sends StandUp, gait changes, or any advanced action;
  the operator remains responsible for putting the robot in its normal walking
  mode and holding the physical remote for emergency intervention.

## Package Layout

```text
go2_base_navi/
  README.md
  src/go2_base_nav/
    package.xml
    setup.py
    setup.cfg
    resource/go2_base_nav
    go2_base_nav/
      __init__.py
      planar_odom.py
      pose_math.py
      sport_client.py
      cmd_vel_bridge.py
      command_safety.py
    launch/
      sensors.launch.py
      mapping.launch.py
      navigation.launch.py
    config/
      pointcloud_to_laserscan.yaml
      slam_toolbox.yaml
      nav2_params.yaml
    rviz/
      mapping.rviz
      navigation.rviz
    test/
      test_pose_math.py
      test_planar_odom.py
      test_command_safety.py
      test_launch_files.py
  maps/
  docs/
    TESTING.md
    superpowers/specs/2026-07-14-go2-real-mapping-navigation-design.md
```

Pure quaternion/pose math and command limiting are separated from ROS node
wrappers so their safety-critical behavior can be tested without hardware.

## Error Handling and Diagnostics

- Missing Unitree message packages fail at build/startup with a documented
  source-order correction.
- Missing `pointcloud_to_laserscan` fails dependency checks with the exact Jazzy
  package name documented.
- Missing or stale robot odometry prevents fresh TF publication; Nav2 then
  refuses to compute a valid control command.
- Missing or stale point cloud prevents fresh `/scan`; collision monitoring
  commands a stop during navigation.
- Invalid quaternions or non-finite odometry values are skipped and throttled
  warnings identify the input fault.
- TF, scan, SLAM, AMCL, costmaps, plans, and collision state are preconfigured
  in RViz for direct visual diagnosis.
- Operator instructions distinguish ROS daemon cache problems from live DDS
  data and always use the Unitree CycloneDDS environment for runtime checks.

## Verification Strategy

### Automated verification

1. Pose-math tests prove that yaw extraction and residual body orientation
   recompose to the original 3D orientation within numeric tolerance.
2. Odometry tests prove that planar output has zero z/roll/pitch, preserves
   x/y/yaw and timestamps, and uses the required frame names.
3. Command-safety tests prove clamping to `0.4 m/s`, `0.0 m/s` lateral, and
   `0.4 rad/s`, plus zero/timeout/shutdown StopMove behavior.
4. Launch/config tests load every launch file and YAML file and verify the
   required topics, frames, speed limits, and safety timeouts.
5. `colcon build`, `colcon test`, and `colcon test-result --verbose` must finish
   without failures.

### Supervised real-robot verification

1. Sensor smoke test: while stationary, `/scan` is fresh, TF resolves from
   `odom` to `base_link`, and RViz shows walls/furniture without persistent
   floor rings.
2. Motion scan test: rotate and walk while observing that walls remain stable
   enough for SLAM rather than bending or duplicating severely.
3. Mapping test: cover the room, close a loop, save the map, and reopen the
   saved YAML successfully.
4. Localization test: load the saved map, set the initial pose, and verify the
   AMCL scan overlay agrees with walls and furniture.
5. First navigation test: command a short goal in open space with the remote
   emergency stop ready; verify travel in the `0.30-0.40 m/s` range and
   an explicit stop at the goal.
6. Obstacle test: place a chair in the path and verify replanning or collision
   stop without contact.
7. Data-loss test: stop the navigation launch or interrupt the scan source and
   verify the bridge issues StopMove within the configured timeout.

## Acceptance Criteria

The first version is accepted when all of the following are true:

- one mapping launch produces a stable 2D occupancy grid from the live GO2
  LiDAR and odometry;
- the map can be saved and subsequently loaded;
- one navigation launch localizes the GO2 on the saved map and accepts RViz
  goals;
- the GO2 follows a collision-free path around representative furniture;
- commanded speed never exceeds `0.4 m/s` linear or `0.4 rad/s` angular and no
  lateral command is sent;
- a zero command, stale command, scan loss, or node shutdown results in a
  StopMove request; and
- automated tests and the supervised smoke-test checklist pass.

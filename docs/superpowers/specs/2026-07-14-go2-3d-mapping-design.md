# GO2 3D Mapping Design

## Goal

Add an isolated, mapping-only 3D SLAM mode for the real GO2. The first test
must show whether full furniture geometry gives more distinctive and stable
mapping than the existing height-sliced 2D map. It must not start Nav2 or any
node that can publish robot motion commands.

This first increment produces a graph-optimized RTAB-Map database and an
interactive 3D view. Integrating the resulting 3D localization transform with
Nav2 is a separate increment after the map quality is observed on the robot.

## Evidence and constraints

- The live GO2 already publishes motion-deskewed 3D LiDAR data on
  `/utlidar/cloud_deskewed` and LiDAR odometry on `/utlidar/robot_odom`.
- The existing `planar_odom` node publishes `odom -> base_footprint` and the
  residual z/roll/pitch transform `base_footprint -> base_link`. Therefore
  `odom -> base_link` retains the full measured pose needed by 3D mapping even
  though `/odom` itself is planar for Nav2.
- The existing point-cloud-to-laser pipeline proves that the live LiDAR frame
  can be transformed into the GO2 base frame.
- The L2 is mounted upside down and its cloud contains floor and self returns.
  A 3D mapping pipeline must explicitly remove the robot body volume.
- The environment is one finite, flat household floor containing tables,
  chairs, and other clutter. There are no stairs.
- Mapping is supervised and motion is performed only with the physical remote.
- Computation runs on the desktop, not on the GO2 computer, to avoid adding to
  the robot's thermal load.

## Approaches considered

### A. RTAB-Map with external GO2 LiDAR odometry — selected

Use the existing deskewed cloud and 6-DoF TF pose as RTAB-Map input. RTAB-Map
adds 3D ICP registration, graph optimization, proximity loop closures, a
persistent database, and a later localization-only mode. Jazzy binary packages
are available, so this does not require maintaining a SLAM fork.

Trade-off: it requires installing `ros-jazzy-rtabmap-ros` and
`ros-jazzy-pcl-ros`, and it consumes more desktop CPU and memory than the 2D
pipeline.

### B. Local voxel point-cloud accumulator

Transform every cloud by GO2 odometry, voxelize it, and save one PCD file. This
has few dependencies and is useful for a visual sensor check, but it has no
loop closure or 3D relocalization. It would not address the stated localization
problem, so it is rejected as the primary implementation.

### C. Point-LIO ROS 2

Run a separate LiDAR-inertial odometry stack from cloud and IMU data. This can
replace the onboard odometry, but the available L2 ROS 2 port is third-party,
primarily documented for Humble, and requires calibration and source builds on
this Jazzy system. It also does not by itself provide the loop-closure and
long-term localization workflow needed here. It is deferred unless the onboard
LiDAR odometry proves to be the remaining source of drift.

## Architecture

```text
/utlidar/robot_odom
        |
        v
planar_odom (existing, motion-free)
        |
        +--> odom -> base_footprint -> base_link TF (full 6-DoF chain)

/utlidar/cloud_deskewed
        |
        v
pcl_ros CropBox in base_link
  remove x[-0.45,0.45], y[-0.32,0.32], z[-0.45,0.30]
        |
        v
pcl_ros VoxelGrid, 0.08 m leaves
        |
        v
/cloud_3d_filtered
        |
        v
RTAB-Map scan_cloud input + external odom TF
        |
        +--> map_3d -> odom
        +--> maps/room_3d.db
        +--> RTAB-Map 3D visualization and loop-closure graph
```

The self crop is performed after transforming the cloud to `base_link`.
`negative=true` keeps points outside the crop box. The box covers the body and
normal leg envelope but does not discard furniture merely because it is below
the LiDAR. RTAB-Map additionally applies a `0.25 m` minimum and `8.0 m` maximum
scan range.

RTAB-Map reads odometry from TF by setting its nonempty
`odom_frame_id=odom`; it does not consume the planar `/odom` message. This is
important because the TF lookup from `odom` to `base_link` traverses both
transforms above and therefore retains z, roll, and pitch for 3D registration.

The 0.08 m voxel size and 1 Hz graph update are conservative desktop-load
defaults for the first household test. They preserve chair legs and table
edges while avoiding the full 15 Hz point-cloud load in the graph optimizer.

## RTAB-Map configuration

The mapping node uses:

- `frame_id=base_link`;
- `odom_frame_id=odom`;
- `map_frame_id=map_3d`;
- `subscribe_scan_cloud=true`, with RGB, depth, stereo, and 2D scan inputs
  disabled;
- 3D ICP registration with point-to-plane matching;
- `Icp/VoxelSize=0.08`;
- `Icp/MaxCorrespondenceDistance=0.30`;
- proximity-by-space loop closure enabled;
- `Grid/3D=true`, `Grid/FromDepth=false`, and LiDAR range `0.25-8.0 m`;
- a 1 Hz detection rate; and
- database path `maps/room_3d.db` by default.

The first mapping launch starts the existing odometry adapter, the two PCL
filters, RTAB-Map SLAM, and RTAB-Map visualization. It never includes
`sensors.launch.py`, because that would also start the unnecessary 2D laser
projection, and it never includes `go2_cmd_vel_bridge`, Nav2, or a teleoperation
node.

## Data persistence and operation

RTAB-Map writes the graph, poses, and compressed sensor data into the database
specified by the launch argument `database_path`. A `new_map` launch argument
controls whether RTAB-Map starts with a clean database; the default is `true`
for this initial comparison run. Existing database deletion must be explicit in
the launch process and limited to that configured database path.

The operator starts mapping, walks a slow closed loop with the physical remote,
revisits the start area from a similar direction, observes any accepted loop
closure, and stops the launch with Ctrl-C. The database is the primary saved
artifact. PCD/PLY export uses RTAB-Map's installed database tooling after the
first database has been validated.

## Failure handling and safety

- Missing cloud or TF data leaves the mapper waiting and must not trigger any
  robot action.
- Invalid transforms or ICP rejection are logged; the current graph remains
  intact.
- The launch contains no `/cmd_vel` subscriber and no
  `/api/sport/request` publisher.
- Shutdown stops visualization and mapping cleanly so the database can flush.
- If self returns remain visible, adjust only the crop-box bounds before ICP
  tuning.
- If the graph drifts despite clean filtered clouds, investigate onboard
  `/utlidar/robot_odom`; do not hide odometry drift with map-display settings.

## Verification

Automated tests must verify:

1. package metadata declares the RTAB-Map and PCL runtime dependencies;
2. `mapping_3d.launch.py` loads without hardware;
3. the launch contains the odometry adapter, CropBox, VoxelGrid, RTAB-Map SLAM,
   and visualization;
4. self-filter bounds, voxel size, frames, input topic, ranges, database path,
   and ICP parameters match this design;
5. no motion bridge, Nav2 bringup, or motion topic is present; and
6. the operator documentation contains build, launch, closed-loop walking,
   database, and shutdown instructions.

The supervised robot acceptance test passes when:

- `/cloud_3d_filtered` contains furniture and floor but no persistent robot-body
  cluster;
- table tops, chair legs, and wall planes stay at consistent heights;
- revisiting the start produces a graph correction without double walls;
- `maps/room_3d.db` exists and is nonempty after clean shutdown; and
- no software motion command is published during the entire mapping run.

## Non-goals for this increment

- autonomous movement during mapping;
- Nav2 integration or 3D obstacle planning;
- replacing the GO2 onboard LiDAR odometry;
- multi-floor or stair handling;
- mesh reconstruction or colorized mapping; and
- automatic conversion of the 3D map back into a 2D navigation map.

# GO2 3D Mapping RViz Design

## Goal

Make the existing motion-free GO2 3D mapping launch open a preconfigured RViz
view by default. One launch command must show the live filtered scan, the
accumulated RTAB-Map cloud, the projected 2D occupancy map, the mapped path, and
TF while preserving the current database and physical-remote-only workflow.

## Approaches considered

### A. Extend the existing 3D launch — selected

Add a dedicated `mapping_3d.rviz` asset and a conditional RViz node directly
to `mapping_3d.launch.py`. Default RViz on and RTAB-Map's specialist
visualizer off. This keeps one canonical command and matches the existing 2D
mapping launch pattern.

### B. Add an RViz wrapper launch

Keep `mapping_3d.launch.py` unchanged and add a second launch that includes it
plus RViz. This preserves old defaults but creates two competing operator
entrypoints and duplicated arguments.

### C. Add a shell helper

Start the existing launch and RViz from a script. This is short, but it hides
ROS launch arguments, has weaker shutdown behavior, and is harder to test and
install correctly.

## Runtime design

`mapping_3d.launch.py` gains `use_rviz`, defaulting to `true`.
`use_rtabmap_viz` remains available but defaults to `false`. The RViz node
loads the installed `rviz/mapping_3d.rviz` file and shares `use_sim_time`
with the mapping stack.

The default launch remains:

```bash
ros2 launch go2_base_nav mapping_3d.launch.py \
  database_path:=/home/yufei/Desktop/go2_base_navi/maps/room_3d.db \
  new_map:=true
```

The operator can enable the specialist RTAB-Map view explicitly with
`use_rtabmap_viz:=true`, or disable all GUI processes with
`use_rviz:=false`.

## RViz view

The fixed frame is `map_3d`. The view contains:

- TF and an XY reference grid;
- `/cloud_3d_filtered` as the current body-cropped, voxelized scan;
- `/cloud_map` as the accumulated graph-optimized 3D cloud;
- `/map` as the projected 2D occupancy grid; and
- `/mapPath` as the accumulated robot trajectory.

The live scan uses orange 4-pixel points with best-effort reliability. The
accumulated cloud uses cyan 2-pixel points with reliable delivery. This makes
current sensor coverage distinct from saved map structure. The occupancy map
uses transient-local durability, and the initial camera is an oblique orbit
view suitable for a single flat floor.

## Safety and failure behavior

RViz is visualization-only. The launch must still exclude Nav2,
`go2_cmd_vel_bridge`, teleoperation nodes, `/cmd_vel`, and
`/api/sport/request`. Closing RViz must not stop RTAB-Map or issue motion.
Missing map topics leave the associated display empty while mapping continues.
Ctrl-C remains the required clean shutdown so RTAB-Map flushes its database.

## Verification

Automated tests will verify:

1. the RViz asset is installed and uses `map_3d`;
2. the grid, TF, two point clouds, occupancy map, and path displays are present
   with the four required topic names;
3. the 3D launch declares `use_rviz`, starts one conditional RViz node, and
   keeps one optional RTAB-Map visualizer;
4. RViz defaults on and RTAB-Map visualization defaults off;
5. existing fresh/resume database behavior and motion-free safety checks still
   pass; and
6. the operator documentation shows the one-command workflow and explains the
   live scan versus accumulated map.

An offline launch smoke test will run with both visualizers disabled to verify
the ROS graph without requiring a display or commanding the robot. The live
acceptance test passes when RViz incrementally shows `/cloud_map`, `/map`,
and `/mapPath` while the GO2 is moved only with its physical remote.

## Non-goals

- 3D localization-only mode;
- Nav2 integration or autonomous mapping;
- changing RTAB-Map ICP or body-crop parameters; and
- replacing `rtabmap_viz` for graph-level diagnostics.

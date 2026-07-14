# GO2 RViz Navigation Goal Fix Design

## Context

The running Nav2 stack, AMCL, and `map -> base_footprint` transform were healthy,
but clicking the goal tool produced no `/navigate_to_pose` action status, global
plan, local plan, or velocity command. The project RViz configuration contains
`nav2_rviz_plugins/GoalTool` but omits the `nav2_rviz_plugins/Navigation 2`
panel. The installed Nav2 reference configuration contains both. The panel owns
the `NavigateToPose` action client, so the tool alone cannot start navigation.

## Chosen Approach

Add the Navigation 2 panel to the existing project RViz configuration while
retaining the custom GO2 map, scan, costmap, path, and footprint displays. Add a
regression test that requires both the panel and goal tool to be present.

This is preferred over replacing the whole file with Nav2's default RViz
configuration, which would discard the GO2-specific displays, and over sending
goals from the command line, which is only a temporary workaround and requires
manually entering a physical target pose.

## Data Flow

After the change, the goal path is:

```text
RViz GoalTool
  -> Navigation 2 panel
  -> /navigate_to_pose action
  -> planner_server
  -> controller_server
  -> velocity_smoother
  -> collision_monitor
  -> /cmd_vel
  -> go2_cmd_vel_bridge
  -> /api/sport/request
```

## Scope and Safety

- Modify only the navigation RViz configuration and its regression test.
- Do not change controller gains, velocity limits, collision monitoring,
  mapping, localization, or the Unitree command bridge.
- Do not send an automated navigation goal during offline verification.
- A live test still requires the physical controller in hand, a correctly set
  initial pose, aligned scan/map data, and one short goal in open space.
- All old mapping, sensors, and navigation launch instances must be stopped
  before restarting a single navigation launch, because duplicate sensor nodes
  were observed during diagnosis.

## Verification

1. A focused test fails before the RViz panel is added.
2. The focused test passes after the panel is added and still requires the goal
   tool.
3. The full package test suite and build pass.
4. After a clean navigation restart, clicking a goal produces a
   `/navigate_to_pose` action and a global plan before any live motion is
   evaluated.
5. During the live test, trace the three velocity stages and the Unitree Sport
   request; stop immediately if localization, scan alignment, or direction is
   wrong.

## Success Criteria

The RViz goal click reaches the Nav2 `NavigateToPose` action without changing
any established GO2 motion or collision-safety limits. A separate live test
then determines whether any downstream motion issue remains.

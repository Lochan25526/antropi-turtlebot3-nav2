# Test results

Each run was driven by `scripts/nav_test.py` against a live `demo.launch.py` and records
Gazebo ground truth at 10 Hz. `<name>.json` has the per-goal result, time, recoveries and
re-plan count plus the closest approach to each obstacle; `<name>.png` overlays the robot
and obstacle tracks on the map; `<name>_timeseries.png` shows robot-to-obstacle distance
and robot speed against time.

| File | Launch arguments | Notes |
|---|---|---|
| `headless_no_obstacles` | `gui:=false rviz:=false obstacle:=false static_obstacle:=false` | baseline |
| `headless_patrol_box` | `gui:=false rviz:=false` | the default configuration |
| `headless_patrol_box_restart` | `gui:=false rviz:=false` | same, after a full restart; recorded before the obstacle's yield rule existed |
| `headless_crossing_mode` | `gui:=false rviz:=false obstacle_mode:=crossing` | experimental mode |
| `gui_recorded_run` | defaults | the run in the submitted screen recording |
| `gui_run_from_manual_pose` | defaults | GUI run that started from a pose set by hand in RViz instead of the spawn pose |
| `tb3_world_headless`, `tb3_house_headless`, `aws_small_warehouse_headless` | `scene:=<name> gui:=false rviz:=false` | the three stock-world scenes |
| `tb3_world_gui_recorded`, `tb3_house_gui_recorded`, `aws_small_warehouse_gui_recorded` | `scene:=<name>` | the runs in `docs/media/<name>_demo.mp4` |
| `earlier_layouts/` | | two obstacle layouts that were rejected (see `SUBMISSION_ANSWERS.md`, challenges) |

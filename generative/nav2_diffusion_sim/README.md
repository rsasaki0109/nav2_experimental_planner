# nav2_diffusion_sim

Closed-loop Gazebo **course assets** for `nav2_experimental_planner` (see
[../docs/simulation.md](../docs/simulation.md)).

Each obstacle course is defined **once** as a spec in
[`nav2_diffusion_sim/gen_courses.py`](nav2_diffusion_sim/gen_courses.py) (map
extent + start pose + goals + wall boxes). From that single spec the generator
emits three mutually-consistent artifacts, so the world, the map, and the goals
cannot drift apart:

- a self-contained gz-sim world `worlds/<course>.sdf.xacro` (physics / sensors /
  sun / ground + the course walls, mirroring the stock `tb3_sandbox` plugin set
  so the simulated LiDAR renders);
- a matching occupancy map `maps/<course>.pgm` + `maps/<course>.yaml` (so AMCL /
  the global costmap see the same walls as the simulator);
- the mission goals (`"label|x|y|yaw|timeout"`) consumed by `sim_mission.py`.

The courses mirror the off-line `planner_benchmark` obstacle scenarios so the
**full closed-loop stack** (global planner + controller + costmap) can be
measured on the same shapes the proposal-stage benchmark uses:

| Course | Shape | Straight line |
|---|---|---|
| `centred` | gap on the line (dead ahead) | clear (trivial by design) |
| `gap` | gap ~2 m off the line | blocked → detour |
| `slalom` | two staggered gaps (low then high) | blocked → S-detour |

## Run (real ROS host)

```bash
ros2 launch nav2_diffusion_sim tb3_gazebo_course.launch.py course:=gap \
    results_file:=/tmp/course_gap_result.md
```

This loads the course world + map, spawns TB3 at the course start, brings up Nav2
with the `DiffusionController`, drives the mission goals, and writes a Markdown
leaderboard.

> **Honest scope.** The course assets are generated and **geometry-checked in-tree**
> (`test/test_gen_courses.py`: start/goals clear, obstacle courses block the
> straight line, map↔walls↔goals consistent, committed artifacts match the spec).
> The closed-loop run itself needs a real ROS host — the dev sandbox blocks
> inter-process DDS, so no fabricated sim numbers are committed (see
> [../docs/simulation.md](../docs/simulation.md) section 10.5).

## Regenerate the assets

After editing a spec in `gen_courses.py`, re-render the committed worlds/maps:

```bash
python3 nav2_diffusion_sim/gen_courses.py
```

`test_committed_artifacts_match_the_specs` fails CI if a spec was changed without
re-running this, keeping the checked-in assets in sync with the specs.

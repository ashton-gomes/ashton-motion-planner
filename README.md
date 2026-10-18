Motion Planner using Unit Dual Quaternions, Screw Linear Interpolation (ScLERP), Resolved motion rate control (RMRC), and RRT. 

## Quick start

From the project directory, build and check the program:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

Run an interactive sample with `./build/testing`, then choose an environment
from the menu. Close the viewer to exit.

For a quick benchmark, run 10 trials per condition:

```bash
./build/benchmark --trials 10 --output "benchmark-smoke-$(date +%Y%m%d-%H%M%S)"
```

For the full experiment, run 1,000 trials per condition (5,000 total):

```bash
./build/benchmark --trials 1000 --output "benchmark-results-$(date +%Y%m%d-%H%M%S)"
```

Each benchmark writes `trials.csv`, `summary.csv`, and `settings.csv` in its
output directory. Use a new output directory for each run. The build needs
Eigen3, MuJoCo, and GLFW for the viewer; use `-DBUILD_VIEWER=OFF` for a headless
build. If MuJoCo is installed elsewhere, pass `-DMUJOCO_ROOT=/path/to/mujoco`.
See [BENCHMARK.md](BENCHMARK.md) for all options and
[the completed 5,000-trial report](benchmark-results-2026-10-16/REPORT.md)
for the saved results.

## Planner behavior

Build with `cmake -S . -B build` and `cmake --build build`, then run
`./build/testing`. The first prompt selects the obstacle environment:
press Enter for `environments/scene.xml`, choose a numbered XML, or type an
XML path (relative to your working directory or absolute).

Set `RRTStarSettings::use_rrt_star` in `include/rrt_star.hpp` to `false` and
rebuild to use regular RRT: new nodes keep their nearest-node parent and planning
stops at the first goal. Setting it to `true` enables RRT*, which selects cheaper
parents, rewires neighbors, and continues improving until `max_expansions`.
The optional optimization is contained in `addRRTNode` in `source/rrt_star.cpp`;
its edge validator reuses the planner's collision checks.

With `use_goal_bias = false` (default), each expansion first attempts a step
toward the goal. If the collision or RMRC checks reject it, the planner tries
up to `samples_per_expansion` random targets and accepts the first valid edge.
Sampling is lazy, so accepting the goal step skips random sampling. Set
`use_goal_bias = true` to restore the original sampling strategy, where each
candidate has `goal_bias` probability of targeting the goal. These settings
are in `include/rrt_star.hpp` and apply to both RRT and RRT*.

Goal steps prefer the newest accepted node, allowing a detour to continue toward
the goal. `selectGoalExpansionNode` remembers tried goal edges and selects the
nearest untried node when needed. Each node's goal edge is tried once for the
current fixed goal; remembering successful attempts also avoids duplicate edges.

Add complete MuJoCo scene XMLs to `environments/` to have them appear in the
menu automatically; changing or adding XMLs requires no rebuild. Each scene
must include the same Kinova robot model for collision checking and playback.
Include paths are relative to the scene XML, so a scene in `environments/`
can include `../mujoco_menagerie/kinova_gen3/gen3.xml`. `empty.xml` is provided
as a minimal example without obstacles.

The existing random placement behavior still applies to geoms named
`obs_cylinder_1`, `obs_cylinder_2`, and `obs_cylinder_3`; other obstacle names
use the positions in the selected XML.

Goal poses are sampled by choosing joint angles within the robot model's limits
and computing forward kinematics, so position and orientation share a joint
solution. The `sampleReachablePose` helper in `sampler.hpp` reuses the workspace
and collision checks and stops after `obstacle_sample_attempts` tries. Continuous
hinge joints use one full turn, from -pi to pi. A valid goal configuration does
not guarantee that the planner will find a path from home.

When planning fails, the console prints the failure reason, counts of rejected
candidates by check, and the closest tree node's position and rotation errors.
The viewer then loops the current tree, traversing each accepted edge out and
back through its parent (including rewired edges). If only the home node exists,
it displays that configuration. Close the viewer to exit; a failed plan still
returns exit code 1. The helper lives in `planner_debug.hpp`/`planner_debug.cpp`.
Rejection counts describe observed checks; they do not prove the goal is unreachable.



# Modular robot arms

The planner used to work only with the Kinova Gen3. It now works with any arm
made of hinge joints. Each robot is described by one `RobotConfig`, and you
switch robots by changing one line.

Robots included:

| Function | Model file | Planned joints | Tool body |
|---|---|---|---|
| `robots::kinovaGen3()` | `mujoco_menagerie/kinova_gen3/gen3.xml` | all 7 joints in the model | `bracelet_link` |
| `robots::g1RightArm()` | `unitree_ros/robots/g1_description/g1_29dof.xml` | 7 right-arm joints | `right_wrist_yaw_link` |
| `robots::g1LeftArm()` | `unitree_ros/robots/g1_description/g1_29dof.xml` | 7 left-arm joints | `left_wrist_yaw_link` |

Every environment has a G1 version (`environments/g1_*.xml`, laid out for the
right arm). `testing` and `benchmark` pick the selected robot's versions
automatically through `environment_prefix` (see below).

---

## How to switch robots

Change `robots::active()` in [testbed/src/robots.cpp](../testbed/src/robots.cpp):

```cpp
// Change this line to switch the robot used by `testing` and `benchmark`.
// The environment files must include the same robot model.
TestRobot active() { return kinovaGen3(); }   // or g1RightArm(), g1LeftArm()
```

Then rebuild (`make` in `build/`). `testing` prints the selected robot when it
starts, e.g. `Robot: kinova_gen3 (7 joints).`

## How to add a new robot

Add a function to [testbed/src/robots.cpp](../testbed/src/robots.cpp) and declare it in
[testbed/include/testbed/robots.hpp](../testbed/include/testbed/robots.hpp):

```cpp
TestRobot myArm() {
    TestRobot test_robot;
    RobotConfig& robot = test_robot.config;   // what the planner needs
    robot.name = "my_arm";
    robot.model_path = kRoot + "/path/to/my_arm.xml";   // MJCF, or a URDF MuJoCo can load
    robot.tool_body = "end_effector_link";
    robot.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    robot.home_joints = {0.0, 0.5, 0.0, 1.0, 0.0, 0.0};  // or leave empty and add a "home" keyframe
    test_robot.environment_prefix = "my_arm_";  // optional: environments/my_arm_*.xml
    return test_robot;
}
```

`RobotConfig` is the planner's part and is all `MotionPlanner` takes.
`TestRobot` adds settings only the testbed uses.

The `RobotConfig` fields ([planner/include/gplanner/robot_config.hpp](../planner/include/gplanner/robot_config.hpp)):

| Field | What it is | If left empty |
|---|---|---|
| `name` | Label for messages and benchmark output | — |
| `model_path` | MuJoCo model of the robot on its own | required |
| `tool_body` | Body whose pose is planned (end effector) | required |
| `joint_names` | Hinge joints to plan with, base to tool | every joint in the model is used |
| `home_joints` | Start angles, one per planned joint (radians) | read from the `home_keyframe` |
| `home_keyframe` | Keyframe name, default `"home"` | joints not planned use the model's default pose |
| `held_joints` | Positions for joints that are not planned, by name (e.g. let a humanoid's other arm hang at its side) | keyframe or default pose |
| `visual_collision_meshes` | Meshes the model only draws but that should block motion too; the planner adds a collision copy of each when loading (the G1's hands) | only the model's own collision shapes |

`TestRobot` adds one testbed-only field:

| Field | What it is | If left empty |
|---|---|---|
| `environment_prefix` | Prefix of this robot's environment files, e.g. `"g1_"` → `g1_wall_hole.xml` | the plain (Kinova) files |

The planner reads everything else from the model file: joint axes, joint
positions, joint limits, and arm length (for the sampling workspace). Joint
count is no longer fixed at 7.

If the config doesn't match the model, the planner stops with a clear error,
for example:

- `Robot model has no joint named 'right_elbow'.`
- `Joint 'x' is not a hinge; only revolute joints can be planned.`
- `Joint 'b' does not follow 'a' in one chain from base to tool.`
- `Tool body 'hand' does not come after the last planned joint.`
- `home_joints has 6 values but 7 joints are planned.`

---

## How it works

### 1. Planning with part of a bigger robot

The G1 has 30 joints (a floating base, legs, waist, two arms). The planner
plans only the joints in `joint_names` and holds every other joint still:

- **Held values** come from the `home_keyframe` if the model has one, otherwise
  from the model's default pose (`qpos0`). The G1 model has no keyframes, so
  the G1 stands at its default pose with the pelvis at 0.793 m.
- **Joints are matched by name**, not by position in the joint list. The same
  config works on the robot's own model and on any scene that `<include>`s it.
  Internally a `ChainBinding` records where each planned joint sits in a loaded
  model's `qpos`, and writes the held values plus the planned values before
  every MuJoCo call.

### 2. Kinematics with any number of joints

The fixed `const int n = 7` is gone. Joint vectors are `Eigen::VectorXd`, and
the Jacobian is `Eigen::Matrix<double, 6, Eigen::Dynamic>`, sized from the
robot. The math itself (dual-quaternion product of exponentials, ScLERP, RMRC,
RRT) is unchanged.

`loadRobot()` (previously `extractURDF()`):

1. Finds the planned joints and checks they are hinges in one chain ending at
   `tool_body`.
2. Sets every joint to its held value and the planned joints to zero, then
   reads each planned joint's axis and anchor point. This gives the screw axes
   and the zero-angle tool pose `M`.
3. Reads joint limits and estimates the arm's reach.
4. Takes the home pose from `home_joints`, or from the keyframe.

### 3. Collision checking without the Kinova's "group 3" rule

Before, the code found the robot's own collision shapes by MuJoCo geom group 3,
which is how the Kinova model happens to be set up. The G1 puts its collision
shapes in group 0, so its body would have been treated as an obstacle.

Now:

- **Robot geoms** are any geoms attached to a body that exists in the robot's
  model (matched by body name). This applies to the obstacle checker and the
  obstacle sampler (`loadSceneObstacles`).
- **Self-collision** ignores contacts between links that can't move relative to
  each other, meaning there is no planned joint between them. For the G1 this
  means legs, pelvis, torso and the held arm never count as colliding with each
  other, but the moving arm hitting the torso or the other arm does count.
  Contacts with world geoms (like the G1 model's floor) are ignored as well.
  Directly connected links are still ignored, as before.

For the Kinova, these rules give exactly the same result as the old ones.

### 4. Viewer playback

Plans store one value per planned joint (7 for a G1 arm), but the viewer needs
the full scene pose (36 values for the G1). `MotionPlanner::toSceneQpos()` and
`toSceneTrajectory()` fill in the held joints. `main.cpp` and the failure
replay (`debugPlannerFailure`) use them. For the Kinova, the two are the same.

---

## What changed, file by file

This records the change that made robots modular. File paths are updated to
the current `planner/` and `testbed/` layout.

**New files**

- [planner/include/gplanner/robot_config.hpp](../planner/include/gplanner/robot_config.hpp): the `RobotConfig` struct.
- [testbed/include/testbed/robots.hpp](../testbed/include/testbed/robots.hpp), [testbed/src/robots.cpp](../testbed/src/robots.cpp):
  the Kinova and G1 presets, and `robots::active()`.

**Changed files**

- [planner/src/planner.cpp](../planner/src/planner.cpp)
  - Removed `const int n = 7` and the hard-coded `ROBOT_XML_PATH`.
  - All `Eigen::Matrix<double, n, …>` types became dynamic (`JointVector`, `SpatialJacobian`).
  - `extractURDF(path)` became `loadRobot(config)` (see "How it works" above).
  - Added `ChainBinding` / `bindChain()` (planned joints and held values per model).
  - `SelfCollisionChecker` uses the configured tool body instead of
    `"bracelet_link"`, and the new "links that can't move apart" rule.
  - `ObstacleCollisionChecker` detects robot geoms by body name, not group 3.
  - Scene `start`/`goal` keyframes are read through the binding, so a scene can
    have more joints than the planned arm.
  - New methods: `robot()`, `dof()`, `toSceneQpos()`, `toSceneTrajectory()`.
- [planner/include/gplanner/planner.hpp](../planner/include/gplanner/planner.hpp): the constructor takes a
  `RobotConfig` instead of a model path, plus the four new methods.
- [planner/include/gplanner/sampler.hpp](../planner/include/gplanner/sampler.hpp), [planner/src/sampler.cpp](../planner/src/sampler.cpp):
  `loadSceneObstacles()` takes the robot's body names and skips their geoms
  (it used to skip group 3).
- [testbed/include/testbed/planner_debug.hpp](../testbed/include/testbed/planner_debug.hpp), [testbed/src/planner_debug.cpp](../testbed/src/planner_debug.cpp):
  `debugPlannerFailure()` takes a function that converts joints to a scene pose
  before replaying the tree.
- [testbed/apps/testing.cpp](../testbed/apps/testing.cpp): uses `robots::active()`, prints the
  robot, converts trajectories for the viewer.
- [testbed/apps/benchmark.cpp](../testbed/apps/benchmark.cpp): uses `robots::active()`;
  `settings.csv` now also records the robot `name`.
- [CMakeLists.txt](../CMakeLists.txt): `KINOVA_MODEL_PATH` / `ROBOT_XML_PATH` removed;
  `robots.cpp` added to the build, with `PROJECT_ROOT_PATH` so model
  paths are relative to the repository.
- [tests/planner_tests.cpp](../tests/planner_tests.cpp): existing tests use
  `robots::kinovaGen3()`. New `checkG1Arms()` test: for each G1 arm, home is
  collision-free and matches MuJoCo, and a random goal is planned and validated.
  The G1's own model is used as an obstacle-free scene.

---

## Verification

- `ctest`: both test programs pass, including the new G1 arm test.
- Kinova before and after: ran `benchmark --trials 20` (140 trials over all 7
  conditions) on the old and new code. Success and valid-path counts are
  identical for every condition. 139 of 140 trials match to within 2e-15 (float
  rounding). One `random_cylinders` trial grew a different tree, because the
  planner's search is very sensitive to tiny rounding differences from the
  dynamic-size matrices. The collision rules were checked to be identical
  for the Kinova.
- Not checked: the interactive `testing` viewer window (needs a display).

## G1 environments

Each Kinova scene has a G1 version with the same idea, scaled to the G1's
right arm (shoulder at about x = 0, y = -0.10, z = 1.085 m; the wrist reaches
about 0.37 m from it; the hand reaches another 13-17 cm past the wrist). The
left arm hangs at the side (`held_joints`), because in the G1's default pose
both forearms point straight forward.

**The G1's hands have no collision shape in the Unitree model** (only a visible
mesh). Without `visual_collision_meshes`, the planner let the hands pass through
obstacles by up to 9 cm. The G1 presets list both hand meshes there, so the
planner checks them like any other link.

| Scene | Task |
|---|---|
| `g1_empty.xml` | No obstacles, random goal |
| `g1_random_cylinders.xml` | Three posts in front of the right shoulder, random goal. Posts named `g1_obs_cylinder_*` move randomly on each interactive run (G1 layout in `testbed/src/scene_randomizer.cpp`) |
| `g1_wall_hole.xml` | Reach through a 20 cm window in a wall 0.24 m in front |
| `g1_long_cylinders.xml` | Reach out between two of three tall posts on an arc around the right shoulder |
| `g1_under_table.xml` | From above a chest-high table to under its near edge (chest-high so the hand fits above and below it within reach) |
| `g1_shelf.xml` | From in front of the chest, hand pointing down, into a 20 cm cubby of a shelf in front |

The G1 files include `unitree_ros/robots/g1_description/g1_29dof.xml`, which
brings its own floor and lights. Keyframes hold the full G1 pose (36 values:
floating base 7, legs and waist 15, left arm 7, right arm 7). Only the planned
arm's 7 values are used.

In each fixed-task scene, the start and goal are collision-free and the
straight-line motion is blocked by an obstacle (`planner_tests` checks this
for every scene). Each planned path was also replayed with every *visible*
robot mesh made solid: no frame in any G1 scene showed the robot overlapping
an obstacle.

Benchmark (`benchmark --trials 20`, which allows 1000 expansions per plan; the
`testing` app allows 2000):

| Scene | G1 goal reached | G1 path validated | Kinova goal reached |
|---|---|---|---|
| empty (sampled) | 90% | 60% | 100% |
| random cylinders | 90% | 60% | 95% |
| wall hole | 100% | 45% | 95% |
| long cylinders | 40% (80% with 2000 expansions) | 5% | 70% |
| under table | 20% (65% with 2000 expansions) | 0% | 80% |
| shelf | 100% | 70% | 100% |

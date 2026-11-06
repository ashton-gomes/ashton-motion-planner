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

The environment files have not changed yet. They still include the Kinova
model, so the `testing` viewer and `benchmark` should keep using the Kinova
until G1 versions of the environments exist.

---

## How to switch robots

Change `robots::active()` in [source/robots.cpp](../source/robots.cpp):

```cpp
// Change this line to switch the robot used by `testing` and `benchmark`.
// The environment files must include the same robot model.
RobotConfig active() { return kinovaGen3(); }   // or g1RightArm(), g1LeftArm()
```

Then rebuild (`make` in `build/`). `testing` prints the selected robot when it
starts, e.g. `Robot: kinova_gen3 (7 joints).`

## How to add a new robot

Add a function to [source/robots.cpp](../source/robots.cpp) and declare it in
[include/robots.hpp](../include/robots.hpp):

```cpp
RobotConfig myArm() {
    RobotConfig robot;
    robot.name = "my_arm";
    robot.model_path = kRoot + "/path/to/my_arm.xml";   // MJCF, or a URDF MuJoCo can load
    robot.tool_body = "end_effector_link";
    robot.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    robot.home_joints = {0.0, 0.5, 0.0, 1.0, 0.0, 0.0};  // or leave empty and add a "home" keyframe
    return robot;
}
```

The `RobotConfig` fields ([include/robot_config.hpp](../include/robot_config.hpp)):

| Field | What it is | If left empty |
|---|---|---|
| `name` | Label for messages and benchmark output | — |
| `model_path` | MuJoCo model of the robot on its own | required |
| `tool_body` | Body whose pose is planned (end effector) | required |
| `joint_names` | Hinge joints to plan with, base to tool | every joint in the model is used |
| `home_joints` | Start angles, one per planned joint (radians) | read from the `home_keyframe` |
| `home_keyframe` | Keyframe name, default `"home"` | joints not planned use the model's default pose |

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

**New files**

- [include/robot_config.hpp](../include/robot_config.hpp): the `RobotConfig` struct.
- [include/robots.hpp](../include/robots.hpp), [source/robots.cpp](../source/robots.cpp):
  the Kinova and G1 presets, and `robots::active()`.

**Changed files**

- [source/planner.cpp](../source/planner.cpp)
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
- [include/planner.hpp](../include/planner.hpp): the constructor takes a
  `RobotConfig` instead of a model path, plus the four new methods.
- [include/sampler.hpp](../include/sampler.hpp), [source/sampler.cpp](../source/sampler.cpp):
  `loadSceneObstacles()` takes the robot's body names and skips their geoms
  (it used to skip group 3).
- [include/planner_debug.hpp](../include/planner_debug.hpp), [source/planner_debug.cpp](../source/planner_debug.cpp):
  `debugPlannerFailure()` takes a function that converts joints to a scene pose
  before replaying the tree.
- [source/main.cpp](../source/main.cpp): uses `robots::active()`, prints the
  robot, converts trajectories for the viewer.
- [source/benchmark.cpp](../source/benchmark.cpp): uses `robots::active()`;
  `settings.csv` now also records the robot `name`.
- [CMakeLists.txt](../CMakeLists.txt): `KINOVA_MODEL_PATH` / `ROBOT_XML_PATH` removed;
  `source/robots.cpp` added to `planner_core`, with `PROJECT_ROOT_PATH` so model
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

## Still to do (environments)

To run the G1 in the viewer and benchmark, each environment needs a G1 version:

- `<include>` the G1 model and set `meshdir` to `unitree_ros/robots/g1_description/meshes`.
- Move obstacles to the G1's reach (shoulder about 1.1 m high, shoulder to wrist about 0.4 m).
- Rewrite the `start`/`goal` keyframes for the G1 scene's full joint list
  (36 values: 7 floating base + 29 joints). Only the planned arm's values are used.

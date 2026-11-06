#pragma once

#include <string>
#include <vector>

// Everything the planner needs to know about one robot arm. The planner reads
// the joint axes, limits, and link lengths from the model file itself, so a new
// arm only needs a new RobotConfig (see include/robots.hpp).
struct RobotConfig {
    // Short label used in messages and benchmark output.
    std::string name;

    // MuJoCo model of the robot on its own (MJCF .xml, or a .urdf that
    // MuJoCo can load). It is also used for self-collision checks.
    std::string model_path;

    // Body whose frame is the end effector, i.e. the pose being planned.
    std::string tool_body;

    // Hinge joints to plan with, ordered from base to tool. They must form one
    // chain. Leave empty to use every joint in the model, in model order
    // (the model must then be a single arm, like the Kinova Gen3).
    // Every other joint is held still, so a humanoid can plan with one arm.
    std::vector<std::string> joint_names;

    // Start configuration, one value per planned joint (radians). If empty,
    // the planned joints are read from the keyframe named home_keyframe.
    std::vector<double> home_joints;

    // Optional keyframe in the model. If present, it also sets the values of
    // the joints that are held still; otherwise the model's default pose is used.
    std::string home_keyframe = "home";
};

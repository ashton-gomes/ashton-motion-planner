#pragma once

#include <map>
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

    // Optional URDF of the same robot. If set, the planned joints' position
    // limits are read from its <limit lower upper> tags (continuous joints are
    // unlimited) instead of from model_path. Every planned joint must be in it.
    std::string urdf_path;

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

    // Positions (radians) for joints that are held still, by joint name. These
    // override the keyframe or default pose, e.g. to tuck in a humanoid's
    // other arm so it stays out of the way.
    std::map<std::string, double> held_joints;

    // Meshes that the model only shows (no collision shape) but that should
    // block motion too. The planner adds a collision copy of each when it
    // loads the model. The G1's hands need this: without it, the planner
    // lets them pass through obstacles.
    std::vector<std::string> visual_collision_meshes;

    // Environment files made for this robot are named <prefix><scene>.xml,
    // e.g. "g1_" + "wall_hole.xml". The benchmark and the default scene in
    // `testing` use this to pick this robot's version of each scene.
    std::string environment_prefix;
};

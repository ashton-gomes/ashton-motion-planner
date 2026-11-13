#include "robots.hpp"

#include <string>

#ifndef PROJECT_ROOT_PATH
#error "PROJECT_ROOT_PATH must be set by CMake."
#endif

namespace robots {
namespace {

const std::string kRoot = PROJECT_ROOT_PATH;
const std::string kG1Model =
    kRoot + "/unitree_ros/robots/g1_description/g1_29dof.xml";
// Joint limits for the planned arm joints come from the URDF.
const std::string kG1Urdf =
    kRoot + "/unitree_ros/robots/g1_description/g1_29dof.urdf";

// The 7 joints of one G1 arm, shoulder to wrist. side is "left" or "right".
std::vector<std::string> g1ArmJoints(const std::string& side) {
    std::vector<std::string> joints;
    for (const char* joint : {"shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
                              "wrist_roll", "wrist_pitch", "wrist_yaw"})
        joints.push_back(side + "_" + joint + "_joint");
    return joints;
}

}  // namespace

RobotConfig kinovaGen3() {
    RobotConfig robot;
    robot.name = "kinova_gen3";
    robot.model_path = kRoot + "/mujoco_menagerie/kinova_gen3/gen3.xml";
    robot.tool_body = "bracelet_link";
    // joint_names left empty: the model contains only the arm's 7 joints.
    // home_joints left empty: the model's "home" keyframe is used.
    return robot;
}

RobotConfig g1RightArm() {
    RobotConfig robot;
    robot.name = "g1_right_arm";
    robot.model_path = kG1Model;
    robot.urdf_path = kG1Urdf;
    robot.tool_body = "right_wrist_yaw_link";
    robot.joint_names = g1ArmJoints("right");
    // Arm slightly out from the body with the elbow bent, away from the
    // straight-arm singularity. The G1 model has no keyframes.
    robot.home_joints = {0.0, -0.3, 0.0, 1.0, 0.0, 0.0, 0.0};
    // In the default pose the forearms point straight forward. Let the other
    // arm hang at the side (elbow bent 1.5 rad) so it stays out of the obstacles.
    robot.held_joints = {{"left_shoulder_roll_joint", 0.3}, {"left_elbow_joint", 1.5}};
    robot.environment_prefix = "g1_";  // environments/g1_*.xml
    // The model draws the hands but gives them no collision shape.
    robot.visual_collision_meshes = {"right_rubber_hand", "left_rubber_hand"};
    return robot;
}

RobotConfig g1LeftArm() {
    RobotConfig robot;
    robot.name = "g1_left_arm";
    robot.model_path = kG1Model;
    robot.urdf_path = kG1Urdf;
    robot.tool_body = "left_wrist_yaw_link";
    robot.joint_names = g1ArmJoints("left");
    robot.home_joints = {0.0, 0.3, 0.0, 1.0, 0.0, 0.0, 0.0};
    robot.held_joints = {{"right_shoulder_roll_joint", -0.3}, {"right_elbow_joint", 1.5}};
    // The g1_ environments are laid out for the right arm, so none is set here.
    robot.visual_collision_meshes = {"right_rubber_hand", "left_rubber_hand"};
    return robot;
}

// Change this line to switch the robot used by `testing` and `benchmark`.
// The environment files must include the same robot model.
RobotConfig active() { return kinovaGen3(); }

}  // namespace robots

#pragma once

#include "robot_config.hpp"

// Ready-made robot arms. To add a robot, write another function like these in
// source/robots.cpp. To switch robots, change robots::active() there.
namespace robots {

// Kinova Gen3 7-DoF arm from mujoco_menagerie.
RobotConfig kinovaGen3();

// One 7-DoF arm of the Unitree G1 humanoid (unitree_ros). The legs, waist,
// and other arm are held still at the model's default pose.
RobotConfig g1RightArm();
RobotConfig g1LeftArm();

// The robot used by the `testing` viewer and the `benchmark` program.
RobotConfig active();

}  // namespace robots

#pragma once

#include "gplanner/robot_config.hpp"
#include "testbed/sensor.hpp"

#include <string>
#include <vector>

// Ready-made robot arms. To add a robot, write another function like these in
// testbed/src/robots.cpp. To switch robots, change robots::active() there.
namespace robots {

// A robot as the testbed uses it: the planner's configuration plus the
// testbed's own settings.
struct TestRobot {
    RobotConfig config;

    // Environment files made for this robot are named <prefix><scene>.xml,
    // e.g. "g1_" + "wall_hole.xml". The benchmark and the default scene in
    // `testing` use this to pick this robot's version of each scene.
    std::string environment_prefix;

    // Depth cameras for ObstacleMode::SimulatedSensor (see testbed/sensor.hpp).
    std::vector<DepthCameraSettings> cameras;
};

// Kinova Gen3 7-DoF arm from mujoco_menagerie.
TestRobot kinovaGen3();

// One 7-DoF arm of the Unitree G1 humanoid (unitree_ros). The legs, waist,
// and other arm are held still at the model's default pose.
TestRobot g1RightArm();
TestRobot g1LeftArm();

// The robot used by the `testing` viewer and the `benchmark` program.
TestRobot active();

}  // namespace robots

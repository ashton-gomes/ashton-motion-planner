#pragma once

#include "gplanner/planner.hpp"
#include "testbed/robots.hpp"

#include <cstdint>
#include <string>

// How the testbed gives the planner its obstacles.
enum class ObstacleMode {
    KnownGeometry,     // The scene's exact obstacle shapes.
    SimulatedSensor,   // A point cloud from the robot's simulated depth cameras.
};

// "known" or "sensed", as used in benchmark output.
std::string obstacleModeName(ObstacleMode mode);

// Planner options for the mode. edit_scene (e.g. applyCylinderPlacements) is
// applied to the known scene; in sensor mode, pass it to senseObstacles.
ObstacleOptions obstacleOptions(ObstacleMode mode, const SceneEditor& edit_scene = {});

// SimulatedSensor mode: capture the scene with the robot's cameras, with the
// robot posed at the planner's start, and give the cloud to the planner.
// Returns the cloud (e.g. to count or draw its points).
PointCloud senseObstacles(MotionPlanner& planner, const robots::TestRobot& robot,
                          const std::string& scene_xml, const SceneEditor& edit_scene = {},
                          std::uint32_t seed = 1);

#include "gplanner/planner.hpp"
#include "testbed/environment_selector.hpp"
#include "testbed/obstacle_mode.hpp"
#include "testbed/planner_debug.hpp"
#include "testbed/robots.hpp"
#include "testbed/scene_randomizer.hpp"
#include "testbed/viewer.hpp"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

// Set to true to save each successful motion plan as an MP4 in saved_trials/.
constexpr bool kSaveTrialVideo = false;

// Where the planner gets obstacles from:
//   KnownGeometry:   the scene's exact obstacle shapes.
//   SimulatedSensor: a point cloud from the robot's simulated depth cameras
//                    (cameras are set per robot in testbed/src/robots.cpp).
//                    The viewer draws what the planner sensed as blue boxes.
constexpr ObstacleMode kObstacleMode = ObstacleMode::SimulatedSensor;

// Build a path like saved_trials/wall_hole_2026-10-26_19-30-05.mp4.
std::string trialVideoPath(const std::string& scene) {
    std::filesystem::create_directories(SAVED_TRIALS_PATH);
    const std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::ostringstream name;
    name << std::filesystem::path(scene).stem().string() << '_'
         << std::put_time(std::localtime(&now), "%Y-%m-%d_%H-%M-%S") << ".mp4";
    return (std::filesystem::path(SAVED_TRIALS_PATH) / name.str()).string();
}

int main() {
    try {
        // To plan with another arm, change robots::active() in testbed/src/robots.cpp.
        const robots::TestRobot test_robot = robots::active();
        const RobotConfig& robot = test_robot.config;
        // Default to this robot's version of the scene, e.g. g1_wall_hole.xml.
        const std::filesystem::path default_scene(SCENE_XML_PATH);
        const std::string scene = selectEnvironment(
            (default_scene.parent_path() / (test_robot.environment_prefix +
                default_scene.filename().string())).string(), std::cin, std::cout);
        RRTStarSettings settings;
        // Random cylinder positions are applied to every copy of the scene,
        // including the one the simulated sensor looks at.
        MotionPlanner planner(robot, scene, settings,
                              obstacleOptions(kObstacleMode, applyCylinderPlacements));
        std::cout << "Robot: " << planner.robot().name << " (" << planner.dof() << " joints).\n";
        std::vector<SceneObstacle> overlay;
        if (kObstacleMode == ObstacleMode::SimulatedSensor) {
            const PointCloud cloud = senseObstacles(planner, test_robot, scene, applyCylinderPlacements,
                                                    static_cast<std::uint32_t>(std::time(nullptr)));
            overlay = planner.obstacles();
            std::cout << "Sensed " << cloud.points.size() << " points; planning around "
                      << overlay.size() << " voxel boxes.\n";
        } else {
            std::cout << "Using the scene's known obstacle geometry.\n";
        }
        const bool fixed_goal = planner.sceneGoal().has_value();
        const Pose goal = fixed_goal ? *planner.sceneGoal() : planner.sampleGoal();
        std::cout << (fixed_goal ? "Using the scene's fixed goal.\n" : "Using a random goal.\n");
        std::cout << "Planning with " << (settings.use_rrt_star ? "RRT*" : "RRT") << ".\n";
        const auto result = planner.plan(goal, PlannerMode::SampledScLERP);
        if (!result.success) {
            return debugPlannerFailure(result.failure_reason, result.stats, result.tree,
                                       scene, goal, settings.animation_frames_per_edge,
                                       [&](const Eigen::VectorXd& joints) {
                                           return planner.toSceneQpos(joints);
                                       }, overlay);
        }
        std::cout << "Goal reached with " << result.tree.size() << " nodes in "
                  << result.planning_seconds << " s.\n";
        const std::string video_path = kSaveTrialVideo ? trialVideoPath(scene) : "";
        return MujocoViewer::play(scene, planner.toSceneTrajectory(result.trajectory),
                                  planner.homePose(), goal, video_path, overlay);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

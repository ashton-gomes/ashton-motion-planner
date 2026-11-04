#include "environment_selector.hpp"
#include "planner.hpp"
#include "viewer.hpp"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

// Set to true to save each successful motion plan as an MP4 in saved_trials/.
constexpr bool kSaveTrialVideo = true;

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
        const std::string scene = selectEnvironment(SCENE_XML_PATH, std::cin, std::cout);
        RRTStarSettings settings;
        MotionPlanner planner(ROBOT_XML_PATH, scene, settings);
        const bool fixed_goal = planner.sceneGoal().has_value();
        const Pose goal = fixed_goal ? *planner.sceneGoal() : planner.sampleGoal();
        std::cout << (fixed_goal ? "Using the scene's fixed goal.\n" : "Using a random goal.\n");
        std::cout << "Planning with " << (settings.use_rrt_star ? "RRT*" : "RRT") << ".\n";
        const auto result = planner.plan(goal, PlannerMode::SampledScLERP);
        if (!result.success) {
            return debugPlannerFailure(result.failure_reason, result.stats, result.tree,
                                       scene, goal, settings.animation_frames_per_edge);
        }
        std::cout << "Goal reached with " << result.tree.size() << " nodes in "
                  << result.planning_seconds << " s.\n";
        const std::string video_path = kSaveTrialVideo ? trialVideoPath(scene) : "";
        return MujocoViewer::play(scene, result.trajectory, planner.homePose(), goal, video_path);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

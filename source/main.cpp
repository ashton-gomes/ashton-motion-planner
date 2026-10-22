#include "environment_selector.hpp"
#include "planner.hpp"
#include "viewer.hpp"

#include <iostream>

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
        return MujocoViewer::play(scene, result.trajectory, planner.homePose(), goal);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

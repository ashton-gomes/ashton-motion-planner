#pragma once

#include "gplanner/planner.hpp"

#include <functional>
#include <string>

// Visit every edge in both directions, following the current parent links.
std::vector<Eigen::VectorXd> buildTreeDebugTrajectory(
    const std::vector<RRTNode>& tree, int frames_per_edge);

// Report the observed failure and replay the partial tree. Always returns 1.
// to_scene_qpos turns planned-joint values into a full scene qpos
// (MotionPlanner::toSceneQpos). overlay boxes are drawn over the scene.
int debugPlannerFailure(
    const std::string& reason,
    const PlannerDebugStats& stats,
    const std::vector<RRTNode>& tree,
    const std::string& scene_path,
    const Eigen::Matrix<double, 7, 1>& goal,
    int frames_per_edge,
    const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& to_scene_qpos,
    const std::vector<SceneObstacle>& overlay = {});

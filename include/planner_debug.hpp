#pragma once

#include "rrt_star.hpp"

#include <string>

// Count the first reason each candidate edge was rejected.
struct PlannerDebugStats {
    int expansions = 0;
    int no_progress = 0;
    int non_finite = 0;
    int self_collision = 0;
    int obstacle_collision = 0;
    int rmrc_not_converged = 0;
    int joint_path_collision = 0;
};

// Visit every edge in both directions, following the current parent links.
std::vector<Eigen::VectorXd> buildTreeDebugTrajectory(
    const std::vector<RRTNode>& tree, int frames_per_edge);

// Report the observed failure and replay the partial tree. Always returns 1.
int debugPlannerFailure(
    const std::string& reason,
    const PlannerDebugStats& stats,
    const std::vector<RRTNode>& tree,
    const std::string& scene_path,
    const Eigen::Matrix<double, 7, 1>& goal,
    int frames_per_edge);

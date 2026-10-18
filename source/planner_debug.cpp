#include "planner_debug.hpp"
#include "viewer.hpp"

#include <Eigen/Geometry>
#include <iostream>
#include <stdexcept>

std::vector<Eigen::VectorXd> buildTreeDebugTrajectory(
    const std::vector<RRTNode>& tree, int frames_per_edge)
{
    if (frames_per_edge <= 0) {
        throw std::invalid_argument("Tree playback frames per edge must be positive.");
    }
    if (tree.empty()) return {};

    // Rewiring can give a node a parent with a larger index.
    std::vector<std::vector<int>> children(tree.size());
    for (int node = 1; node < static_cast<int>(tree.size()); ++node) {
        children.at(tree[node].parent).push_back(node);
    }

    std::vector<Eigen::VectorXd> trajectory{tree.front().qpos};
    const auto append_edge = [&](int from, int to) {
        for (int frame = 1; frame <= frames_per_edge; ++frame) {
            const double t = static_cast<double>(frame) / frames_per_edge;
            trajectory.push_back((1.0 - t) * tree[from].qpos + t * tree[to].qpos);
        }
    };

    // Iterative depth-first traversal also handles long, narrow trees.
    std::vector<int> stack{0};
    std::vector<size_t> next_child(tree.size(), 0);
    while (!stack.empty()) {
        const int node = stack.back();
        if (next_child[node] < children[node].size()) {
            const int child = children[node][next_child[node]++];
            append_edge(node, child);
            stack.push_back(child);
        } else {
            stack.pop_back();
            if (!stack.empty()) append_edge(node, stack.back());
        }
    }
    return trajectory;
}

int debugPlannerFailure(
    const std::string& reason,
    const PlannerDebugStats& stats,
    const std::vector<RRTNode>& tree,
    const std::string& scene_path,
    const Eigen::Matrix<double, 7, 1>& goal,
    int frames_per_edge)
{
    std::cerr << "Planner failed: " << reason << '\n'
              << "Expansions attempted: " << stats.expansions
              << "; tree nodes: " << tree.size() << '\n'
              << "Rejected candidates (first failing check):\n"
              << "  No Cartesian progress: " << stats.no_progress << '\n'
              << "  Non-finite RMRC result: " << stats.non_finite << '\n'
              << "  Self-collision during RMRC: " << stats.self_collision << '\n'
              << "  Obstacle collision during RMRC: " << stats.obstacle_collision << '\n'
              << "  RMRC did not converge to the edge pose: " << stats.rmrc_not_converged << '\n'
              << "  Collision along joint-space playback: " << stats.joint_path_collision << '\n';

    if (tree.empty()) return 1;
    const int closest = findNearestNode(tree, goal.head<3>());
    const auto& pose = tree[closest].pose;
    const Eigen::Quaterniond rotation(pose[3], pose[4], pose[5], pose[6]);
    const Eigen::Quaterniond goal_rotation(goal[3], goal[4], goal[5], goal[6]);
    std::cerr << "Closest node to goal: " << closest
              << "; position error: " << (tree[closest].position - goal.head<3>()).norm()
              << " m; rotation error: " << rotation.angularDistance(goal_rotation) << " rad.\n"
              << "Replaying the current tree; close the viewer to exit.\n";

    if (MujocoViewer::play(scene_path, buildTreeDebugTrajectory(tree, frames_per_edge),
                           tree.front().pose, goal) != 0) {
        std::cerr << "Could not open the planner debug playback.\n";
    }
    return 1;
}

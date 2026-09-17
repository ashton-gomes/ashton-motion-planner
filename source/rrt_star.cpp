#include "rrt_star.hpp"

#include "sampler.hpp"

#include <limits>
#include <random>
#include <stdexcept>

namespace {

void validateSettings(const RRTStarSettings& settings) {
    if (settings.goal_bias < 0.0 || settings.goal_bias > 1.0) {
        throw std::invalid_argument("RRT* goal_bias must be in [0, 1].");
    }
    if (settings.samples_per_expansion <= 0) {
        throw std::invalid_argument("RRT* samples_per_expansion must be positive.");
    }
    if (settings.maximum_step <= 0.0) {
        throw std::invalid_argument("RRT* maximum_step must be positive.");
    }
    if (settings.rewire_radius <= 0.0) {
        throw std::invalid_argument("RRT* rewire_radius must be positive.");
    }
    if (settings.max_expansions <= 0 ||
        settings.rmrc_iterations_per_edge <= 0 ||
        settings.sclerp_steps_per_edge <= 0 ||
        settings.animation_frames_per_edge <= 0 ||
        settings.obstacle_sample_attempts <= 0) {
        throw std::invalid_argument("RRT* iteration counts must be positive.");
    }
    if (settings.obstacle_clearance < 0.0) {
        throw std::invalid_argument("RRT* obstacle_clearance cannot be negative.");
    }
}

// Recalculate all descendants after a node gets a cheaper parent.
void updateChildCosts(std::vector<RRTNode>& nodes, int parent_index) {
    for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
        if (nodes[i].parent != parent_index) {
            continue;
        }

        nodes[i].cost = nodes[parent_index].cost +
                        (nodes[i].position - nodes[parent_index].position).norm();
        updateChildCosts(nodes, i);
    }
}

} // namespace

std::vector<Eigen::Vector3d> sampleExpansionPoints(
    const WorkspaceSamplingBounds& workspace,
    const Eigen::Vector3d& goal,
    const std::vector<SceneObstacle>& obstacles,
    const RRTStarSettings& settings)
{
    validateSettings(settings);

    static std::mt19937 rng(std::random_device{}());
    std::bernoulli_distribution choose_goal(settings.goal_bias);

    std::vector<Eigen::Vector3d> samples;
    samples.reserve(settings.samples_per_expansion);
    for (int i = 0; i < settings.samples_per_expansion; ++i) {
        if (choose_goal(rng)) {
            samples.push_back(goal);
        } else {
            samples.push_back(sampleWorkspacePointAvoidingObstacles(
                workspace.center,
                workspace.inner_radius,
                workspace.outer_radius,
                obstacles,
                settings.obstacle_sample_attempts));
        }
    }
    return samples;
}

int findNearestNode(const std::vector<RRTNode>& nodes,
                    const Eigen::Vector3d& point)
{
    if (nodes.empty()) {
        throw std::invalid_argument("Cannot find a nearest node in an empty tree.");
    }

    int nearest_index = 0;
    double nearest_distance_squared =
        (nodes.front().position - point).squaredNorm();

    for (int i = 1; i < static_cast<int>(nodes.size()); ++i) {
        const double distance_squared =
            (nodes[i].position - point).squaredNorm();
        if (distance_squared < nearest_distance_squared) {
            nearest_index = i;
            nearest_distance_squared = distance_squared;
        }
    }
    return nearest_index;
}

Eigen::Vector3d steerTowards(const Eigen::Vector3d& from,
                              const Eigen::Vector3d& target,
                              double maximum_step)
{
    if (maximum_step <= 0.0) {
        throw std::invalid_argument("RRT* maximum_step must be positive.");
    }

    const Eigen::Vector3d direction = target - from;
    const double distance = direction.norm();
    if (distance <= maximum_step || distance == 0.0) {
        return target;
    }
    return from + (maximum_step / distance) * direction;
}

std::vector<int> findNearbyNodes(const std::vector<RRTNode>& nodes,
                                 const Eigen::Vector3d& point,
                                 double radius)
{
    if (radius <= 0.0) {
        throw std::invalid_argument("RRT* nearby-node radius must be positive.");
    }

    std::vector<int> nearby;
    const double radius_squared = radius * radius;
    for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
        if ((nodes[i].position - point).squaredNorm() <= radius_squared) {
            nearby.push_back(i);
        }
    }
    return nearby;
}

void rewireNode(std::vector<RRTNode>& nodes,
                int node_index,
                int new_parent_index)
{
    const int node_count = static_cast<int>(nodes.size());
    if (node_index <= 0 || node_index >= node_count ||
        new_parent_index < 0 || new_parent_index >= node_count) {
        throw std::out_of_range("RRT* rewire node index is invalid.");
    }

    // Do not allow a node to become a child of itself or one of its children.
    for (int parent = new_parent_index; parent >= 0; parent = nodes[parent].parent) {
        if (parent == node_index) {
            throw std::invalid_argument("RRT* rewire would create a cycle.");
        }
    }

    nodes[node_index].parent = new_parent_index;
    nodes[node_index].cost = nodes[new_parent_index].cost +
        (nodes[node_index].position - nodes[new_parent_index].position).norm();
    updateChildCosts(nodes, node_index);
}

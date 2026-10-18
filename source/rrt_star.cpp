#include "rrt_star.hpp"

#include "sampler.hpp"

#include <limits>
#include <random>
#include <stdexcept>
#include <utility>

void validateRRTSettings(const RRTStarSettings& settings) {
    if (settings.use_goal_bias && (settings.goal_bias < 0.0 || settings.goal_bias > 1.0)) {
        throw std::invalid_argument("RRT goal_bias must be in [0, 1].");
    }
    if (settings.samples_per_expansion <= 0) {
        throw std::invalid_argument("RRT samples_per_expansion must be positive.");
    }
    if (settings.maximum_step <= 0.0) {
        throw std::invalid_argument("RRT maximum_step must be positive.");
    }
    if (settings.use_rrt_star && settings.rewire_radius <= 0.0) {
        throw std::invalid_argument("RRT* rewire_radius must be positive.");
    }
    if (settings.max_expansions <= 0 ||
        settings.rmrc_iterations_per_edge <= 0 ||
        settings.sclerp_steps_per_edge <= 0 ||
        settings.animation_frames_per_edge <= 0 ||
        settings.obstacle_sample_attempts <= 0) {
        throw std::invalid_argument("RRT iteration counts must be positive.");
    }
    if (settings.obstacle_clearance < 0.0) {
        throw std::invalid_argument("RRT obstacle_clearance cannot be negative.");
    }
}

namespace {
std::mt19937 expansionRng(std::random_device{}());

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

void seedRRTSampling(std::uint32_t seed) {
    expansionRng.seed(seed + 1001u);
}

Eigen::Vector3d sampleExpansionPoint(
    const WorkspaceSamplingBounds& workspace,
    const Eigen::Vector3d& goal,
    const std::vector<SceneObstacle>& obstacles,
    const RRTStarSettings& settings,
    int attempt)
{
    validateRRTSettings(settings);
    if (attempt < 0) {
        throw std::invalid_argument("Expansion attempt cannot be negative.");
    }
    if (!settings.use_goal_bias && attempt == 0) return goal;

    if (settings.use_goal_bias && std::bernoulli_distribution(settings.goal_bias)(expansionRng)) {
        return goal;
    }
    return sampleWorkspacePointAvoidingObstacles(
        workspace.center, workspace.inner_radius, workspace.outer_radius,
        obstacles, settings.obstacle_sample_attempts);
}

std::vector<Eigen::Vector3d> sampleExpansionPoints(
    const WorkspaceSamplingBounds& workspace,
    const Eigen::Vector3d& goal,
    const std::vector<SceneObstacle>& obstacles,
    const RRTStarSettings& settings)
{
    validateRRTSettings(settings);
    const int attempts = settings.samples_per_expansion + (settings.use_goal_bias ? 0 : 1);
    std::vector<Eigen::Vector3d> samples;
    samples.reserve(attempts);
    for (int attempt = 0; attempt < attempts; ++attempt) {
        samples.push_back(sampleExpansionPoint(workspace, goal, obstacles, settings, attempt));
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

int selectGoalExpansionNode(const std::vector<RRTNode>& nodes,
                            const Eigen::Vector3d& goal,
                            std::vector<bool>& attempted)
{
    attempted.resize(nodes.size(), false);
    if (nodes.empty()) return -1;

    int selected = -1;
    if (!attempted.back()) {
        selected = static_cast<int>(nodes.size()) - 1;
    } else {
        double nearest_distance = std::numeric_limits<double>::infinity();
        for (int node = 0; node < static_cast<int>(nodes.size()); ++node) {
            if (attempted[node]) continue;
            const double distance = (nodes[node].position - goal).squaredNorm();
            if (distance < nearest_distance) {
                selected = node;
                nearest_distance = distance;
            }
        }
    }
    if (selected >= 0) attempted[selected] = true;
    return selected;
}

Eigen::Vector3d steerTowards(const Eigen::Vector3d& from,
                              const Eigen::Vector3d& target,
                              double maximum_step)
{
    if (maximum_step <= 0.0) {
        throw std::invalid_argument("RRT maximum_step must be positive.");
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

int addRRTNode(
    std::vector<RRTNode>& nodes,
    RRTNode node,
    const RRTStarSettings& settings,
    const std::function<bool(const Eigen::VectorXd&, const Eigen::VectorXd&)>& is_edge_valid)
{
    std::vector<int> nearby;
    if (settings.use_rrt_star) {
        nearby = findNearbyNodes(nodes, node.position, settings.rewire_radius);
        for (const int parent : nearby) {
            const double cost = nodes[parent].cost +
                (node.position - nodes[parent].position).norm();
            if (cost < node.cost && is_edge_valid(nodes[parent].qpos, node.qpos)) {
                node.parent = parent;
                node.cost = cost;
            }
        }
    }

    nodes.push_back(std::move(node));
    const int new_index = static_cast<int>(nodes.size()) - 1;
    for (const int neighbor : nearby) {
        if (neighbor == 0 || neighbor == nodes[new_index].parent) continue;
        const double cost = nodes[new_index].cost +
            (nodes[neighbor].position - nodes[new_index].position).norm();
        if (cost < nodes[neighbor].cost &&
            is_edge_valid(nodes[new_index].qpos, nodes[neighbor].qpos)) {
            rewireNode(nodes, neighbor, new_index);
        }
    }
    return new_index;
}

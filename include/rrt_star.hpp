#pragma once

#include <Eigen/Dense>
#include "sampler.hpp"

#include <functional>
#include <vector>

void seedRRTSampling(std::uint32_t seed);

// Shared RRT/RRT* settings. Disable use_rrt_star for plain RRT.
struct RRTStarSettings {
    bool use_rrt_star = false;        // Cheapest-parent selection and rewiring.
    bool use_goal_bias = false;      // Enable the original probabilistic sampling.
    double goal_bias = 0.30;          // Goal probability when use_goal_bias is true.
    int samples_per_expansion = 10;   // Random fallback attempts or biased samples.
    double maximum_step = 0.05;       // Maximum Cartesian edge length, in metres.
    double rewire_radius = 0.10;       // Radius used to choose parents and rewire.
    int max_expansions = 1000;         // Planning batches before returning the best path.
    int rmrc_iterations_per_edge = 100;
    int sclerp_steps_per_edge = 20;
    int animation_frames_per_edge = 24;
    double obstacle_clearance = 0.02; // Extra distance around scene obstacles.
    int obstacle_sample_attempts = 1000;
};

void validateRRTSettings(const RRTStarSettings& settings);

struct RRTNode {
    Eigen::Vector3d position;
    // Pose/configuration are needed to start the next ScLERP + RMRC edge.
    Eigen::Matrix<double, 7, 1> pose;
    Eigen::VectorXd qpos;
    int parent = -1;
    double cost = 0.0;
};

struct WorkspaceSamplingBounds {
    Eigen::Vector3d center;
    double inner_radius = 0.0;
    double outer_radius = 0.0;
};

// Step 1: choose one target lazily. With goal bias off, attempt 0 is the goal
// and later attempts are random. With it on, each attempt uses goal_bias.
Eigen::Vector3d sampleExpansionPoint(
    const WorkspaceSamplingBounds& workspace,
    const Eigen::Vector3d& goal,
    const std::vector<SceneObstacle>& obstacles,
    const RRTStarSettings& settings,
    int attempt);

// Batch variant: goal plus fallback targets, or the original biased samples.
std::vector<Eigen::Vector3d> sampleExpansionPoints(
    const WorkspaceSamplingBounds& workspace,
    const Eigen::Vector3d& goal,
    const std::vector<SceneObstacle>& obstacles,
    const RRTStarSettings& settings);

// Step 2: locate the closest existing Cartesian RRT node.
int findNearestNode(const std::vector<RRTNode>& nodes,
                    const Eigen::Vector3d& point);

// For a fixed goal and fixed node configurations, attempt each goal edge once.
// Prefer the newest node, then the nearest untried node; return -1 if none remain.
// Remember successful attempts too, since repeating them creates duplicate nodes.
int selectGoalExpansionNode(const std::vector<RRTNode>& nodes,
                            const Eigen::Vector3d& goal,
                            std::vector<bool>& attempted);

// Step 3: advance from one point toward a target without exceeding maximum_step.
Eigen::Vector3d steerTowards(const Eigen::Vector3d& from,
                              const Eigen::Vector3d& target,
                              double maximum_step);

// Step 4: return every tree node inside the RRT* rewiring radius.
std::vector<int> findNearbyNodes(const std::vector<RRTNode>& nodes,
                                 const Eigen::Vector3d& point,
                                 double radius);

// Step 5: give a node a cheaper parent and update the costs below it.
void rewireNode(std::vector<RRTNode>& nodes,
                int node_index,
                int new_parent_index);

// Insert a validated candidate with its nearest-node parent. In RRT* mode,
// choose a cheaper valid parent and rewire neighbors using is_edge_valid.
// Returns the new node index; plain RRT never calls is_edge_valid here.
int addRRTNode(
    std::vector<RRTNode>& nodes,
    RRTNode node,
    const RRTStarSettings& settings,
    const std::function<bool(const Eigen::VectorXd&, const Eigen::VectorXd&)>& is_edge_valid);

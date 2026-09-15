#pragma once

#include <Eigen/Dense>
#include "sampler.hpp"

#include <vector>

// Settings kept together so the RRT* behavior can be adjusted from main.
struct RRTStarSettings {
    double goal_bias = 0.20;          // Probability of sampling the goal.
    int samples_per_expansion = 10;   // Candidate samples for one expansion.
    double maximum_step = 0.05;       // Maximum Cartesian edge length, in metres.
    int max_expansions = 200;         // Stop if the tree has not reached the goal.
    int rmrc_iterations_per_edge = 100;
    int sclerp_steps_per_edge = 20;
    int animation_frames_per_edge = 24;
    double obstacle_clearance = 0.02; // Extra distance around scene obstacles.
    int obstacle_sample_attempts = 1000;
};

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

// Step 1: create a configurable batch of workspace samples.  Each sample has
// goal_bias probability of being exactly goal; otherwise it is random.
std::vector<Eigen::Vector3d> sampleExpansionPoints(
    const WorkspaceSamplingBounds& workspace,
    const Eigen::Vector3d& goal,
    const std::vector<SceneObstacle>& obstacles,
    const RRTStarSettings& settings);

// Step 2: locate the closest existing Cartesian RRT node.
int findNearestNode(const std::vector<RRTNode>& nodes,
                    const Eigen::Vector3d& point);

// Step 3: advance from from toward target without exceeding maximum_step.
Eigen::Vector3d steerTowards(const Eigen::Vector3d& from,
                              const Eigen::Vector3d& target,
                              double maximum_step);

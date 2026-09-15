#pragma once
#include <random>
#include <Eigen/Dense>

// Plain uniform workspace sampler used by the single-tree RRT.  Collision
// validity is deliberately left to the configuration-space collision check using Mujoco


// Takes the workspace and finds a random point within it 
class CandidateSampler {
public:
    CandidateSampler(const Eigen::Vector3d& lo, const Eigen::Vector3d& hi, unsigned int seed = 1337);

    // returns a random point from 0 to 1
    Eigen::Vector3d sampleWorkspacePoint();
    double rollUniform() { return uni01_(rng_); }

private:
    Eigen::Vector3d ws_min_, ws_max_;
    std::mt19937 rng_;
    std::uniform_real_distribution<double> uni01_;
};

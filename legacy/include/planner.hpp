#pragma once
#include <vector>
#include <Eigen/Dense>
#include <mujoco/mujoco.h>
#include "sampler.hpp"

struct RRTNode {
    Eigen::VectorXd q;
    Eigen::Vector3d p;
    Eigen::Matrix3d R;
    int parent;
};

using Tree = std::vector<RRTNode>;

struct SingleTreePlanResult {
    bool reached = false;
    Tree tree;
    std::vector<int> path;
};

void slerp(const Eigen::Vector3d& p1, const Eigen::Matrix3d& R1,
            const Eigen::Vector3d& p2, const Eigen::Matrix3d& R2,
            double t, Eigen::Vector3d& p_out, Eigen::Matrix3d& R_out);

Eigen::Matrix3d coupledOrientation(const Eigen::Vector3d& p_near, const Eigen::Matrix3d& R_near,
                                   const Eigen::Vector3d& p_anchor, const Eigen::Matrix3d& R_anchor,
                                   const Eigen::Vector3d& p_rand);

SingleTreePlanResult planSingleTreeRRT(mjModel* m, mjData* d, CandidateSampler& sampler,
                                       const Eigen::VectorXd& q_start,
                                       const Eigen::Vector3d& p_goal,
                                       const Eigen::Matrix3d& R_goal);

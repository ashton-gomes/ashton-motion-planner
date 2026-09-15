#include "planner.hpp"
#include "config.hpp"
#include "kinematics.hpp"

#include <algorithm>

void interpolation(const Eigen::Vector3d& p1, const Eigen::Matrix3d& R1,
            const Eigen::Vector3d& p2, const Eigen::Matrix3d& R2,
            double t, Eigen::Vector3d& p_out, Eigen::Matrix3d& R_out) {
    p_out = (1.0 - t) * p1 + t * p2;
    R_out = Eigen::Quaterniond(R1).slerp(t, Eigen::Quaterniond(R2)).toRotationMatrix();
}

Eigen::Matrix3d coupledOrientation(const Eigen::Vector3d& p_near, const Eigen::Matrix3d& R_near,
                                   const Eigen::Vector3d& p_goal, const Eigen::Matrix3d& R_goal,
                                   const Eigen::Vector3d& p_rand) {
    const double total_distance = (p_goal - p_near).norm();
    const double alpha = total_distance < 1e-6 ? 1.0 :
        std::clamp((p_rand - p_near).norm() / total_distance, 0.0, 1.0);
    return Eigen::Quaterniond(R_near).slerp(alpha, Eigen::Quaterniond(R_goal)).toRotationMatrix();
}

static int nearestIndex(const Tree& tree, const Eigen::Vector3d& p_query) {
    int best = 0;
    double best_distance = (tree[0].p - p_query).norm();
    for (size_t i = 1; i < tree.size(); ++i) {
        const double distance = (tree[i].p - p_query).norm();
        if (distance < best_distance) {
            best = static_cast<int>(i);
            best_distance = distance;
        }
    }
    return best;
}

// The single-tree RRT extension from main_new.cpp: add at most one node.
static bool expandOneStep(mjModel* m, mjData* d, Tree& tree, int near_idx,
                          const Eigen::Vector3d& p_target, const Eigen::Matrix3d& R_target) {
    const RRTNode& near = tree[near_idx];
    const double pos_distance = (p_target - near.p).norm();
    const double angle_distance = rotationError(near.R, R_target).norm();
    const double t_pos = pos_distance < 1e-9 ? 1.0 :
        std::min(1.0, config::RRT_STEP_POS / pos_distance);
    const double t_angle = angle_distance < 1e-9 ? 1.0 :
        std::min(1.0, config::RRT_STEP_ANG / angle_distance);

    Eigen::Vector3d p_step;
    Eigen::Matrix3d R_step;
    interpolation(near.p, near.R, p_target, R_target, std::min(t_pos, t_angle), p_step, R_step);

    Eigen::VectorXd q_next;
    if (!solveIK_RMRC(m, d, near.q, p_step, R_step, q_next) ||
        !checkJointLimits(q_next) || checkCollision(m, d, q_next)) {
        return false;
    }

    RRTNode node;
    node.q = q_next;
    Eigen::MatrixXd unused_jacobian;
    getPoseAndJacobianAt(m, d, q_next, node.p, node.R, unused_jacobian);
    node.parent = near_idx;
    tree.push_back(std::move(node));
    return true;
}

SingleTreePlanResult planSingleTreeRRT(mjModel* m, mjData* d, CandidateSampler& sampler,
                                       const Eigen::VectorXd& q_start,
                                       const Eigen::Vector3d& p_goal,
                                       const Eigen::Matrix3d& R_goal) {
    SingleTreePlanResult result;
    RRTNode root;
    root.q = q_start;
    Eigen::MatrixXd unused_jacobian;
    getPoseAndJacobianAt(m, d, q_start, root.p, root.R, unused_jacobian);
    root.parent = -1;
    result.tree.push_back(std::move(root));

    int goal_idx = -1;
    while (static_cast<int>(result.tree.size()) < config::RRT_MAX_NODES) {
        const bool goal_sample = sampler.rollUniform() < config::RRT_GOAL_BIAS;
        const Eigen::Vector3d p_rand = goal_sample ? p_goal : sampler.sampleWorkspacePoint();
        const int near_idx = nearestIndex(result.tree, p_rand);
        const Eigen::Matrix3d R_rand = goal_sample ? R_goal :
            coupledOrientation(result.tree[near_idx].p, result.tree[near_idx].R,
                               p_goal, R_goal, p_rand);

        if (expandOneStep(m, d, result.tree, near_idx, p_rand, R_rand)) {
            const RRTNode& latest = result.tree.back();
            if ((latest.p - p_goal).norm() < config::GOAL_TOL_POS &&
                rotationError(latest.R, R_goal).norm() < config::GOAL_TOL_ANG) {
                goal_idx = static_cast<int>(result.tree.size()) - 1;
                result.reached = true;
                break;
            }
        }
    }

    if (result.reached) {
        for (int index = goal_idx; index != -1; index = result.tree[index].parent) {
            result.path.push_back(index);
        }
        std::reverse(result.path.begin(), result.path.end());
    }
    return result;
}

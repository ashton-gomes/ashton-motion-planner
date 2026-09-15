#include "kinematics.hpp"
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>

double normalizeAngle(double angle) {
    while (angle > M_PI)  angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

Eigen::Vector3d rotationError(const Eigen::Matrix3d& R_cur, const Eigen::Matrix3d& R_des) {
    Eigen::Matrix3d R_err = R_des * R_cur.transpose();
    double cos_theta = std::clamp(0.5 * (R_err.trace() - 1.0), -1.0, 1.0);
    double theta = std::acos(cos_theta);
    if (std::abs(theta) < 1e-6) return Eigen::Vector3d::Zero();
    double factor = theta / (2.0 * std::sin(theta));
    return factor * Eigen::Vector3d(R_err(2,1) - R_err(1,2),
                                    R_err(0,2) - R_err(2,0),
                                    R_err(1,0) - R_err(0,1));
}

Eigen::Vector3d getEEPosition(const mjModel* m, const mjData* d) {
    int id = mj_name2id(m, mjOBJ_SITE, config::EE_SITE);
    return Eigen::Vector3d(d->site_xpos[3*id], d->site_xpos[3*id+1], d->site_xpos[3*id+2]);
}

Eigen::Matrix3d getEERotation(const mjModel* m, const mjData* d) {
    int id = mj_name2id(m, mjOBJ_SITE, config::EE_SITE);
    Eigen::Matrix3d R;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R(i, j) = d->site_xmat[9*id + 3*i + j];
    return R;
}

Eigen::MatrixXd getEEJacobian6(const mjModel* m, mjData* d) {
    int id = mj_name2id(m, mjOBJ_SITE, config::EE_SITE);
    std::vector<double> jacp(3 * m->nv, 0.0), jacr(3 * m->nv, 0.0);
    mj_jacSite(m, d, jacp.data(), jacr.data(), id);

    Eigen::MatrixXd J(6, config::N_JOINTS);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < config::N_JOINTS; ++c) {
            J(r, c)     = jacp[r * m->nv + c];
            J(r + 3, c) = jacr[r * m->nv + c];
        }
    }
    return J;
}

void getPoseAndJacobianAt(const mjModel* m, mjData* d, const Eigen::VectorXd& q,
                          Eigen::Vector3d& p, Eigen::Matrix3d& R, Eigen::MatrixXd& J) {
    std::vector<double> qpos_save(m->nq);  // array for Mujoco generalized position
    for (int i = 0; i < m->nq; ++i) qpos_save[i] = d->qpos[i]; // mujoco position into a backup array

    for (int i = 0; i < config::N_JOINTS; ++i) d->qpos[i] = q(i); // joint positions equal to given q
    mj_kinematics(m, d);
    mj_comPos(m, d);

    p = getEEPosition(m, d);
    R = getEERotation(m, d);
    J = getEEJacobian6(m, d);

    for (int i = 0; i < m->nq; ++i) d->qpos[i] = qpos_save[i];
    mj_kinematics(m, d);
    mj_comPos(m, d);
}

bool checkJointLimits(const Eigen::VectorXd& q) {
    for (int i = 0; i < config::N_JOINTS; ++i) {
        if (q(i) < config::Q_LIM_LOW[i] || q(i) > config::Q_LIM_HIGH[i]) return false;
    }
    return true;
}

bool checkCollision(const mjModel* m, mjData* d, const Eigen::VectorXd& q) {
    std::vector<double> qpos_save(m->nq);
    for (int i = 0; i < m->nq; ++i) qpos_save[i] = d->qpos[i];

    for (int i = 0; i < config::N_JOINTS; ++i) d->qpos[i] = q(i);
    mj_kinematics(m, d);
    mj_collision(m, d);

    bool collide = false;
    for (int i = 0; i < d->ncon; ++i) {
        int g1 = d->contact[i].geom1;
        int g2 = d->contact[i].geom2;
        std::string n1 = (m->name_geomadr[g1] >= 0) ? std::string(m->names + m->name_geomadr[g1]) : "";
        std::string n2 = (m->name_geomadr[g2] >= 0) ? std::string(m->names + m->name_geomadr[g2]) : "";
        if (n1.find("obs_") == 0 || n2.find("obs_") == 0 || n1 == "table" || n2 == "table") {
            collide = true;
            break;
        }
    }

    for (int i = 0; i < m->nq; ++i) d->qpos[i] = qpos_save[i];
    mj_kinematics(m, d);
    return collide;
}

bool solveIK_RMRC(const mjModel* m, mjData* d, const Eigen::VectorXd& q_init,
                  const Eigen::Vector3d& p_des, const Eigen::Matrix3d& R_des,
                  Eigen::VectorXd& q_out) {
    q_out = q_init;
    constexpr int MAX_IK_ITERS = 30;
    constexpr double IK_TOL = 2e-3;

    for (int i = 0; i < MAX_IK_ITERS; ++i) {
        Eigen::Vector3d p_cur; Eigen::Matrix3d R_cur; Eigen::MatrixXd J;
        getPoseAndJacobianAt(m, d, q_out, p_cur, R_cur, J);

        Eigen::VectorXd err(6);
        err.head(3) = p_des - p_cur;
        err.tail(3) = rotationError(R_cur, R_des);

        if (err.norm() < IK_TOL) return true;

        constexpr double lambda2 = 0.01 * 0.01;
        Eigen::MatrixXd JJT = J * J.transpose();
        JJT.diagonal().array() += lambda2;
        Eigen::MatrixXd J_pinv = J.transpose() * JJT.inverse();

        Eigen::MatrixXd N = Eigen::MatrixXd::Identity(config::N_JOINTS, config::N_JOINTS) - J_pinv * J;
        Eigen::VectorXd q_null(config::N_JOINTS);
        for (int k = 0; k < config::N_JOINTS; ++k) q_null(k) = config::Q_MID[k] - q_out(k);

        Eigen::VectorXd dq = J_pinv * err + 0.1 * N * q_null;

        for (int k = 0; k < config::N_JOINTS; ++k) {
            dq(k) = normalizeAngle(dq(k));
            dq(k) = std::clamp(dq(k), -0.15, 0.15);
            q_out(k) += dq(k);
        }
    }
    return false;
}

#pragma once
#include <Eigen/Dense>
#include <mujoco/mujoco.h>
#include "config.hpp"

double normalizeAngle(double angle);
Eigen::Vector3d rotationError(const Eigen::Matrix3d& R_cur, const Eigen::Matrix3d& R_des);
Eigen::Vector3d getEEPosition(const mjModel* m, const mjData* d);
Eigen::Matrix3d getEERotation(const mjModel* m, const mjData* d);
Eigen::MatrixXd getEEJacobian6(const mjModel* m, mjData* d);

void getPoseAndJacobianAt(const mjModel* m, mjData* d, const Eigen::VectorXd& q,
                          Eigen::Vector3d& p, Eigen::Matrix3d& R, Eigen::MatrixXd& J);

bool checkJointLimits(const Eigen::VectorXd& q);
bool checkCollision(const mjModel* m, mjData* d, const Eigen::VectorXd& q);
bool solveIK_RMRC(const mjModel* m, mjData* d, const Eigen::VectorXd& q_init,
                  const Eigen::Vector3d& p_des, const Eigen::Matrix3d& R_des,
                  Eigen::VectorXd& q_out);
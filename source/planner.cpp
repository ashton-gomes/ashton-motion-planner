// current build includes a simple dual quaternion implementation to then actually use in the global planner.

#include <iostream>
#include <algorithm>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <stdexcept>
#include <vector>
#include <mujoco/mujoco.h>
#include "rrt_star.hpp"
#include "planner.hpp"
#include <chrono>
#include <array>
#include <limits>
#ifndef ROBOT_XML_PATH
#define ROBOT_XML_PATH "/home/ash_gomes/gplanner/mujoco_menagerie/kinova_gen3/gen3_obstacle.xml"
#endif



namespace {

const int n = 7;                  // DoF for robot arm, take fom URDF later

// General helpers (Quaternions and Dual Quaternions)

// skew symmetric matrix
Eigen::Matrix3d skew(const Eigen::Vector3d& p) {

    Eigen::Matrix3d p_hat;

    p_hat <<     0, -p[2],  p[1],
              p[2],     0, -p[0],
             -p[1],  p[0],     0;

    return p_hat;
}

// Quaternion arithmetic for dual components (which need not be unit quaternions).
Eigen::Quaterniond q_mult(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b) {
    return a * b;
}

Eigen::Quaterniond q_conj(const Eigen::Quaterniond& q) {
    return q.conjugate();
}

Eigen::Quaterniond q_scale(const Eigen::Quaterniond& q, double scale) {
    Eigen::Quaterniond result = q;
    result.coeffs() *= scale;
    return result;
}

Eigen::Quaterniond q_add(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b) {
    Eigen::Quaterniond result;
    result.coeffs() = a.coeffs() + b.coeffs();
    return result;
}

Eigen::Matrix<double, 3, 4> quaternionJacobian(
    const Eigen::Vector4d& q)
{
    double w = q[0];
    Eigen::Vector3d v = q.tail<3>();

    Eigen::Matrix<double, 3, 4> J1;

    J1.block<3,1>(0,0) = -v;

    J1.block<3,3>(0,1) =
        w * Eigen::Matrix3d::Identity()
        + skew(v);

    return J1;
}

struct DQ {
    Eigen::Quaterniond real = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond dual = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
};

// Keep printed components in the original scalar-first order.
std::ostream& operator<<(std::ostream& out, const DQ& dq) {
    return out << dq.real.w() << '\n' << dq.real.vec() << '\n'
               << dq.dual.w() << '\n' << dq.dual.vec();
}

DQ dq_mult(const DQ& A, const DQ& B) {
    return {A.real * B.real, q_add(A.dual * B.real, A.real * B.dual)};
}

DQ dq_add(const DQ& A, const DQ& B) {
    return {q_add(A.real, B.real), q_add(A.dual, B.dual)};
}

// Inverse for a dual quaternion with a unit real component.
DQ dq_inv(const DQ& A) {
    Eigen::Quaterniond a_inv = A.real.conjugate();
    return {a_inv, q_scale(a_inv * A.dual * a_inv, -1.0)};
}

DQ dq_conj(const DQ& A) {
    return {A.real.conjugate(), A.dual.conjugate()};
}

// DQ Power:

DQ dq_power(const DQ& dq, double tau) {

    Eigen::Vector3d v = dq.real.vec();
    double sin_half_theta = v.norm();
    double cos_half_theta = dq.real.w();

    // Compute angle robustly
    double theta = 2.0 * std::atan2(sin_half_theta, cos_half_theta);

    // 1st part: 2*A_d x A_r_conj
    Eigen::Quaterniond d_1 = q_scale(dq.dual * dq.real.conjugate(), 2.0);

    // 2. Compute unit axis safely
    Eigen::Vector3d u;
    if (sin_half_theta < 1e-8) {
        Eigen::Quaterniond d_tau(0.0, 0.0, 0.0, 0.0);
        d_tau.vec() = 0.5 * tau * d_1.vec();
        return {Eigen::Quaterniond::Identity(), d_tau};
    }

    // 2nd part after defining u: 2*A_d x A_r_conj * (0,l)^t
    u = v / sin_half_theta;
    double d = d_1.vec().dot(u);

    // m = 1/2 * (p x u + (p - du) * cot(theta/2)), where p = last 3 values the 1st part of d
    Eigen::Vector3d m_3d = 0.5 * (d_1.vec().cross(u) + (d_1.vec() - d*u) * ((std::cos(theta/2))/(std::sin(theta/2))));

    Eigen::Quaterniond m(0.0, m_3d.x(), m_3d.y(), m_3d.z());
    DQ u_DQ{Eigen::Quaterniond(0.0, u.x(), u.y(), u.z()), m};

    // sin(theta) = sin(theta/2) + e((d/2) * cos(theta/2))


    DQ u_sin_theta, u_cos_theta;

    double theta_tau = theta * tau;
    double d_tau = d * tau;

    // cos(theta) = cos(theta/2) - e((d/2) * sin(theta/2)), represented in a dual scalar
    u_cos_theta = {Eigen::Quaterniond(std::cos(theta_tau/2), 0, 0, 0),
                   Eigen::Quaterniond((-d_tau/2)*std::sin(theta_tau/2), 0, 0, 0)};

    // sin(theta) = sin(theta/2) + e((d/2) * cos(theta/2))
    double sin_real = std::sin(theta_tau/2);
    double sin_dual = (d_tau/2)*std::cos(theta_tau/2);

    // multiply by u_DQ for the formula in 2 parts, distributive property:
    u_sin_theta = {q_scale(u_DQ.real, sin_real),
                   q_add(q_scale(u_DQ.real, sin_dual), q_scale(u_DQ.dual, sin_real))};

    DQ A_t;
    A_t = dq_add(u_cos_theta, u_sin_theta);
    return A_t;
}


// Converting regular coordinates into a dual quaternion system:

// Making the rotation part for dual quaternion
Eigen::Quaterniond dq_r(const double& theta, const Eigen::Vector3d& axis) {
    return Eigen::Quaterniond(std::cos(theta/2), axis.x() * std::sin(theta/2),
                              axis.y() * std::sin(theta/2), axis.z() * std::sin(theta/2));
}

// Translation (dual) component; do not normalize this quaternion.
Eigen::Quaterniond dq_d(const Eigen::Vector3d& P, const Eigen::Quaterniond& q_r) {
    Eigen::Quaterniond p(0.0, P.x(), P.y(), P.z());
    return q_scale(p * q_r, 0.5);
}

DQ make_dq(const double& theta, const Eigen::Vector3d& axis, const Eigen::Vector3d& P) {
    Eigen::Quaterniond q_r = dq_r(theta, axis);
    return {q_r, dq_d(P, q_r)};
}

DQ ScLERP(const DQ& a_dq, const DQ& b_dq, double tau) {
    DQ end = b_dq;
    if (a_dq.real.dot(end.real) < 0.0) {
        end.real.coeffs() *= -1.0;
        end.dual.coeffs() *= -1.0;
    }
    DQ step1 = dq_mult(dq_inv(a_dq), end);
    DQ step2 = dq_power(step1, tau);
    DQ C_t = dq_mult(a_dq, step2);

    return C_t;
}

// Return a rotation partway from the current pose to the end pose.
Eigen::Quaterniond rot_diff(
    const Eigen::Matrix<double, 7, 1>& current,
    const Eigen::Matrix<double, 7, 1>& end,
    double tau)
{
    Eigen::Quaterniond current_rotation(
        current[3], current[4], current[5], current[6]);
    Eigen::Quaterniond end_rotation(end[3], end[4], end[5], end[6]);

    current_rotation.normalize();
    end_rotation.normalize();
    return current_rotation.slerp(tau, end_rotation).normalized();
}


// getting rotation from dual quaternion:

Eigen::Matrix3d get_rot(const Eigen::Matrix<double, 7, 1>& dq) {

    Eigen::Quaterniond q_rot(dq[3] ,dq[4], dq[5], dq[6]);
    q_rot.normalize();

    return q_rot.toRotationMatrix();

}

Eigen::Matrix<double, 6, 7> representationJacobian(
    const Eigen::Matrix<double, 7, 1>& pose)
{
    Eigen::Vector3d p = pose.head<3>();
    Eigen::Vector4d q = pose.tail<4>();

    q.normalize();

    Eigen::Matrix<double, 3, 4> J1 =
        quaternionJacobian(q);

    Eigen::Matrix<double, 6, 7> J2;
    J2.setZero();

    // linear velocity from position
    J2.block<3,3>(0,0) =
        Eigen::Matrix3d::Identity();

    // effect of quaternion rate on spatial linear velocity
    J2.block<3,4>(0,3) =
        2.0 * skew(p) * J1;

    // quaternion rate -> angular velocity
    J2.block<3,4>(3,3) =
        2.0 * J1;

    return J2;
}

//

// Unused below, using 4 x 4 transformation matrix and converting it instead of working entirely in quaternions

// DQ dq_transform(DQ& pose, DQ& DQ_transform)  {

//     DQ conj = dq_conj(DQ_transform);
//     DQ transform1 = (dq_mult(DQ_transform, pose));
//     DQ transform2 = dq_mult(transform1, conj);
//     return transform2;

// }

// given as a position (xyz) vector and quaternion for rotation
// Pose vectors use [px, py, pz, w, x, y, z]. Eigen coeffs() uses [x, y, z, w].
DQ pose_to_dq(const Eigen::Matrix<double, 7, 1>& pose) {
    Eigen::Quaterniond quat(pose[3], pose[4], pose[5], pose[6]);
    quat.normalize();
    return {quat, dq_d(pose.head<3>(), quat)};
}

Eigen::Matrix<double, 7, 1> dq_to_pose(const DQ& dq) {
    Eigen::Quaterniond translation = q_scale(dq.dual * dq.real.conjugate(), 2.0);
    Eigen::Matrix<double, 7, 1> pose;
    pose << translation.vec(), dq.real.w(), dq.real.x(), dq.real.y(), dq.real.z();
    return pose;
}

// Product of Exponentials Method

struct ScrewJoint {
    Eigen::Vector3d omega; // Unit direction of rotation axis in {S}
    Eigen::Vector3d q;     // Point on rotation axis in {S}
};

struct RobotModel {
    std::vector<ScrewJoint> joints;
    DQ M_dq; // Tool pose dual quaternion at theta = 0
    Eigen::Matrix<double, n, 1> home_qpos;
    Eigen::Matrix<double, 7, 1> home_pose;
    Eigen::Matrix<double, n, 1> joint_lower_limits;
    std::array<bool, n> joint_limited{};
    Eigen::Matrix<double, n, 1> joint_upper_limits;

    // Approximate Cartesian sampling workspace, centered at the base joint.
    Eigen::Vector3d workspace_center = Eigen::Vector3d::Zero();
    double workspace_outer_radius = 0.0;
    double workspace_inner_radius = 0.0;
};


// For spatial jacobian:

Eigen::Matrix<double, 6, 1> screwAxis(
    const ScrewJoint& joint)
{
    Eigen::Vector3d v =
        -joint.omega.cross(joint.q);

    Eigen::Matrix<double, 6, 1> S;

    S << v,
         joint.omega;

    return S;
}


// Adjoint:

Eigen::Matrix<double, 6, 6> adjoint(
    const Eigen::Matrix<double, 7, 1>& pose)
{
    Eigen::Vector3d p = pose.head<3>();

    Eigen::Matrix3d R =
        get_rot(pose);

    Eigen::Matrix<double, 6, 6> Ad;
    Ad.setZero();

    Ad.block<3,3>(0,0) = R;
    Ad.block<3,3>(0,3) = skew(p) * R;
    Ad.block<3,3>(3,3) = R;

    return Ad;
}


DQ exp_twist(const ScrewJoint& joint, double theta) {
    Eigen::Vector3d v = -joint.omega.cross(joint.q);

    double half_theta = 0.5 * theta;
    double sin_half = std::sin(half_theta);
    double cos_half = std::cos(half_theta);

    Eigen::Quaterniond q_r(cos_half, joint.omega[0] * sin_half, joint.omega[1] * sin_half, joint.omega[2] * sin_half);
    Eigen::Quaterniond q_d(0.0, v[0] * sin_half, v[1] * sin_half, v[2] * sin_half);

    return {q_r, q_d};
}


// Making spatial jacobian:

Eigen::Matrix<double, 6, n> spatialJacobian(
    const RobotModel& robot,
    const Eigen::Matrix<double, n, 1>& theta)
{
    Eigen::Matrix<double, 6, n> J;

    // DQ's default members give the identity: real = (1,0,0,0), dual = (0,0,0,0).
    DQ T{};


    for (int i = 0; i < n; ++i) {

        // Screw axis at the HOME configuration
        Eigen::Matrix<double, 6, 1> S =
            screwAxis(robot.joints[i]);


        if (i == 0) {

            // First joint has no previous transformation
            J.col(i) = S;

        } else {

            // Convert current cumulative DQ into pose
            Eigen::Matrix<double, 7, 1> pose =
                dq_to_pose(T);


            // Move screw axis into current spatial frame
            Eigen::Matrix<double, 6, 6> Ad =
                adjoint(pose);

            J.col(i) = Ad * S;
        }


        // Add this joint transformation for the NEXT joint
        DQ exp_i =
            exp_twist(robot.joints[i], theta[i]);

        T = dq_mult(T, exp_i);
    }


    return J;
}


// psuedoinverse:

Eigen::Matrix<double, n, 6> pseudoInverse(
    const Eigen::Matrix<double, 6, n>& J)
{
    Eigen::Matrix<double, 6, 6> JJt =
        J * J.transpose();

    Eigen::Matrix<double, n, 6> J_pinv =
        J.transpose() *
        (JJt + 1e-6 * Eigen::Matrix<double, 6, 6>::Identity())
            .ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());

    return J_pinv;
}


// B from paper:

Eigen::Matrix<double, n, 7> getB(
    const RobotModel& robot,
    const Eigen::Matrix<double, n, 1>& theta,
    const Eigen::Matrix<double, 7, 1>& pose)
{
    // Manipulator spatial Jacobian
    Eigen::Matrix<double, 6, n> Js =
        spatialJacobian(robot, theta);


    // Representation Jacobian
    Eigen::Matrix<double, 6, 7> J2 =
        representationJacobian(pose);


    // Pseudoinverse
    Eigen::Matrix<double, n, 6> Js_pinv =
        pseudoInverse(Js);


    // B = Js+ * J2
    Eigen::Matrix<double, n, 7> B =
        Js_pinv * J2;


    return B;
}

// RMRC:

Eigen::Matrix<double, n, 1> RMRC(
    const RobotModel& robot,
    const Eigen::Matrix<double, n, 1>& theta_current,
    const Eigen::Matrix<double, 7, 1>& pose_current,
    const Eigen::Matrix<double, 7, 1>& pose_next,
    double beta)
{
    Eigen::Matrix<double, n, 7> B =
        getB(
            robot,
            theta_current,
            pose_current
        );


    Eigen::Matrix<double, 7, 1> aligned_next = pose_next;
    if (pose_current.tail<4>().dot(aligned_next.tail<4>()) < 0.0)
        aligned_next.tail<4>() *= -1.0;
    Eigen::Matrix<double, 7, 1> pose_change = aligned_next - pose_current;


    Eigen::Matrix<double, n, 1> theta_next =
        theta_current
        + beta * B * pose_change;


    return theta_next;
}


// Extracting URDF:

RobotModel extractURDF(const std::string& xml_path) {
    char error[500];
    mjModel* m = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
    if (!m) {
        throw std::runtime_error(std::string("Robot model load error: ") + error);
    }
    mjData* d = mj_makeData(m);
    if (!d) {
        mj_deleteModel(m);
        throw std::runtime_error("Could not create robot kinematics data.");
    }

    // 1. Set all joints to zero and evaluate static geometry ONLY
    mju_zero(d->qpos, m->nq);
    mj_kinematics(m, d); // Computes Cartesian positions without physics

    RobotModel robot;
    robot.joints.resize(m->njnt);

    if (m->nq != n || m->njnt != n) {
        mj_deleteData(d);
        mj_deleteModel(m);
        throw std::runtime_error("Expected a seven-DoF robot model.");
    }

    // 2. Read the screw axis and anchor point for each joint
    for (int i = 0; i < m->njnt; ++i) {
        robot.joints[i].omega = Eigen::Map<Eigen::Vector3d>(&d->xaxis[3 * i]).normalized();
        robot.joints[i].q     = Eigen::Map<Eigen::Vector3d>(&d->xanchor[3 * i]);
        // One full turn suffices for the arm's continuous hinge joints.
        const double pi = std::acos(-1.0);
        robot.joint_limited[i] = m->jnt_limited[i];
        robot.joint_lower_limits[i] = m->jnt_limited[i] ? m->jnt_range[2 * i] : -pi;
        robot.joint_upper_limits[i] = m->jnt_limited[i] ? m->jnt_range[2 * i + 1] : pi;
    }

    // 3. Read the tool frame pose (last body in the kinematic chain)
    int tool_id = m->nbody - 1;
    Eigen::Vector3d p_home = Eigen::Map<Eigen::Vector3d>(&d->xpos[3 * tool_id]);
    // MuJoCo stores [w, x, y, z]; Eigen's pointer constructor expects [x, y, z, w].
    const mjtNum* raw_quat = &d->xquat[4 * tool_id];
    Eigen::Quaterniond q_home(raw_quat[0], raw_quat[1], raw_quat[2], raw_quat[3]);

    Eigen::Matrix<double, 7, 1> home_pose;
    home_pose << p_home, q_home.w(), q_home.x(), q_home.y(), q_home.z();
    robot.M_dq = pose_to_dq(home_pose);

    // Approximate the arm reach from distances between joint anchors.  This is
    // a cheap sampling bound, not a guarantee that every sampled pose has IK.
    robot.workspace_center = robot.joints.front().q;
    double total_link_length = 0.0;
    for (int i = 1; i < n; ++i) {
        total_link_length +=
            (robot.joints[i].q - robot.joints[i - 1].q).norm();
    }
    total_link_length += (p_home - robot.joints.back().q).norm();

    // Avoid full-extension singularities and reject points near the base.
    robot.workspace_outer_radius = 0.95 * total_link_length;
    robot.workspace_inner_radius =
        0.5 * (robot.joints[1].q - robot.joints[0].q).norm();

    // Use the model's named home keyframe rather than duplicating its joint
    // positions in the planner.  Evaluate its tool pose with MuJoCo as well.
    const int home_key_id = mj_name2id(m, mjOBJ_KEY, "home");
    if (home_key_id < 0) {
        mj_deleteData(d);
        mj_deleteModel(m);
        throw std::runtime_error("Model does not define a 'home' keyframe.");
    }
    for (int i = 0; i < n; ++i)
        robot.home_qpos[i] = m->key_qpos[home_key_id * m->nq + i];

    std::copy(robot.home_qpos.data(), robot.home_qpos.data() + n, d->qpos);
    mj_kinematics(m, d);
    const Eigen::Vector3d p_initial = Eigen::Map<Eigen::Vector3d>(&d->xpos[3 * tool_id]);
    const mjtNum* initial_quat = &d->xquat[4 * tool_id];
    robot.home_pose << p_initial, initial_quat[0], initial_quat[1],
                       initial_quat[2], initial_quat[3];

    // 4. Discard MuJoCo completely
    mj_deleteData(d);
    mj_deleteModel(m);

    return robot;
}

bool isInsideWorkspace(const RobotModel& robot,
                       const Eigen::Vector3d& point) {
    const double radius = (point - robot.workspace_center).norm();
    return radius >= robot.workspace_inner_radius &&
           radius <= robot.workspace_outer_radius;
}

// Uses a second MuJoCo model only for collision queries.  ROBOT_XML_PATH
// contains the arm alone, so any reported contact is a self-collision.
class SelfCollisionChecker {
public:
    explicit SelfCollisionChecker(const std::string& xml_path) {
        char error[500]{};
        model_ = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
        if (!model_) {
            throw std::runtime_error(std::string("Collision model load error: ") + error);
        }
        data_ = mj_makeData(model_);
        if (!data_) {
            mj_deleteModel(model_);
            throw std::runtime_error("Could not create collision-check data.");
        }
    }

    ~SelfCollisionChecker() {
        mj_deleteData(data_);
        mj_deleteModel(model_);
    }

    Pose toolPose(const Eigen::VectorXd& qpos) {
        std::copy(qpos.data(), qpos.data() + n, data_->qpos);
        mj_kinematics(model_, data_);
        const int tool = mj_name2id(model_, mjOBJ_BODY, "bracelet_link");
        if (tool < 0) throw std::runtime_error("Missing bracelet_link tool frame.");
        Pose pose;
        pose << Eigen::Map<Eigen::Vector3d>(&data_->xpos[3 * tool]),
                data_->xquat[4 * tool], data_->xquat[4 * tool + 1],
                data_->xquat[4 * tool + 2], data_->xquat[4 * tool + 3];
        return pose;
    }

    bool hasSelfCollision(const Eigen::Matrix<double, n, 1>& qpos) {
        std::copy(qpos.data(), qpos.data() + n, data_->qpos);
        mj_forward(model_, data_);

        for (int i = 0; i < data_->ncon; ++i) {
            const mjContact& contact = data_->contact[i];
            const int body1 = model_->geom_bodyid[contact.geom1];
            const int body2 = model_->geom_bodyid[contact.geom2];

            // Collision meshes of links joined directly together overlap at
            // their joint.  Only non-neighboring link contacts are rejected.
            const bool same_body = body1 == body2;
            const bool directly_connected =
                model_->body_parentid[body1] == body2 ||
                model_->body_parentid[body2] == body1;
            if (!same_body && !directly_connected) {
                return true;
            }
        }
        return false;
    }

private:
    mjModel* model_ = nullptr;
    mjData* data_ = nullptr;
};

// Checks robot collision meshes (group 3) against non-plane physical geoms
// from scene.xml.  The obstacle margin is enlarged for a conservative path.
class ObstacleCollisionChecker {
public:
    ObstacleCollisionChecker(const std::string& scene_path, double clearance) {
        char error[500]{};
        model_ = mj_loadXML(scene_path.c_str(), nullptr, error, sizeof(error));
        if (!model_) {
            throw std::runtime_error(std::string("Scene collision load error: ") + error);
        }
        applyCylinderPlacements(model_);
        for (int geom = 0; geom < model_->ngeom; ++geom) {
            if (isObstacleGeom(geom)) {
                model_->geom_margin[geom] += clearance;
            }
        }
        data_ = mj_makeData(model_);
        if (!data_) {
            mj_deleteModel(model_);
            throw std::runtime_error("Could not create scene collision data.");
        }
    }

    ~ObstacleCollisionChecker() {
        mj_deleteData(data_);
        mj_deleteModel(model_);
    }

    bool hasObstacleCollision(const Eigen::Matrix<double, n, 1>& qpos) {
        std::copy(qpos.data(), qpos.data() + n, data_->qpos);
        mj_forward(model_, data_);

        for (int i = 0; i < data_->ncon; ++i) {
            const mjContact& contact = data_->contact[i];
            if ((isRobotCollisionGeom(contact.geom1) && isObstacleGeom(contact.geom2)) ||
                (isRobotCollisionGeom(contact.geom2) && isObstacleGeom(contact.geom1))) {
                return true;
            }
        }
        return false;
    }

private:
    bool isRobotCollisionGeom(int geom) const {
        return model_->geom_group[geom] == 3;
    }

    bool isObstacleGeom(int geom) const {
        return !isRobotCollisionGeom(geom) &&
               model_->geom_type[geom] != mjGEOM_PLANE &&
               model_->geom_contype[geom] != 0 &&
               model_->geom_conaffinity[geom] != 0;
    }

    mjModel* model_ = nullptr;
    mjData* data_ = nullptr;
};

// Check the simple joint-space motion used to replay and rewire tree edges.
bool isJointPathCollisionFree(
    const Eigen::Matrix<double, n, 1>& from,
    const Eigen::Matrix<double, n, 1>& to,
    int checks,
    SelfCollisionChecker& self_collision_checker,
    ObstacleCollisionChecker& obstacle_collision_checker)
{
    for (int i = 1; i <= checks; ++i) {
        const double t = static_cast<double>(i) / checks;
        const Eigen::Matrix<double, n, 1> qpos = (1.0 - t) * from + t * to;
        if (!qpos.allFinite() ||
            self_collision_checker.hasSelfCollision(qpos) ||
            obstacle_collision_checker.hasObstacleCollision(qpos)) {
            return false;
        }
    }
    return true;
}


// Forward Kinematics via Dual Quaternion PoE
Eigen::Matrix<double, 7, 1> FK(const RobotModel& robot, const Eigen::Matrix<double, n, 1>& theta) {
    DQ T_cumulative;
    // Default DQ is the identity transform

    for (int i = 0; i < n; ++i) {
        DQ exp_i = exp_twist(robot.joints[i], theta[i]);
        T_cumulative = dq_mult(T_cumulative, exp_i);
    }

    DQ T_final = dq_mult(T_cumulative, robot.M_dq);
    return dq_to_pose(T_final);
}



} // namespace

struct MotionPlanner::Impl {
    RobotModel robot;
    RRTStarSettings settings;
    SelfCollisionChecker self;
    ObstacleCollisionChecker scene;
    std::vector<SceneObstacle> obstacles;
    static constexpr double tolerance = 1e-3;
    static constexpr double beta = 0.1;

    Impl(const std::string& robot_xml, const std::string& scene_xml,
         const RRTStarSettings& options)
        : robot(extractURDF(robot_xml)), settings(options), self(robot_xml),
          scene(scene_xml, options.obstacle_clearance),
          obstacles(loadSceneObstacles(scene_xml, options.obstacle_clearance)) {
        validateRRTSettings(settings);
    }

    bool collisionFree(const Eigen::VectorXd& from, const Eigen::VectorXd& to) {
        return isJointPathCollisionFree(from, to, settings.animation_frames_per_edge,
                                        self, scene);
    }

    bool atGoal(const Pose& reached, const Pose& goal) const {
        return (reached.head<3>() - goal.head<3>()).norm() < tolerance &&
               pose_to_dq(reached).real.angularDistance(pose_to_dq(goal).real) < tolerance;
    }

    // Both modes use exactly the same ScLERP, RMRC, and collision checks.
    bool followEdge(const RRTNode& start, const Pose& end, RRTNode& reached,
                    PlannerDebugStats& stats,
                    std::vector<Eigen::VectorXd>* screw_trajectory = nullptr) {
        const DQ from = pose_to_dq(start.pose), to = pose_to_dq(end);
        Eigen::Matrix<double, n, 1> joints = start.qpos;
        int steps = settings.sclerp_steps_per_edge;
        if (screw_trajectory) {
            // One continuous home-to-goal screw, with time resolution scaled to
            // its length. These integration samples are not sampled RRT nodes.
            const double distance = (end.head<3>() - start.position).norm();
            const double angle = from.real.angularDistance(to.real);
            steps *= static_cast<int>(std::ceil(std::max(
                {1.0, distance / settings.maximum_step, angle / 0.2})));
        }
        const int iterations = settings.rmrc_iterations_per_edge +
            (steps - settings.sclerp_steps_per_edge);
        for (int i = 1; i <= iterations; ++i) {
            const double progress = std::min(static_cast<double>(i) / steps, 1.0);
            const Pose desired = dq_to_pose(ScLERP(from, to, progress));
            const Eigen::Matrix<double, n, 1> previous = joints;
            joints = RMRC(robot, joints, FK(robot, joints), desired, beta);
            if (!joints.allFinite()) { ++stats.non_finite; return false; }
            if (self.hasSelfCollision(joints)) { ++stats.self_collision; return false; }
            if (scene.hasObstacleCollision(joints)) { ++stats.obstacle_collision; return false; }
            if (screw_trajectory) {
                if (!collisionFree(previous, joints)) { ++stats.joint_path_collision; return false; }
                screw_trajectory->push_back(joints);
            }
        }
        const Pose pose = FK(robot, joints);
        if (!atGoal(pose, end)) { ++stats.rmrc_not_converged; return false; }
        if (!screw_trajectory && !collisionFree(start.qpos, joints)) {
            ++stats.joint_path_collision;
            return false;
        }
        reached.position = pose.head<3>();
        reached.pose = pose;
        reached.qpos = joints;
        return true;
    }

    void sampledPlan(const Pose& goal, PlanResult& result) {
        auto& tree = result.tree;
        auto& stats = result.stats;
        const WorkspaceSamplingBounds workspace{robot.workspace_center,
            robot.workspace_inner_radius, robot.workspace_outer_radius};
        std::vector<int> goal_nodes;
        std::vector<bool> goal_attempted;
        const int attempts = settings.samples_per_expansion + (settings.use_goal_bias ? 0 : 1);
        for (int expansion = 0; expansion < settings.max_expansions &&
             (settings.use_rrt_star || goal_nodes.empty()); ++expansion) {
            ++stats.expansions;
            for (int attempt = 0; attempt < attempts; ++attempt) {
                const Eigen::Vector3d sample = sampleExpansionPoint(
                    workspace, goal.head<3>(), obstacles, settings, attempt);
                const int nearest = !settings.use_goal_bias && attempt == 0
                    ? selectGoalExpansionNode(tree, goal.head<3>(), goal_attempted)
                    : findNearestNode(tree, sample);
                if (nearest < 0) continue;
                const Eigen::Vector3d position = steerTowards(
                    tree[nearest].position, sample, settings.maximum_step);
                const double distance = (goal.head<3>() - tree[nearest].position).norm();
                const double length = (position - tree[nearest].position).norm();
                // A goal with only rotation remaining is still a useful edge.
                if (length < 1e-6 && atGoal(tree[nearest].pose, goal)) {
                    goal_nodes.push_back(nearest);
                    break;
                }
                if (length < 1e-6 && distance > tolerance) {
                    ++stats.no_progress;
                    continue;
                }
                Pose end = tree[nearest].pose;
                end.head<3>() = position;
                const auto rotation = rot_diff(tree[nearest].pose, goal,
                    distance > 1e-9 ? std::min(length / distance, 1.0) : 1.0);
                end.tail<4>() << rotation.w(), rotation.x(), rotation.y(), rotation.z();
                RRTNode node;
                if (!followEdge(tree[nearest], end, node, stats)) continue;
                node.parent = nearest;
                node.cost = tree[nearest].cost + (node.position - tree[nearest].position).norm();
                const int index = addRRTNode(tree, std::move(node), settings,
                    [&](const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
                        return collisionFree(a, b);
                    });
                if (atGoal(tree[index].pose, goal)) {
                    goal_nodes.push_back(index);
                    if (!settings.use_rrt_star) break;
                }
                if (!settings.use_goal_bias) break;
            }
        }
        if (goal_nodes.empty()) {
            result.failure_reason = "expansion_limit";
            return;
        }
        const int goal_node = *std::min_element(goal_nodes.begin(), goal_nodes.end(),
            [&](int a, int b) { return tree[a].cost < tree[b].cost; });
        std::vector<int> path;
        for (int i = goal_node; i >= 0; i = tree[i].parent) path.push_back(i);
        std::reverse(path.begin(), path.end());
        result.trajectory = {tree.front().qpos};
        for (size_t i = 1; i < path.size(); ++i) {
            const auto& from = tree[path[i - 1]].qpos;
            const auto& to = tree[path[i]].qpos;
            for (int frame = 1; frame <= settings.animation_frames_per_edge; ++frame) {
                const double t = static_cast<double>(frame) / settings.animation_frames_per_edge;
                result.trajectory.push_back((1 - t) * from + t * to);
            }
        }
        result.success = true;
    }
};

MotionPlanner::MotionPlanner(const std::string& robot_xml, const std::string& scene_xml,
                             const RRTStarSettings& settings)
    : impl_(std::make_unique<Impl>(robot_xml, scene_xml, settings)) {}
MotionPlanner::~MotionPlanner() = default;
const Pose& MotionPlanner::homePose() const { return impl_->robot.home_pose; }

Pose MotionPlanner::sampleGoal() {
    auto& p = *impl_;
    return sampleReachablePose(p.robot.joint_lower_limits, p.robot.joint_upper_limits,
        [&](const Eigen::VectorXd& joints) { return FK(p.robot, joints); },
        [&](const Eigen::VectorXd& joints, const Pose& pose) {
            return isInsideWorkspace(p.robot, pose.head<3>()) &&
                   !isPointInsideObstacle(pose.head<3>(), p.obstacles) &&
                   !p.self.hasSelfCollision(joints) && !p.scene.hasObstacleCollision(joints);
        }, p.settings.obstacle_sample_attempts);
}

PlanResult MotionPlanner::plan(const Pose& goal, PlannerMode mode) {
    auto& p = *impl_;
    PlanResult result;
    RRTNode root;
    root.pose = p.robot.home_pose;
    root.position = root.pose.head<3>();
    root.qpos = p.robot.home_qpos;
    result.tree.push_back(root);
    const auto started = std::chrono::steady_clock::now();
    try {
        if (!goal.allFinite() || std::abs(goal.tail<4>().norm() - 1.0) > 1e-6) {
            result.failure_reason = "invalid_goal_pose";
        } else if (p.self.hasSelfCollision(root.qpos) || p.scene.hasObstacleCollision(root.qpos)) {
            result.failure_reason = "invalid_start";
        } else if (p.atGoal(root.pose, goal)) {
            result.success = true;
            result.trajectory.push_back(root.qpos);
        } else if (mode == PlannerMode::DirectScLERP) {
            result.stats.expansions = 1;
            result.trajectory.push_back(root.qpos);
            RRTNode end;
            result.success = p.followEdge(root, goal, end, result.stats, &result.trajectory);
            if (result.success) {
                end.parent = 0;
                end.cost = (end.position - root.position).norm();
                result.tree.push_back(std::move(end));
            } else {
                result.failure_reason = "direct_edge_rejected";
            }
        } else {
            p.sampledPlan(goal, result);
        }
    } catch (const std::runtime_error& error) {
        result.failure_reason = error.what();
    }
    result.planning_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
    if (result.failure_reason == "invalid_goal_pose") {
        result.position_error = result.rotation_error = std::numeric_limits<double>::infinity();
        return result;
    }
    Pose reached = result.trajectory.empty() ? root.pose : FK(p.robot, result.trajectory.back());
    if (!result.success && mode == PlannerMode::SampledScLERP)
        reached = result.tree[findNearestNode(result.tree, goal.head<3>())].pose;
    result.position_error = (reached.head<3>() - goal.head<3>()).norm();
    result.rotation_error = pose_to_dq(reached).real.angularDistance(pose_to_dq(goal).real);
    return result;
}

PathValidation MotionPlanner::validatePath(const PlanResult& result, const Pose& goal,
                                          int checks_per_segment) {
    if (checks_per_segment <= 0) throw std::invalid_argument("Validation density must be positive.");
    PathValidation validation;
    if (!result.success || result.trajectory.empty()) return validation;
    auto& p = *impl_;
    // Check joint bounds and collisions at a finer density than playback.
    // Sample every playback interval, including the initial configuration.
    const auto check = [&](const Eigen::VectorXd& joints) {
        if (!joints.allFinite()) {
            validation.joint_limits_ok = validation.collision_free = false;
            return;
        }
        for (int joint = 0; joint < n; ++joint) {
            // Limits -pi..pi represent unrestricted continuous hinges in the
            // sampler. Only MuJoCo-limited joints constrain a planned path.
            // Read actual limited flags below through the robot XML metadata.
            if (p.robot.joint_limited[joint] &&
                (joints[joint] < p.robot.joint_lower_limits[joint] - 1e-9 ||
                 joints[joint] > p.robot.joint_upper_limits[joint] + 1e-9))
                validation.joint_limits_ok = false;
        }
        if (p.self.hasSelfCollision(joints) || p.scene.hasObstacleCollision(joints))
            validation.collision_free = false;
    };
    check(result.trajectory.front());
    // 240 samples per original sampled edge = 10 per playback interval.
    // Direct trajectories already contain every RMRC integration step.
    const int density = std::max(1, static_cast<int>(std::ceil(
        static_cast<double>(checks_per_segment) / p.settings.animation_frames_per_edge)));
    for (size_t i = 1; i < result.trajectory.size(); ++i) {
        for (int sample = 1; sample <= density; ++sample) {
            const double t = static_cast<double>(sample) / density;
            check((1 - t) * result.trajectory[i - 1] + t * result.trajectory[i]);
        }
    }
    const Pose reached = p.self.toolPose(result.trajectory.back());
    validation.position_error = (reached.head<3>() - goal.head<3>()).norm();
    validation.rotation_error = pose_to_dq(reached).real.angularDistance(pose_to_dq(goal).real);
    validation.valid = validation.joint_limits_ok && validation.collision_free &&
        validation.position_error < Impl::tolerance && validation.rotation_error < Impl::tolerance;
    return validation;
}

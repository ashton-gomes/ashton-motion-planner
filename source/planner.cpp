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
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>

namespace {

// Joint-space vectors have one entry per planned joint; the count comes from
// the RobotConfig, so any number of hinge joints works.
using JointVector = Eigen::VectorXd;
using SpatialJacobian = Eigen::Matrix<double, 6, Eigen::Dynamic>;

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
    // Planned joints, base to tool, and the end-effector body.
    std::vector<std::string> joint_names;
    std::string tool_body;
    // Every body of the robot; scene geoms on these bodies belong to the robot.
    std::unordered_set<std::string> body_names;
    // qpos values of the joints that are not planned (held still), by name.
    std::map<std::string, std::vector<double>> held_joint_values;

    std::vector<ScrewJoint> joints;
    DQ M_dq; // Tool pose dual quaternion at theta = 0
    JointVector home_qpos;
    Eigen::Matrix<double, 7, 1> home_pose;
    JointVector joint_lower_limits;
    std::vector<bool> joint_limited;
    JointVector joint_upper_limits;

    // Approximate Cartesian sampling workspace, centered at the base joint.
    Eigen::Vector3d workspace_center = Eigen::Vector3d::Zero();
    double workspace_outer_radius = 0.0;
    double workspace_inner_radius = 0.0;

    int dof() const { return static_cast<int>(joints.size()); }
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

SpatialJacobian spatialJacobian(
    const RobotModel& robot,
    const JointVector& theta)
{
    const int n = robot.dof();
    SpatialJacobian J(6, n);

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

Eigen::Matrix<double, Eigen::Dynamic, 6> pseudoInverse(
    const SpatialJacobian& J)
{
    Eigen::Matrix<double, 6, 6> JJt =
        J * J.transpose();

    Eigen::Matrix<double, Eigen::Dynamic, 6> J_pinv =
        J.transpose() *
        (JJt + 1e-6 * Eigen::Matrix<double, 6, 6>::Identity())
            .ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());

    return J_pinv;
}


// B from paper:

Eigen::Matrix<double, Eigen::Dynamic, 7> getB(
    const RobotModel& robot,
    const JointVector& theta,
    const Eigen::Matrix<double, 7, 1>& pose)
{
    // Manipulator spatial Jacobian
    SpatialJacobian Js =
        spatialJacobian(robot, theta);


    // Representation Jacobian
    Eigen::Matrix<double, 6, 7> J2 =
        representationJacobian(pose);


    // Pseudoinverse
    Eigen::Matrix<double, Eigen::Dynamic, 6> Js_pinv =
        pseudoInverse(Js);


    // B = Js+ * J2
    Eigen::Matrix<double, Eigen::Dynamic, 7> B =
        Js_pinv * J2;


    return B;
}

// RMRC:

JointVector RMRC(
    const RobotModel& robot,
    const JointVector& theta_current,
    const Eigen::Matrix<double, 7, 1>& pose_current,
    const Eigen::Matrix<double, 7, 1>& pose_next,
    double beta)
{
    Eigen::Matrix<double, Eigen::Dynamic, 7> B =
        getB(
            robot,
            theta_current,
            pose_current
        );


    Eigen::Matrix<double, 7, 1> aligned_next = pose_next;
    if (pose_current.tail<4>().dot(aligned_next.tail<4>()) < 0.0)
        aligned_next.tail<4>() *= -1.0;
    Eigen::Matrix<double, 7, 1> pose_change = aligned_next - pose_current;


    JointVector theta_next =
        theta_current
        + beta * B * pose_change;


    return theta_next;
}


// Loading the robot from its MuJoCo model:

using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPtr = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

ModelPtr loadModel(const std::string& path, const std::string& what) {
    char error[1000]{};
    ModelPtr model(mj_loadXML(path.c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
    if (!model) throw std::runtime_error(what + " load error (" + path + "): " + error);
    return model;
}

DataPtr makeData(const mjModel* model, const std::string& what) {
    DataPtr data(mj_makeData(model), mj_deleteData);
    if (!data) throw std::runtime_error("Could not create " + what + " data.");
    return data;
}

std::string nameOf(const mjModel* m, mjtObj type, int id) {
    const char* name = mj_id2name(m, type, id);
    return name ? name : "";
}

// Number of qpos values one joint uses.
int qposWidth(int joint_type) {
    if (joint_type == mjJNT_FREE) return 7;
    if (joint_type == mjJNT_BALL) return 4;
    return 1;
}

// True if `body` is `ancestor` or sits below it in the body tree.
bool isAncestorOrSelf(const mjModel* m, int ancestor, int body) {
    for (; body > 0; body = m->body_parentid[body])
        if (body == ancestor) return true;
    return ancestor == 0;
}

// World pose [x, y, z, qw, qx, qy, qz] of one body after mj_kinematics.
Pose bodyPose(const mjData* d, int body) {
    Pose pose;
    pose << Eigen::Map<const Eigen::Vector3d>(&d->xpos[3 * body]),
            d->xquat[4 * body], d->xquat[4 * body + 1],
            d->xquat[4 * body + 2], d->xquat[4 * body + 3];
    return pose;
}

// Where the planned joints sit inside one loaded MuJoCo model (the robot on
// its own, or a scene that includes it). All other joints keep held_qpos.
struct ChainBinding {
    std::vector<int> qpos_address; // qpos index of each planned joint
    Eigen::VectorXd held_qpos;     // full qpos used for the joints not planned

    void write(const JointVector& joints, mjData* data) const {
        if (joints.size() != static_cast<int>(qpos_address.size()))
            throw std::invalid_argument("Joint vector size does not match the planned joints.");
        std::copy(held_qpos.data(), held_qpos.data() + held_qpos.size(), data->qpos);
        for (size_t i = 0; i < qpos_address.size(); ++i)
            data->qpos[qpos_address[i]] = joints[static_cast<int>(i)];
    }

    Eigen::VectorXd fullQpos(const JointVector& joints) const {
        if (joints.size() != static_cast<int>(qpos_address.size()))
            throw std::invalid_argument("Joint vector size does not match the planned joints.");
        Eigen::VectorXd qpos = held_qpos;
        for (size_t i = 0; i < qpos_address.size(); ++i)
            qpos[qpos_address[i]] = joints[static_cast<int>(i)];
        return qpos;
    }

    JointVector extract(const Eigen::VectorXd& qpos) const {
        JointVector joints(static_cast<int>(qpos_address.size()));
        for (size_t i = 0; i < qpos_address.size(); ++i)
            joints[static_cast<int>(i)] = qpos[qpos_address[i]];
        return joints;
    }
};

// Joints are matched by name, so the same robot works on its own model and in
// any scene that includes that model.
ChainBinding bindChain(const mjModel* m, const RobotModel& robot,
                       const std::string& model_label) {
    ChainBinding binding;
    binding.held_qpos = Eigen::Map<const Eigen::VectorXd>(m->qpos0, m->nq);
    for (int j = 0; j < m->njnt; ++j) {
        const auto held = robot.held_joint_values.find(nameOf(m, mjOBJ_JOINT, j));
        if (held != robot.held_joint_values.end() &&
            static_cast<int>(held->second.size()) == qposWidth(m->jnt_type[j])) {
            std::copy(held->second.begin(), held->second.end(),
                      binding.held_qpos.data() + m->jnt_qposadr[j]);
        }
    }
    for (const std::string& name : robot.joint_names) {
        const int joint = mj_name2id(m, mjOBJ_JOINT, name.c_str());
        if (joint < 0)
            throw std::runtime_error(model_label + " has no joint named '" + name + "'.");
        binding.qpos_address.push_back(m->jnt_qposadr[joint]);
    }
    return binding;
}

RobotModel loadRobot(const RobotConfig& config) {
    ModelPtr model = loadModel(config.model_path, "Robot model");
    const mjModel* m = model.get();
    DataPtr data = makeData(m, "robot kinematics");
    mjData* d = data.get();

    RobotModel robot;
    robot.tool_body = config.tool_body;
    const int tool_id = mj_name2id(m, mjOBJ_BODY, config.tool_body.c_str());
    if (tool_id < 0)
        throw std::runtime_error("Robot model has no tool body named '" + config.tool_body + "'.");

    // 1. Planned joints: the configured names, or every joint in the model.
    robot.joint_names = config.joint_names;
    if (robot.joint_names.empty())
        for (int j = 0; j < m->njnt; ++j) robot.joint_names.push_back(nameOf(m, mjOBJ_JOINT, j));
    if (robot.joint_names.empty())
        throw std::runtime_error("Robot model has no joints to plan with.");
    std::vector<int> ids;
    for (const std::string& name : robot.joint_names) {
        const int joint = mj_name2id(m, mjOBJ_JOINT, name.c_str());
        if (joint < 0)
            throw std::runtime_error("Robot model has no joint named '" + name + "'.");
        if (m->jnt_type[joint] != mjJNT_HINGE)
            throw std::runtime_error("Joint '" + name + "' is not a hinge; only revolute joints can be planned.");
        if (!ids.empty() && !isAncestorOrSelf(m, m->jnt_bodyid[ids.back()], m->jnt_bodyid[joint]))
            throw std::runtime_error("Joint '" + name + "' does not follow '" +
                robot.joint_names[ids.size() - 1] + "' in one chain from base to tool.");
        ids.push_back(joint);
    }
    if (!isAncestorOrSelf(m, m->jnt_bodyid[ids.back()], tool_id))
        throw std::runtime_error("Tool body '" + config.tool_body +
                                 "' does not come after the last planned joint.");
    const int n = static_cast<int>(ids.size());

    // 2. Starting value of every joint: the keyframe if present, else the
    // model defaults. Joints that are not planned stay at these values.
    Eigen::VectorXd start_qpos = Eigen::Map<const Eigen::VectorXd>(m->qpos0, m->nq);
    const int key = config.home_keyframe.empty()
        ? -1 : mj_name2id(m, mjOBJ_KEY, config.home_keyframe.c_str());
    if (key >= 0)
        start_qpos = Eigen::Map<const Eigen::VectorXd>(&m->key_qpos[key * m->nq], m->nq);
    for (int j = 0; j < m->njnt; ++j) {
        const std::string name = nameOf(m, mjOBJ_JOINT, j);
        if (name.empty()) continue;
        const double* values = start_qpos.data() + m->jnt_qposadr[j];
        robot.held_joint_values[name].assign(values, values + qposWidth(m->jnt_type[j]));
    }
    for (int body = 1; body < m->nbody; ++body)
        robot.body_names.insert(nameOf(m, mjOBJ_BODY, body));
    const ChainBinding binding = bindChain(m, robot, "Robot model");

    // 3. Read the screw axis and anchor point of each planned joint with the
    // planned joints at zero. The tool pose here is M in the PoE formula.
    binding.write(JointVector::Zero(n), d);
    mj_kinematics(m, d); // Computes Cartesian positions without physics
    robot.joints.resize(n);
    robot.joint_limited.resize(n);
    robot.joint_lower_limits.resize(n);
    robot.joint_upper_limits.resize(n);
    const double pi = std::acos(-1.0);
    for (int i = 0; i < n; ++i) {
        const int j = ids[i];
        robot.joints[i].omega = Eigen::Map<Eigen::Vector3d>(&d->xaxis[3 * j]).normalized();
        robot.joints[i].q     = Eigen::Map<Eigen::Vector3d>(&d->xanchor[3 * j]);
        // One full turn suffices for continuous hinge joints.
        robot.joint_limited[i] = m->jnt_limited[j];
        robot.joint_lower_limits[i] = m->jnt_limited[j] ? m->jnt_range[2 * j] : -pi;
        robot.joint_upper_limits[i] = m->jnt_limited[j] ? m->jnt_range[2 * j + 1] : pi;
    }
    const Pose zero_pose = bodyPose(d, tool_id);
    robot.M_dq = pose_to_dq(zero_pose);

    // Approximate the arm reach from distances between joint anchors.  This is
    // a cheap sampling bound, not a guarantee that every sampled pose has IK.
    robot.workspace_center = robot.joints.front().q;
    double total_link_length = 0.0;
    for (int i = 1; i < n; ++i) {
        total_link_length +=
            (robot.joints[i].q - robot.joints[i - 1].q).norm();
    }
    total_link_length += (zero_pose.head<3>() - robot.joints.back().q).norm();

    // Avoid full-extension singularities and reject points near the base.
    robot.workspace_outer_radius = 0.95 * total_link_length;
    robot.workspace_inner_radius = n > 1
        ? 0.5 * (robot.joints[1].q - robot.joints[0].q).norm() : 0.0;

    // 4. Home configuration: from the config, else from the keyframe.
    if (!config.home_joints.empty()) {
        if (static_cast<int>(config.home_joints.size()) != n)
            throw std::runtime_error("home_joints has " + std::to_string(config.home_joints.size()) +
                                     " values but " + std::to_string(n) + " joints are planned.");
        robot.home_qpos = Eigen::Map<const JointVector>(config.home_joints.data(), n);
    } else if (key >= 0) {
        robot.home_qpos = binding.extract(start_qpos);
    } else {
        throw std::runtime_error("Robot '" + config.name + "' needs home_joints or a '" +
                                 config.home_keyframe + "' keyframe.");
    }
    binding.write(robot.home_qpos, d);
    mj_kinematics(m, d);
    robot.home_pose = bodyPose(d, tool_id);
    return robot;
}

bool isInsideWorkspace(const RobotModel& robot,
                       const Eigen::Vector3d& point) {
    const double radius = (point - robot.workspace_center).norm();
    return radius >= robot.workspace_inner_radius &&
           radius <= robot.workspace_outer_radius;
}

// Uses the robot model on its own for collision queries, so any contact is a
// self-collision.
class SelfCollisionChecker {
public:
    SelfCollisionChecker(const std::string& xml_path, const RobotModel& robot)
        : model_(loadModel(xml_path, "Collision model")),
          data_(makeData(model_.get(), "collision-check")),
          binding_(bindChain(model_.get(), robot, "Collision model")),
          tool_(mj_name2id(model_.get(), mjOBJ_BODY, robot.tool_body.c_str())) {
        if (tool_ < 0)
            throw std::runtime_error("Collision model has no tool body named '" + robot.tool_body + "'.");
        // Group each body by the last planned joint above it. Bodies in the
        // same group never move relative to each other (for example a G1's
        // legs and torso while one arm moves), so their contacts are ignored.
        std::vector<int> planned_joint_body;
        for (const std::string& name : robot.joint_names)
            planned_joint_body.push_back(
                model_->jnt_bodyid[mj_name2id(model_.get(), mjOBJ_JOINT, name.c_str())]);
        rigid_group_.assign(model_->nbody, -1);
        for (int body = 1; body < model_->nbody; ++body) {
            rigid_group_[body] = rigid_group_[model_->body_parentid[body]];
            for (size_t i = 0; i < planned_joint_body.size(); ++i)
                if (planned_joint_body[i] == body) rigid_group_[body] = static_cast<int>(i);
        }
        skipIgnoredPairs();
    }

    Pose toolPose(const JointVector& joints) {
        binding_.write(joints, data_.get());
        mj_kinematics(model_.get(), data_.get());
        return bodyPose(data_.get(), tool_);
    }

    bool hasSelfCollision(const JointVector& joints) {
        binding_.write(joints, data_.get());
        // Only link positions and contacts are needed, not the full physics step.
        mj_kinematics(model_.get(), data_.get());
        mj_collision(model_.get(), data_.get());

        for (int i = 0; i < data_->ncon; ++i) {
            const mjContact& contact = data_->contact[i];
            const int body1 = model_->geom_bodyid[contact.geom1];
            const int body2 = model_->geom_bodyid[contact.geom2];

            // Geoms on the world body (such as a floor) are not robot links.
            if (body1 == 0 || body2 == 0) continue;
            // Links with no planned joint between them never move apart.
            if (rigid_group_[body1] == rigid_group_[body2]) continue;
            // Collision meshes of links joined directly together overlap at
            // their joint.  Only non-neighboring link contacts are rejected.
            const bool directly_connected =
                model_->body_parentid[body1] == body2 ||
                model_->body_parentid[body2] == body1;
            if (!directly_connected) {
                return true;
            }
        }
        return false;
    }

private:
    // Speed-up: stop MuJoCo from computing contacts that hasSelfCollision
    // ignores anyway (world geoms, and links in the same rigid group, such as
    // a G1's legs against its pelvis). Each rigid group gets one collision
    // bit and does not collide with its own bit. Only done when every
    // collision geom uses MuJoCo's default masks (contype = conaffinity = 1),
    // so pairs the model turned off on purpose stay off.
    void skipIgnoredPairs() {
        // Groups run from -1 (before the first planned joint) to dof - 1.
        const int groups = *std::max_element(rigid_group_.begin(), rigid_group_.end()) + 2;
        if (groups > 30) return;
        for (int geom = 0; geom < model_->ngeom; ++geom) {
            const int type = model_->geom_contype[geom], affinity = model_->geom_conaffinity[geom];
            if ((type || affinity) && !(type == 1 && affinity == 1)) return;
        }
        const int all = (1 << groups) - 1;
        for (int geom = 0; geom < model_->ngeom; ++geom) {
            if (!model_->geom_contype[geom] && !model_->geom_conaffinity[geom]) continue;
            const int body = model_->geom_bodyid[geom];
            if (body == 0) {
                model_->geom_contype[geom] = model_->geom_conaffinity[geom] = 0;
                continue;
            }
            const int bit = 1 << (rigid_group_[body] + 1);
            model_->geom_contype[geom] = bit;
            model_->geom_conaffinity[geom] = all & ~bit;
        }
    }

    ModelPtr model_;
    DataPtr data_;
    ChainBinding binding_;
    int tool_ = -1;
    std::vector<int> rigid_group_;
};

// Checks the robot's collision geoms against non-plane physical geoms from
// scene.xml.  The obstacle margin is enlarged for a conservative path.
class ObstacleCollisionChecker {
public:
    ObstacleCollisionChecker(const std::string& scene_path, const RobotModel& robot,
                             double clearance)
        : model_(loadModel(scene_path, "Scene collision")),
          data_(nullptr, mj_deleteData) {
        applyCylinderPlacements(model_.get());
        binding_ = bindChain(model_.get(), robot, "Scene " + scene_path);
        // A geom belongs to the robot if its body is one of the robot's bodies.
        robot_geom_.resize(model_->ngeom);
        obstacle_geom_.resize(model_->ngeom);
        for (int geom = 0; geom < model_->ngeom; ++geom) {
            const std::string body = nameOf(model_.get(), mjOBJ_BODY, model_->geom_bodyid[geom]);
            robot_geom_[geom] = robot.body_names.count(body) > 0;
        }
        for (int geom = 0; geom < model_->ngeom; ++geom) {
            obstacle_geom_[geom] = !robot_geom_[geom] &&
                                   model_->geom_type[geom] != mjGEOM_PLANE &&
                                   model_->geom_contype[geom] != 0 &&
                                   model_->geom_conaffinity[geom] != 0;
            if (obstacle_geom_[geom]) {
                model_->geom_margin[geom] += clearance;
            }
        }
        onlyRobotObstaclePairs();
        data_ = makeData(model_.get(), "scene collision");
    }

    bool hasObstacleCollision(const JointVector& joints) {
        binding_.write(joints, data_.get());
        // Only link positions and contacts are needed, not the full physics step.
        mj_kinematics(model_.get(), data_.get());
        mj_collision(model_.get(), data_.get());

        for (int i = 0; i < data_->ncon; ++i) {
            const mjContact& contact = data_->contact[i];
            if ((isRobotCollisionGeom(contact.geom1) && isObstacleGeom(contact.geom2)) ||
                (isRobotCollisionGeom(contact.geom2) && isObstacleGeom(contact.geom1))) {
                return true;
            }
        }
        return false;
    }

    const ChainBinding& binding() const { return binding_; }

private:
    bool isRobotCollisionGeom(int geom) const {
        return robot_geom_[geom];
    }

    bool isObstacleGeom(int geom) const {
        return obstacle_geom_[geom];
    }

    // Speed-up: only robot-vs-obstacle contacts are used here, so stop MuJoCo
    // from computing robot-vs-robot, obstacle-vs-obstacle, and floor contacts.
    // Robot geoms only accept obstacles, and obstacles accept nothing. Only
    // done when every collision geom uses MuJoCo's default masks (1 and 1).
    void onlyRobotObstaclePairs() {
        for (int geom = 0; geom < model_->ngeom; ++geom) {
            const int type = model_->geom_contype[geom], affinity = model_->geom_conaffinity[geom];
            if ((type || affinity) && !(type == 1 && affinity == 1)) return;
        }
        for (int geom = 0; geom < model_->ngeom; ++geom) {
            const bool physical = model_->geom_contype[geom] || model_->geom_conaffinity[geom];
            model_->geom_contype[geom] = obstacle_geom_[geom] ? 1 : 0;
            model_->geom_conaffinity[geom] = robot_geom_[geom] && physical ? 1 : 0;
        }
    }

    ModelPtr model_;
    DataPtr data_;
    ChainBinding binding_;
    std::vector<bool> robot_geom_;
    std::vector<bool> obstacle_geom_; // decided once, before the masks change
};

// Check the simple joint-space motion used to replay and rewire tree edges.
bool isJointPathCollisionFree(
    const JointVector& from,
    const JointVector& to,
    int checks,
    SelfCollisionChecker& self_collision_checker,
    ObstacleCollisionChecker& obstacle_collision_checker)
{
    for (int i = 1; i <= checks; ++i) {
        const double t = static_cast<double>(i) / checks;
        const JointVector qpos = (1.0 - t) * from + t * to;
        if (!qpos.allFinite() ||
            self_collision_checker.hasSelfCollision(qpos) ||
            obstacle_collision_checker.hasObstacleCollision(qpos)) {
            return false;
        }
    }
    return true;
}


// Forward Kinematics via Dual Quaternion PoE
Eigen::Matrix<double, 7, 1> FK(const RobotModel& robot, const JointVector& theta) {
    DQ T_cumulative;
    // Default DQ is the identity transform

    for (int i = 0; i < robot.dof(); ++i) {
        DQ exp_i = exp_twist(robot.joints[i], theta[i]);
        T_cumulative = dq_mult(T_cumulative, exp_i);
    }

    DQ T_final = dq_mult(T_cumulative, robot.M_dq);
    return dq_to_pose(T_final);
}



} // namespace

struct MotionPlanner::Impl {
    RobotConfig config;
    RobotModel robot;
    RRTStarSettings settings;
    SelfCollisionChecker self;
    ObstacleCollisionChecker scene;
    std::vector<SceneObstacle> obstacles;
    std::optional<Pose> fixed_goal;
    static constexpr double tolerance = 1e-3;
    static constexpr double beta = 0.1;

    Impl(const RobotConfig& robot_config, const std::string& scene_xml,
         const RRTStarSettings& options)
        : config(robot_config), robot(loadRobot(robot_config)), settings(options),
          self(robot_config.model_path, robot),
          scene(scene_xml, robot, options.obstacle_clearance),
          obstacles(loadSceneObstacles(scene_xml, options.obstacle_clearance, robot.body_names)) {
        validateRRTSettings(settings);
        loadSceneTask(scene_xml);
    }

    // Optional fixed task from the scene: "start" replaces the robot's home
    // configuration and "goal" fixes the target. Both must be collision-free.
    void loadSceneTask(const std::string& scene_xml) {
        const auto joints = [&](const char* name) -> std::optional<JointVector> {
            const auto qpos = loadSceneKeyframe(scene_xml, name);
            if (!qpos) return std::nullopt;
            if (qpos->size() != scene.binding().held_qpos.size()) throw std::runtime_error(
                std::string("Scene keyframe '") + name + "' has the wrong number of values.");
            const JointVector q = scene.binding().extract(*qpos);
            if (self.hasSelfCollision(q) || scene.hasObstacleCollision(q)) throw std::runtime_error(
                std::string("Scene keyframe '") + name + "' is in collision: " + scene_xml);
            return q;
        };
        if (const auto start = joints("start")) {
            robot.home_qpos = *start;
            robot.home_pose = FK(robot, *start);
        }
        if (const auto goal = joints("goal")) fixed_goal = FK(robot, *goal);
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
        JointVector joints = start.qpos;
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
            const JointVector previous = joints;
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

MotionPlanner::MotionPlanner(const RobotConfig& robot, const std::string& scene_xml,
                             const RRTStarSettings& settings)
    : impl_(std::make_unique<Impl>(robot, scene_xml, settings)) {}
MotionPlanner::~MotionPlanner() = default;
const RobotConfig& MotionPlanner::robot() const { return impl_->config; }
int MotionPlanner::dof() const { return impl_->robot.dof(); }

Eigen::VectorXd MotionPlanner::toSceneQpos(const Eigen::VectorXd& joints) const {
    return impl_->scene.binding().fullQpos(joints);
}

std::vector<Eigen::VectorXd> MotionPlanner::toSceneTrajectory(
    const std::vector<Eigen::VectorXd>& trajectory) const {
    std::vector<Eigen::VectorXd> scene_trajectory;
    scene_trajectory.reserve(trajectory.size());
    for (const auto& joints : trajectory) scene_trajectory.push_back(toSceneQpos(joints));
    return scene_trajectory;
}
const Pose& MotionPlanner::homePose() const { return impl_->robot.home_pose; }
const std::optional<Pose>& MotionPlanner::sceneGoal() const { return impl_->fixed_goal; }

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
        for (int joint = 0; joint < p.robot.dof(); ++joint) {
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

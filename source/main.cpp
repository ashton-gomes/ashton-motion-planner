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
#include "viewer.hpp"
#ifndef ROBOT_XML_PATH
#define ROBOT_XML_PATH "/home/ash_gomes/gplanner/mujoco_menagerie/kinova_gen3/gen3_obstacle.xml"
#endif



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
        std::cerr << "XML Load Error: " << error << std::endl;
        std::exit(1);
    }
    mjData* d = mj_makeData(m);

    // 1. Set all joints to zero and evaluate static geometry ONLY
    mju_zero(d->qpos, m->nq);
    mj_kinematics(m, d); // Computes Cartesian positions without physics

    RobotModel robot;
    robot.joints.resize(m->njnt);

    if (m->nq != n || m->njnt != n) {
        std::cerr << "Expected a " << n << "-DoF robot, but model has "
                  << m->njnt << " joints and " << m->nq << " qpos values.\n";
        mj_deleteData(d);
        mj_deleteModel(m);
        std::exit(1);
    }

    // 2. Read the screw axis and anchor point for each joint
    for (int i = 0; i < m->njnt; ++i) {
        robot.joints[i].omega = Eigen::Map<Eigen::Vector3d>(&d->xaxis[3 * i]).normalized();
        robot.joints[i].q     = Eigen::Map<Eigen::Vector3d>(&d->xanchor[3 * i]);
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
        std::cerr << "Model does not define a 'home' keyframe.\n";
        mj_deleteData(d);
        mj_deleteModel(m);
        std::exit(1);
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
    // Default DQ is the identity transform.

    for (int i = 0; i < n; ++i) {
        DQ exp_i = exp_twist(robot.joints[i], theta[i]);
        T_cumulative = dq_mult(T_cumulative, exp_i);
    }

    DQ T_final = dq_mult(T_cumulative, robot.M_dq);
    return dq_to_pose(T_final);
}

    
int main() {
    RobotModel robot = extractURDF(ROBOT_XML_PATH);
    SelfCollisionChecker collision_checker(ROBOT_XML_PATH);

    RRTStarSettings settings;
        const auto obstacles = loadSceneObstacles(
        SCENE_XML_PATH, settings.obstacle_clearance);
    // if (isPointInsideObstacle(q.head<3>(), obstacles)) {
    //     std::cerr << "Goal position is inside an inflated scene obstacle.\n";
    //     return 1;
    // }

    // The start pose and configuration come from the model's "home" keyframe.

    // defined as (x, y, z, rotation quaternion) (7 value vector)
    Eigen::Matrix<double, 7, 1> p = robot.home_pose, q;

    // q << robot.workspace_center.x() - 0.3, robot.workspace_center.y() - 0.2, robot.workspace_center.z() + 0.3,
    // p[3] - 0,  p[4] - 0, p[5] - 1, p[6] - 0;
  
    // Random start pose below that is valid considering the obstacles:

    Eigen::Vector3d q_pos = sampleWorkspacePointAvoidingObstacles(
        robot.workspace_center, 
        robot.workspace_inner_radius, 
        robot.workspace_outer_radius,
        obstacles,
        settings.obstacle_sample_attempts);

    Eigen::Quaterniond q_rot = sampleQuaternion(); 

    q << q_pos, q_rot.w(), q_rot.x(), q_rot.y(), q_rot.z(); 


    const double q_radius = (q.head<3>() - robot.workspace_center).norm();
    if (!isInsideWorkspace(robot, q.head<3>())) {
        std::cerr << "Position outside of robot workspace: target radius "
                  << q_radius << " m, allowed range ["
                  << robot.workspace_inner_radius << ", "
                  << robot.workspace_outer_radius << "] m.\n";
        return 1;
    }

    const DQ dq_q = pose_to_dq(q);
    // RRTStarSettings settings;
    ObstacleCollisionChecker obstacle_collision_checker(
        SCENE_XML_PATH, settings.obstacle_clearance);

    WorkspaceSamplingBounds workspace{
        robot.workspace_center,
        robot.workspace_inner_radius,
        robot.workspace_outer_radius,
    };

    // One tree, rooted at the robot's home pose.
    RRTNode root;
    root.position = p.head<3>();
    root.pose = p;
    root.qpos = robot.home_qpos;
    std::vector<RRTNode> tree{root};

    if (collision_checker.hasSelfCollision(root.qpos)) {
        std::cerr << "The home configuration is in self-collision.\n";
        return 1;
    }
    if (obstacle_collision_checker.hasObstacleCollision(root.qpos)) {
        std::cerr << "The home configuration collides with a scene obstacle.\n";
        return 1;
    }

    const double tolerance = 1e-3;
    const double beta = 0.1;
    std::vector<int> goal_nodes;

    // Grow for the whole budget so RRT* can improve its first solution.
    for (int expansion = 0; expansion < settings.max_expansions; ++expansion) {
        const auto samples = sampleExpansionPoints(
            workspace, q.head<3>(), obstacles, settings);

        for (const Eigen::Vector3d& sample : samples) {
            const int nearest = findNearestNode(tree, sample);
            const Eigen::Vector3d new_position = steerTowards(
                tree[nearest].position, sample, settings.maximum_step);
            if ((new_position - tree[nearest].position).squaredNorm() < 1e-12) {
                continue;
            }

            Eigen::Matrix<double, 7, 1> edge_end = tree[nearest].pose;
            edge_end.head<3>() = new_position;

            const double distance_to_goal =
                (q.head<3>() - tree[nearest].position).norm();
            const double edge_length =
                (new_position - tree[nearest].position).norm();
            const double rotation_fraction = distance_to_goal > 1e-9
                ? std::min(edge_length / distance_to_goal, 1.0)
                : 1.0;
            const Eigen::Quaterniond intermediate_rotation =
                rot_diff(tree[nearest].pose, q, rotation_fraction);
            edge_end.tail<4>() << intermediate_rotation.w(),
                intermediate_rotation.x(), intermediate_rotation.y(),
                intermediate_rotation.z();

            const DQ edge_start_dq = pose_to_dq(tree[nearest].pose);
            const DQ edge_end_dq = pose_to_dq(edge_end);
            Eigen::Matrix<double, n, 1> edge_qpos = tree[nearest].qpos;
            bool edge_is_valid = true;

            // ScLERP gives RMRC a short smooth Cartesian edge to follow.
            for (int i = 1; i <= settings.rmrc_iterations_per_edge; ++i) {
                const double progress = std::min(
                    static_cast<double>(i) / settings.sclerp_steps_per_edge, 1.0);
                const auto desired_pose = dq_to_pose(
                    ScLERP(edge_start_dq, edge_end_dq, progress));
                edge_qpos = RMRC(robot, edge_qpos, FK(robot, edge_qpos),
                                 desired_pose, beta);
                if (!edge_qpos.allFinite() ||
                    collision_checker.hasSelfCollision(edge_qpos) ||
                    obstacle_collision_checker.hasObstacleCollision(edge_qpos)) {
                    edge_is_valid = false;
                    break;
                }
            }

            if (!edge_is_valid) {
                continue;
            }

            const auto reached_pose = FK(robot, edge_qpos);
            const double position_error =
                (edge_end.head<3>() - reached_pose.head<3>()).norm();
            const double rotation_error = pose_to_dq(reached_pose).real.angularDistance(
                edge_end_dq.real);
            if (position_error >= tolerance || rotation_error >= tolerance) {
                continue;
            }

            // Keep configurations fixed during rewiring.  This check makes
            // the simple joint-space replay safe for every selected edge.
            if (!isJointPathCollisionFree(
                    tree[nearest].qpos, edge_qpos,
                    settings.animation_frames_per_edge,
                    collision_checker, obstacle_collision_checker)) {
                continue;
            }

            RRTNode new_node;
            new_node.position = reached_pose.head<3>();
            new_node.pose = reached_pose;
            new_node.qpos = edge_qpos;
            new_node.parent = nearest;
            new_node.cost = tree[nearest].cost +
                (new_node.position - tree[nearest].position).norm();

            // RRT*: select the cheapest nearby collision-free parent.
            const auto nearby = findNearbyNodes(
                tree, new_node.position, settings.rewire_radius);
            for (const int parent : nearby) {
                const double candidate_cost = tree[parent].cost +
                    (new_node.position - tree[parent].position).norm();
                if (candidate_cost >= new_node.cost) {
                    continue;
                }
                if (isJointPathCollisionFree(
                        tree[parent].qpos, new_node.qpos,
                        settings.animation_frames_per_edge,
                        collision_checker, obstacle_collision_checker)) {
                    new_node.parent = parent;
                    new_node.cost = candidate_cost;
                }
            }

            tree.push_back(std::move(new_node));
            const int new_index = static_cast<int>(tree.size()) - 1;

            // RRT*: redirect nearby nodes through this node when cheaper.
            // rewireNode also refreshes the costs of their descendants.
            for (const int neighbor : nearby) {
                if (neighbor == tree[new_index].parent || neighbor == 0) {
                    continue;
                }
                const double rewired_cost = tree[new_index].cost +
                    (tree[neighbor].position - tree[new_index].position).norm();
                if (rewired_cost >= tree[neighbor].cost) {
                    continue;
                }
                if (isJointPathCollisionFree(
                        tree[new_index].qpos, tree[neighbor].qpos,
                        settings.animation_frames_per_edge,
                        collision_checker, obstacle_collision_checker)) {
                    rewireNode(tree, neighbor, new_index);
                }
            }

            const double goal_position_error =
                (q.head<3>() - reached_pose.head<3>()).norm();
            const double goal_rotation_error =
                pose_to_dq(reached_pose).real.angularDistance(dq_q.real);
            if (goal_position_error < tolerance &&
                goal_rotation_error < tolerance) {
                goal_nodes.push_back(new_index);
                if (goal_nodes.size() == 1) {
                    std::cout << "RRT* found the goal with " << tree.size()
                              << " nodes; continuing to improve it.\n";
                }
            }
        }
    }

    if (goal_nodes.empty()) {
        std::cerr << "RRT* did not reach the goal after "
                  << settings.max_expansions << " expansions.\n";
        return 1;
    }

    // Several samples can reach the goal; keep the cheapest final one.
    int goal_node = goal_nodes.front();
    for (const int node : goal_nodes) {
        if (tree[node].cost < tree[goal_node].cost) {
            goal_node = node;
        }
    }
    std::cout << "Final RRT* path cost: " << tree[goal_node].cost << " m.\n";

    // Follow parents backward, then reverse into start-to-goal order.
    std::vector<int> path;
    for (int node = goal_node; node >= 0; node = tree[node].parent) {
        path.push_back(node);
    }
    std::reverse(path.begin(), path.end());

    std::vector<Eigen::VectorXd> trajectory{tree.front().qpos};
    for (size_t i = 1; i < path.size(); ++i) {
        const auto& from = tree[path[i - 1]].qpos;
        const auto& to = tree[path[i]].qpos;
        // Use a few evenly spaced frames per edge instead of replaying all
        // RMRC settling iterations, which otherwise looks like a pause.
        for (int frame = 1; frame <= settings.animation_frames_per_edge; ++frame) {
            const double t = static_cast<double>(frame) /
                             settings.animation_frames_per_edge;
            trajectory.push_back((1.0 - t) * from + t * to);
        }
    }

    return MujocoViewer::play(SCENE_XML_PATH, trajectory, p, q);
}

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <chrono>
#include <thread>
#include <algorithm>
#include <memory>
#include <limits>
#include <string>
#include <Eigen/Dense>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

// ----------------------------------------------------------------
// constants and parameters:
// ----------------------------------------------------------------

// from URDF:
static const int    N_JOINTS = 7;
static const char*  XML_PATH = "/home/ash_gomes/gplanner/mujoco_menagerie/kinova_gen3/gen3_obstacle.xml";
static const char*  EE_SITE  = "pinch_site";

static const double Q_MID[N_JOINTS] = {0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57};  // comfort home position pose
static const double Q_LIM_LOW[N_JOINTS]  = {-6.28, -2.24, -6.28, -2.57, -6.28, -2.09, -6.28}; // minimum joint limits
static const double Q_LIM_HIGH[N_JOINTS] = { 6.28,  2.24,  6.28,  2.57,  6.28,  2.09,  6.28}; // maximum joint limits

// workspace limits (x, y, z):
static const double WS_X_MIN = -0.30;
static const double WS_X_MAX =  0.70;

static const double WS_Y_MIN = -0.55;
static const double WS_Y_MAX =  0.55;

static const double WS_Z_MIN =  0.30;
static const double WS_Z_MAX =  1.10;

// planner parameters:
static const int    PLANNER_MAX_ITERS       = 6000; // maximum number of graph expansion attempts
static const int    CANDIDATES_PER_EXPANSION = 5;    // number of highest scoring voxels tried from each node
static const double GOAL_POSITION_TOL       = 0.035; // maximum positional error for reaching the goal
static const double GOAL_ANGLE_TOL          = 0.10;  // maximum angular error for reaching the goal
static const double STEP_POS                = 0.03;  // maximum translation between collision checks
static const double STEP_ANG                = 0.08;  // maximum rotation between collision checks
static const double MAX_PARENT_JOINT_STEP   = 0.45;  // reject a candidate if its final IK solution jumps too far from its parent
static const double GOAL_SCORE_WEIGHT       = 1.0;   // weight given to distance toward the goal
static const double DIRECTION_SCORE_WEIGHT  = 0.35;  // weight given to moving in the general goal direction
static const double DISTANCE_SCORE_WEIGHT   = 0.15;  // small preference for nearby candidate voxels
static const double GOAL_BIAS               = 0.10;  // chance of trying the goal position directly

// voxel and obstacle parameters:
static const double CANDIDATE_VOXEL_RES = 0.05; // voxel grid spacing size
static const double OBSTACLE_SAFETY_MARGIN = 0.03; // obstacle size increase

// ----------------------------------------------------------------
// globals
// ----------------------------------------------------------------

mjModel* m      = nullptr; // static model
mjData*  d      = nullptr; // dynamic state

// mujoco rendering and mouse tracking
mjvCamera   cam;
mjvOption   opt;
mjvScene    scn;
mjrContext  con;
GLFWwindow* window = nullptr;

bool   button_left = false, button_middle = false, button_right = false;
double lastx = 0, lasty = 0;

// random number generator
std::mt19937 rng(1337); // keeps sequence generation the same
std::uniform_real_distribution<double> uni01(0.0, 1.0); // between 0 and 1

// ----------------------------------------------------------------
// obstacle information
// ----------------------------------------------------------------

struct ObstacleInfo {
    Eigen::Vector3d center;
    mjtGeom type;
    Eigen::Vector3d halfsize; // dimensions
};

struct CandidateSampler {
    double res; // voxel resolution
    Eigen::Vector3d ws_min, ws_max;
    std::vector<ObstacleInfo> obstacles;
    std::vector<Eigen::Vector3d> candidates; // valid voxel centers in the workspace

    CandidateSampler(double resolution, const Eigen::Vector3d& lo, const Eigen::Vector3d& hi)
        : res(resolution), ws_min(lo), ws_max(hi) {}
        // takes resolution and boundary and assigns to internal member variables

    // goes through all the obstacles
    void populateFromMuJoCo(mjModel* mm) {
        obstacles.clear();

        for (int i = 0; i < mm->ngeom; ++i) {
            std::string gname = (mm->name_geomadr[i] >= 0)
                ? std::string(mm->names + mm->name_geomadr[i]) : "";

            if (gname.find("obs_") == 0) {
                ObstacleInfo o;
                o.center = Eigen::Vector3d(
                    mm->geom_pos[3*i],
                    mm->geom_pos[3*i+1],
                    mm->geom_pos[3*i+2]
                );
                o.type = (mjtGeom)mm->geom_type[i];
                o.halfsize = Eigen::Vector3d(
                    mm->geom_size[3*i],
                    mm->geom_size[3*i+1],
                    mm->geom_size[3*i+2]
                );
                obstacles.push_back(o);
            }
        }

        std::cout << "[CandidateSampler] Loaded " << obstacles.size()
                  << " obstacle(s), lattice res " << res
                  << "m, no dense occupancy grid built.\n";
    }

    // checks if the guess point is inside an obstacle, sphere or box
    bool nearObstacle(const Eigen::Vector3d& p) const {
        for (const auto& o : obstacles) {
            if (o.type == mjGEOM_SPHERE) {
                if ((p - o.center).norm() <= o.halfsize.x() + OBSTACLE_SAFETY_MARGIN)
                    return true;
            } else if (o.type == mjGEOM_BOX) {
                if (std::abs(p.x() - o.center.x()) <= o.halfsize.x() + OBSTACLE_SAFETY_MARGIN &&
                    std::abs(p.y() - o.center.y()) <= o.halfsize.y() + OBSTACLE_SAFETY_MARGIN &&
                    std::abs(p.z() - o.center.z()) <= o.halfsize.z() + OBSTACLE_SAFETY_MARGIN)
                    return true;
            }
        }
        return false;
    }

    // Aligns sampled point to a voxel grid
    Eigen::Vector3d snapToLattice(const Eigen::Vector3d& p) const {
        auto snap1 = [&](double v, double lo) {
            return lo + res * std::floor((v - lo) / res) + res * 0.5;
        };

        Eigen::Vector3d s(
            snap1(p.x(), ws_min.x()),
            snap1(p.y(), ws_min.y()),
            snap1(p.z(), ws_min.z())
        );

        // check to not round off the edge of defined workspace
        s.x() = std::clamp(s.x(), ws_min.x(), ws_max.x());
        s.y() = std::clamp(s.y(), ws_min.y(), ws_max.y());
        s.z() = std::clamp(s.z(), ws_min.z(), ws_max.z());
        return s;
    }

    // creates the complete list of valid voxel centers once at the beginning
    void generateCandidateVoxels() {
        candidates.clear();

        for (double x = ws_min.x() + 0.5 * res; x < ws_max.x(); x += res) {
            for (double y = ws_min.y() + 0.5 * res; y < ws_max.y(); y += res) {
                for (double z = ws_min.z() + 0.5 * res; z < ws_max.z(); z += res) {
                    Eigen::Vector3d p(x, y, z);

                    // Immediately discard points sampled inside an obstacle voxel.
                    if (!nearObstacle(p))
                        candidates.push_back(p);
                }
            }
        }

        std::cout << "[CandidateSampler] Generated " << candidates.size()
                  << " valid candidate voxels.\n";
    }
};

std::unique_ptr<CandidateSampler> sampler;

// ----------------------------------------------------------------
// GLFW CALLBACKS for user interface
// ----------------------------------------------------------------

void keyCallback(GLFWwindow* w, int key, int, int action, int) {
    if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE)
        glfwSetWindowShouldClose(w, GLFW_TRUE);
}

void mouseButtonCallback(GLFWwindow* w, int button, int action, int) {
    button_left   = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT)   == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right  = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_RIGHT)  == GLFW_PRESS);
    glfwGetCursorPos(w, &lastx, &lasty);
}

void mouseMoveCallback(GLFWwindow* w, double xpos, double ypos) {
    if (!button_left && !button_middle && !button_right) return;

    double dx = xpos - lastx, dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    int width, height;
    glfwGetWindowSize(w, &width, &height);

    mjtMouse action = button_right
        ? mjMOUSE_MOVE_V
        : (button_left ? mjMOUSE_ROTATE_V : mjMOUSE_ZOOM);

    mjv_moveCamera(m, action, dx / height, dy / height, &cam);
}

void scrollCallback(GLFWwindow* w, double, double yoffset) {
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &cam);
}

inline double normalizeAngle(double angle) {
    while (angle > M_PI)  angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// ----------------------------------------------------------------
// kinematics and collisions
// ----------------------------------------------------------------

// EE position from mujoco
Eigen::Vector3d getEEPosition() {
    int id = mj_name2id(m, mjOBJ_SITE, EE_SITE);
    return Eigen::Vector3d(
        d->site_xpos[3*id+0],
        d->site_xpos[3*id+1],
        d->site_xpos[3*id+2]
    );
}

// EE rotation from mujoco
Eigen::Matrix3d getEERotation() {
    int id = mj_name2id(m, mjOBJ_SITE, EE_SITE);
    Eigen::Matrix3d R;

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R(i,j) = d->site_xmat[9*id + 3*i+j];

    return R;
}

// Jacobian from mujoco
Eigen::MatrixXd getEEJacobian6() {
    int id = mj_name2id(m, mjOBJ_SITE, EE_SITE);

    std::vector<double> jacp(3 * m->nv, 0.0);
    std::vector<double> jacr(3 * m->nv, 0.0);

    mj_jacSite(m, d, jacp.data(), jacr.data(), id);

    Eigen::MatrixXd J(6, N_JOINTS);

    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < N_JOINTS; ++c) {
            J(r, c)     = jacp[r*m->nv+c];
            J(r+3, c)   = jacr[r*m->nv+c];
        }
    }

    return J;
}

// finds shortest rotational path as an axis-angle vector
Eigen::Vector3d rotationError(
    const Eigen::Matrix3d& R_cur,
    const Eigen::Matrix3d& R_des)
{
    Eigen::Matrix3d R_err = R_des * R_cur.transpose();

    double cos_theta = std::max(
        -1.0,
        std::min(1.0, 0.5 * (R_err.trace() - 1.0))
    );

    double theta = std::acos(cos_theta);

    if (std::abs(theta) < 1e-6)
        return Eigen::Vector3d::Zero();

    // Handle the 180 degree case separately because sin(theta) is
    // approximately zero and the normal axis-angle expression becomes unstable.
    if (std::abs(M_PI - theta) < 1e-5) {
        Eigen::Vector3d axis;

        axis.x() = std::sqrt(std::max(0.0, (R_err(0,0) + 1.0) * 0.5));
        axis.y() = std::sqrt(std::max(0.0, (R_err(1,1) + 1.0) * 0.5));
        axis.z() = std::sqrt(std::max(0.0, (R_err(2,2) + 1.0) * 0.5));

        if (R_err(2,1) - R_err(1,2) < 0.0) axis.x() = -axis.x();
        if (R_err(0,2) - R_err(2,0) < 0.0) axis.y() = -axis.y();
        if (R_err(1,0) - R_err(0,1) < 0.0) axis.z() = -axis.z();

        if (axis.norm() < 1e-6)
            axis = Eigen::Vector3d::UnitX();
        else
            axis.normalize();

        return theta * axis;
    }

    double factor = theta / (2.0 * std::sin(theta));

    return factor * Eigen::Vector3d(
        R_err(2,1) - R_err(1,2),
        R_err(0,2) - R_err(2,0),
        R_err(1,0) - R_err(0,1)
    );
}

// temporary buffer to check the point
void getPoseAndJacobianAt(
    const Eigen::VectorXd& q,
    Eigen::Vector3d& p,
    Eigen::Matrix3d& R,
    Eigen::MatrixXd& J)
{
    static std::vector<double> qpos_save(m->nq);

    for (int i = 0; i < m->nq; ++i)
        qpos_save[i] = d->qpos[i];

    for (int i = 0; i < N_JOINTS; ++i)
        d->qpos[i] = q(i);

    mj_kinematics(m, d);
    mj_comPos(m, d);

    p = getEEPosition();
    R = getEERotation();
    J = getEEJacobian6();

    for (int i = 0; i < m->nq; ++i)
        d->qpos[i] = qpos_save[i];

    mj_kinematics(m, d);
    mj_comPos(m, d);
}

// uses mujoco collision engine to see if the configuration is valid
// ----------------------------------------------------------------
// collision checking
//
// MuJoCo checks the complete robot configuration. During debugging,
// distinguish between:
//   1. robot <-> obstacle collision
//   2. robot self-collision
//
// This is important because the voxel sampler only knows about the
// obstacle geometry, while MuJoCo checks every enabled contact.
// ----------------------------------------------------------------
bool checkCollision(const Eigen::VectorXd& q) {

    static std::vector<double> qpos_save(m->nq);

    // Save the current MuJoCo configuration so that collision
    // checking does not permanently modify the robot state.
    for (int i = 0; i < m->nq; ++i)
        qpos_save[i] = d->qpos[i];

    // Set the robot to the configuration being tested.
    for (int i = 0; i < N_JOINTS; ++i)
        d->qpos[i] = q(i);

    // Update forward kinematics and collision information.
    mj_kinematics(m, d);
    mj_comPos(m, d);
    mj_collision(m, d);

    bool collide = false;

    // Check every contact generated by MuJoCo.
    for (int i = 0; i < d->ncon; ++i) {

        int g1 = d->contact[i].geom1;
        int g2 = d->contact[i].geom2;

        std::string n1 =
            (m->name_geomadr[g1] >= 0)
            ? std::string(m->names + m->name_geomadr[g1])
            : "unknown";

        std::string n2 =
            (m->name_geomadr[g2] >= 0)
            ? std::string(m->names + m->name_geomadr[g2])
            : "unknown";

        bool obstacle_contact =
            (n1.find("obs_") == 0) ||
            (n2.find("obs_") == 0);

        if (obstacle_contact) {

            std::cout
                << "        CONTACT [OBSTACLE]: "
                << n1 << " (geom " << g1 << ")"
                << " <-> "
                << n2 << " (geom " << g2 << ")\n";

            collide = true;
            break;
        }

    }

    // Restore the original configuration.
    for (int i = 0; i < m->nq; ++i)
        d->qpos[i] = qpos_save[i];

    mj_kinematics(m, d);
    mj_comPos(m, d);

    return collide;
}

// ----------------------------------------------------------------
// SE(3) screw interpolation and graph nodes
// ----------------------------------------------------------------

struct RRTNode {
    Eigen::VectorXd q; // joint angles of robot at point
    Eigen::Vector3d p; // 3d position of EE
    Eigen::Matrix3d R; // 3d rotation of EE
    int parent;        // index of previous node
};

// skew-symmetric matrix from a 3d vector
Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S <<      0.0, -v.z(),  v.y(),
           v.z(),      0.0, -v.x(),
          -v.y(),  v.x(),     0.0;
    return S;
}

// exponential map for a rotation vector
Eigen::Matrix3d rotationExp(const Eigen::Vector3d& w) {
    double theta = w.norm();

    if (theta < 1e-9)
        return Eigen::Matrix3d::Identity();

    Eigen::Vector3d axis = w / theta;
    Eigen::Matrix3d K = skew(axis);

    return Eigen::Matrix3d::Identity()
         + std::sin(theta) * K
         + (1.0 - std::cos(theta)) * K * K;
}

// builds the V matrix used by the SE(3) exponential map
Eigen::Matrix3d se3V(const Eigen::Vector3d& w) {
    double theta = w.norm();
    Eigen::Matrix3d W = skew(w);
    Eigen::Matrix3d W2 = W * W;

    if (theta < 1e-8) {
        // Small-angle approximation prevents division by zero.
        return Eigen::Matrix3d::Identity()
             + 0.5 * W
             + (1.0 / 6.0) * W2;
    }

    return Eigen::Matrix3d::Identity()
         + ((1.0 - std::cos(theta)) / (theta * theta)) * W
         + ((theta - std::sin(theta)) / (theta * theta * theta)) * W2;
}

// calculates the body-frame translational twist associated with a rotation
// and translation between two poses.
Eigen::Vector3d se3LogTranslation(
    const Eigen::Vector3d& p_rel,
    const Eigen::Vector3d& w)
{
    double theta = w.norm();

    if (theta < 1e-8)
        return p_rel;

    Eigen::Matrix3d W = skew(w);
    Eigen::Matrix3d W2 = W * W;

    // Inverse of the V matrix from the SE(3) exponential map.
    Eigen::Matrix3d V_inv =
        Eigen::Matrix3d::Identity()
        - 0.5 * W
        + (1.0 / (theta * theta)
           - (1.0 + std::cos(theta)) /
             (2.0 * theta * std::sin(theta))) * W2;

    return V_inv * p_rel;
}

// true screw interpolation between two rigid-body poses.
// Translation and rotation are coupled through one SE(3) exponential map.
void screwInterpolate(
    const Eigen::Vector3d& p1,
    const Eigen::Matrix3d& R1,
    const Eigen::Vector3d& p2,
    const Eigen::Matrix3d& R2,
    double t,
    Eigen::Vector3d& p_out,
    Eigen::Matrix3d& R_out)
{
    t = std::clamp(t, 0.0, 1.0);

    // Relative pose expressed in the frame of pose 1.
    Eigen::Matrix3d R_rel = R1.transpose() * R2;
    Eigen::Vector3d p_rel = R1.transpose() * (p2 - p1);

    // Rotation logarithm gives the screw angular component.
    Eigen::Vector3d w;

    double cos_theta = std::max(
        -1.0,
        std::min(1.0, 0.5 * (R_rel.trace() - 1.0))
    );

    double theta = std::acos(cos_theta);

    if (theta < 1e-8) {
        w = Eigen::Vector3d::Zero();
    } else if (std::abs(M_PI - theta) < 1e-5) {
        Eigen::Vector3d axis;
        axis.x() = std::sqrt(std::max(0.0, (R_rel(0,0) + 1.0) * 0.5));
        axis.y() = std::sqrt(std::max(0.0, (R_rel(1,1) + 1.0) * 0.5));
        axis.z() = std::sqrt(std::max(0.0, (R_rel(2,2) + 1.0) * 0.5));

        if (R_rel(2,1) - R_rel(1,2) < 0.0) axis.x() = -axis.x();
        if (R_rel(0,2) - R_rel(2,0) < 0.0) axis.y() = -axis.y();
        if (R_rel(1,0) - R_rel(0,1) < 0.0) axis.z() = -axis.z();

        if (axis.norm() < 1e-6)
            axis = Eigen::Vector3d::UnitX();
        else
            axis.normalize();

        w = theta * axis;
    } else {
        double factor = theta / (2.0 * std::sin(theta));
        w = factor * Eigen::Vector3d(
            R_rel(2,1) - R_rel(1,2),
            R_rel(0,2) - R_rel(2,0),
            R_rel(1,0) - R_rel(0,1)
        );
    }

    // The translational part is coupled to the same screw parameter.
    Eigen::Vector3d v = se3LogTranslation(p_rel, w);

    Eigen::Vector3d w_t = t * w;
    Eigen::Vector3d v_t = t * v;

    Eigen::Matrix3d R_t = rotationExp(w_t);
    Eigen::Matrix3d V_t = se3V(w_t);

    Eigen::Vector3d p_t = V_t * v_t;

    R_out = R1 * R_t;
    p_out = p1 + R1 * p_t;
}

// determines the amount of rotation to assign to a voxel based on how much
// translation has been made toward the final goal.
Eigen::Matrix3d orientationAtCandidate(
    const Eigen::Vector3d& p_current,
    const Eigen::Matrix3d& R_current,
    const Eigen::Vector3d& p_candidate,
    const Eigen::Vector3d& p_goal,
    const Eigen::Matrix3d& R_goal)
{
    double total_distance = (p_goal - p_current).norm();

    if (total_distance < 1e-8)
        return R_goal;

    // The ratio is local information: how far the candidate moves relative
    // to the remaining translation toward the final goal.
    double candidate_distance = (p_candidate - p_current).norm();
    double alpha = std::clamp(candidate_distance / total_distance, 0.0, 1.0);

    Eigen::Quaterniond q_current(R_current);
    Eigen::Quaterniond q_goal(R_goal);

    return q_current.slerp(alpha, q_goal).normalized().toRotationMatrix();
}

// ----------------------------------------------------------------
// inverse kinematics
// ----------------------------------------------------------------

bool checkJointLimits(const Eigen::VectorXd& q);

// 3d EE pose into 7 joint angles
bool solveIK_RMRC(
    const Eigen::VectorXd& q_init,
    const Eigen::Vector3d& p_des,
    const Eigen::Matrix3d& R_des,
    Eigen::VectorXd& q_out)
{
    q_out = q_init;

    const int MAX_IK_ITERS = 40;
    const double IK_TOL = 2e-3;

    for (int i = 0; i < MAX_IK_ITERS; ++i) {
        Eigen::Vector3d p_cur;
        Eigen::Matrix3d R_cur;
        Eigen::MatrixXd J;

        getPoseAndJacobianAt(q_out, p_cur, R_cur, J);

        Eigen::VectorXd err(6);
        err.head(3) = p_des - p_cur;
        err.tail(3) = rotationError(R_cur, R_des);

        if (err.norm() < IK_TOL)
            return true;

        const double lambda2 = 0.01 * 0.01;

        Eigen::MatrixXd JJT = J * J.transpose();
        JJT.diagonal().array() += lambda2;

        Eigen::MatrixXd J_pinv = J.transpose() * JJT.inverse();

        // Null-space motion keeps the redundant 7th DOF close to a
        // comfortable configuration while the Jacobian handles the task.
        Eigen::MatrixXd N =
            Eigen::MatrixXd::Identity(N_JOINTS, N_JOINTS)
            - J_pinv * J;

        Eigen::VectorXd q_null(N_JOINTS);

        for (int k = 0; k < N_JOINTS; ++k)
            q_null(k) = Q_MID[k] - q_out(k);

        Eigen::VectorXd dq = J_pinv * err + 0.08 * N * q_null;

        for (int k = 0; k < N_JOINTS; ++k) {
            // Keep the individual update small so that consecutive IK
            // solutions remain on the same local configuration branch.
            dq(k) = std::max(-0.15, std::min(0.15, dq(k)));
            q_out(k) += dq(k);
        }

        if (!checkJointLimits(q_out))
            return false;
    }

    return false;
}

bool checkJointLimits(const Eigen::VectorXd& q) {
    for (int i = 0; i < N_JOINTS; ++i) {
        if (q(i) < Q_LIM_LOW[i] || q(i) > Q_LIM_HIGH[i])
            return false;
    }
    return true;
}

// ----------------------------------------------------------------
// candidate scoring and graph expansion
// ----------------------------------------------------------------

// gives each voxel a score based on whether it moves the EE toward the goal.
// Higher scores are preferred.
double scoreCandidate(
    const RRTNode& current,
    const Eigen::Vector3d& candidate,
    const Eigen::Vector3d& p_goal)
{
    Eigen::Vector3d to_goal = p_goal - current.p;
    Eigen::Vector3d to_candidate = candidate - current.p;

    double current_goal_dist = to_goal.norm();
    double candidate_goal_dist = (p_goal - candidate).norm();

    if (current_goal_dist < 1e-8)
        return -std::numeric_limits<double>::infinity();

    // Positive when the candidate gets closer to the goal.
    double goal_progress =
        (current_goal_dist - candidate_goal_dist) / current_goal_dist;

    double direction_score = 0.0;

    if (to_candidate.norm() > 1e-8) {
        direction_score =
            to_candidate.normalized().dot(to_goal.normalized());
    }

    // Prefer points that are not excessively far from the current node.
    double distance_penalty =
        std::min(to_candidate.norm() / current_goal_dist, 1.0);

    return GOAL_SCORE_WEIGHT * goal_progress
         + DIRECTION_SCORE_WEIGHT * direction_score
         - DISTANCE_SCORE_WEIGHT * distance_penalty;
}

// returns true if the candidate voxel is already represented by a node
bool candidateAlreadyUsed(
    const std::vector<RRTNode>& tree,
    const Eigen::Vector3d& candidate)
{
    const double tolerance = sampler->res * 0.25;

    for (const auto& node : tree) {
        if ((node.p - candidate).norm() < tolerance)
            return true;
    }

    return false;
}

// builds one complete screw-coupled motion from the current node to a
// candidate voxel. Collision is checked after every intermediate pose.
int extendToCandidate(
    std::vector<RRTNode>& tree,
    int near_idx,
    const Eigen::Vector3d& p_candidate,
    const Eigen::Vector3d& p_goal,
    const Eigen::Matrix3d& R_goal)
{
    const RRTNode node_near = tree[near_idx];

    // Orientation is selected from the candidate's translational progress.
    Eigen::Matrix3d R_candidate =
        orientationAtCandidate(
            node_near.p,
            node_near.R,
            p_candidate,
            p_goal,
            R_goal
        );

    double pos_dist = (p_candidate - node_near.p).norm();
    double ang_dist = rotationError(node_near.R, R_candidate).norm();

    int steps = std::max(
        1,
        (int)std::ceil(
            std::max(
                pos_dist / STEP_POS,
                ang_dist / STEP_ANG
            )
        )
    );

    int current_parent = near_idx;
    Eigen::VectorXd q_cur = node_near.q;

    for (int k = 1; k <= steps; ++k) {
        double t = (double)k / steps;

        Eigen::Vector3d p_step;
        Eigen::Matrix3d R_step;

        // Generate middle poses using one coupled SE(3) screw motion.
        screwInterpolate(
            node_near.p,
            node_near.R,
            p_candidate,
            R_candidate,
            t,
            p_step,
            R_step
        );

        // Candidate voxels are only a position pre-filter. MuJoCo
        // collision checking below remains authoritative.
        if (sampler->nearObstacle(p_step))
            return near_idx;

        Eigen::VectorXd q_next;

        // Seed IK from the immediately previous valid configuration.
        // This is important for maintaining the same redundant-manipulator
        // branch instead of allowing a sudden elbow configuration change.
 // ----------------------------------------------------------------
// Candidate motion validation
// ----------------------------------------------------------------

// Reject the candidate if the intermediate voxel lies inside an
// obstacle. This is the inexpensive geometric pre-filter.
if (sampler->nearObstacle(p_step)) {
    std::cout << "      rejected: voxel is too close to obstacle\n";
    return near_idx;
}

// Solve IK using the previous configuration as the seed.
// This keeps the planner on a continuous redundant-manipulator
// configuration branch.

if (!solveIK_RMRC(q_cur, p_step, R_step, q_next)) {
    std::cout << "      rejected: IK failed\n";
    return near_idx;
}

// Reject configurations outside the robot's allowed joint limits.
if (!checkJointLimits(q_next)) {
    std::cout << "      rejected: joint limits\n";
    return near_idx;
}

// Prevent the redundant IK solver from jumping to a completely
// different configuration branch.
double joint_step = (q_next - q_cur).norm();

if (joint_step > MAX_PARENT_JOINT_STEP) {
    std::cout << "      rejected: joint-space jump = "
              << joint_step << " rad\n";
    return near_idx;
}

// MuJoCo remains the authoritative collision detector.
// Every intermediate configuration is checked individually.
if (checkCollision(q_next)) {
    std::cout << "      rejected: MuJoCo collision\n";
    return near_idx;
}
        

        RRTNode new_node;
        new_node.q = q_next;

        Eigen::MatrixXd J_dummy;
        getPoseAndJacobianAt(
            q_next,
            new_node.p,
            new_node.R,
            J_dummy
        );

        new_node.parent = current_parent;
        tree.push_back(new_node);

        current_parent = (int)tree.size() - 1;
        q_cur = q_next;
    }

    return current_parent;
}

// ----------------------------------------------------------------
// single-tree voxel / screw planner
// ----------------------------------------------------------------

struct PlanResult {
    bool success = false;
    std::vector<Eigen::VectorXd> path;
    Eigen::Vector3d connect_point = Eigen::Vector3d::Zero();
};

struct CandidateScore {
    Eigen::Vector3d point;
    double score;
};

// searches the entire valid voxel set and keeps the highest scoring
// candidates that have not already been used by the graph.
std::vector<CandidateScore> rankCandidates(
    const std::vector<RRTNode>& tree,
    const RRTNode& current,
    const Eigen::Vector3d& p_goal)
{
    std::vector<CandidateScore> ranked;

    for (const auto& candidate : sampler->candidates) {
        if (candidateAlreadyUsed(tree, candidate))
            continue;

        // Do not select a point that is effectively the current point.
        if ((candidate - current.p).norm() < 1e-5)
            continue;

        CandidateScore c;
        c.point = candidate;
        c.score = scoreCandidate(current, candidate, p_goal);
        ranked.push_back(c);
    }

    std::sort(
        ranked.begin(),
        ranked.end(),
        [](const CandidateScore& a, const CandidateScore& b) {
            return a.score > b.score;
        }
    );

    if ((int)ranked.size() > CANDIDATES_PER_EXPANSION)
        ranked.resize(CANDIDATES_PER_EXPANSION);

    return ranked;
}

PlanResult planVoxelScrew(
    const Eigen::VectorXd& q_start,
    const Eigen::Vector3d& p_goal,
    const Eigen::Matrix3d& R_goal)
{
    std::cout << "=== Voxel / Screw-Coupled Graph Planner ===\n";

    PlanResult result;
    std::vector<RRTNode> tree;

    // root node at the current joint configuration
    RRTNode root;
    root.q = q_start;

    Eigen::MatrixXd J_dummy;
    getPoseAndJacobianAt(
        q_start,
        root.p,
        root.R,
        J_dummy
    );

    root.parent = -1;
    tree.push_back(root);

    std::cout << "  Start pose: " << root.p.transpose() << "\n";
    std::cout << "  Goal pose:  " << p_goal.transpose() << "\n";

    int goal_idx = -1;

    for (int it = 0; it < PLANNER_MAX_ITERS; ++it) {
        // Check whether the current best node has already reached the goal.
        int best_idx = 0;
        double best_goal_dist = (tree[0].p - p_goal).norm();

        for (size_t i = 1; i < tree.size(); ++i) {
            double dist = (tree[i].p - p_goal).norm();
            if (dist < best_goal_dist) {
                best_goal_dist = dist;
                best_idx = (int)i;
            }
        }

        double best_angle =
            rotationError(tree[best_idx].R, R_goal).norm();

        if (best_goal_dist < GOAL_POSITION_TOL &&
            best_angle < GOAL_ANGLE_TOL) {
            goal_idx = best_idx;

            std::cout << "  [SUCCESS] Goal reached at iteration "
                      << it
                      << " | nodes: " << tree.size()
                      << " | position error: " << best_goal_dist
                      << " | angular error: " << best_angle
                      << "\n";
            break;
        }

        // Normally expand from the node closest to the goal. This keeps
        // the graph moving forward instead of repeatedly expanding old branches.
        int expand_idx = best_idx;

        // Occasionally expand from another existing node so that the planner
        // can recover if the most forward node becomes trapped by obstacles.
        if (uni01(rng) < 0.20 && tree.size() > 2) {
            std::uniform_int_distribution<int> node_dist(
                0,
                (int)tree.size() - 1
            );
            expand_idx = node_dist(rng);
        }

        // A small amount of goal bias tries the goal position directly.
        std::vector<CandidateScore> ranked;

        if (uni01(rng) < GOAL_BIAS) {
            if (!candidateAlreadyUsed(tree, p_goal) &&
                !sampler->nearObstacle(p_goal)) {
                ranked.push_back({p_goal, std::numeric_limits<double>::infinity()});
            }
        }

        if (ranked.empty())
            ranked = rankCandidates(tree, tree[expand_idx], p_goal);

        if (ranked.empty()) {
            if (it % 100 == 0)
                std::cout << "  Iter " << it
                          << " | no unused candidates remain\n";
            continue;
        }

        bool added_node = false;

        // Try the best few candidate voxels. A failed candidate is discarded
        // without modifying the graph.
        for (const auto& candidate : ranked) {
            int reached_idx =
                extendToCandidate(
                    tree,
                    expand_idx,
                    candidate.point,
                    p_goal,
                    R_goal
                );

            if (reached_idx != expand_idx) {
                added_node = true;

                double goal_dist =
                    (tree[reached_idx].p - p_goal).norm();

                std::cout << "  Iter " << it
                          << " | added node " << reached_idx
                          << " | goal distance: " << goal_dist
                          << "\n";

                break;
            }
        }

        if (!added_node && it % 100 == 0) {
            std::cout << "  Iter " << it
                      << " | expansion blocked\n";
        }

        // If the graph has reached the goal during this expansion, stop.
        if (!tree.empty()) {
            int latest_idx = (int)tree.size() - 1;

            double pos_err =
                (tree[latest_idx].p - p_goal).norm();
            double ang_err =
                rotationError(tree[latest_idx].R, R_goal).norm();

            if (pos_err < GOAL_POSITION_TOL &&
                ang_err < GOAL_ANGLE_TOL) {
                goal_idx = latest_idx;

                std::cout << "  [SUCCESS] Goal reached at iteration "
                          << it
                          << " | nodes: " << tree.size()
                          << " | position error: " << pos_err
                          << " | angular error: " << ang_err
                          << "\n";
                break;
            }
        }
    }

    if (goal_idx < 0) {
        std::cerr << "  [FAILURE] Planner could not reach the goal "
                     "within the iteration limit.\n";
        return result;
    }

    // backtrack from the goal node to the start node
    std::vector<Eigen::VectorXd> path;

    for (int cur = goal_idx; cur >= 0; cur = tree[cur].parent)
        path.push_back(tree[cur].q);

    std::reverse(path.begin(), path.end());

    result.path = path;
    result.success = true;

    if (!path.empty())
        result.connect_point = tree[goal_idx].p;

    return result;
}

// ----------------------------------------------------------------
// path preparation
// ----------------------------------------------------------------

// The planner already stores collision-checked intermediate configurations,
// so no blind joint-space smoothing is performed here.
// This avoids creating a new trajectory that was never collision checked.
std::vector<Eigen::VectorXd> preparePath(
    const std::vector<Eigen::VectorXd>& raw_path)
{
    return raw_path;
}

// ----------------------------------------------------------------
// VISUALIZATION: pose markers
// ----------------------------------------------------------------

void renderPoseMarker(
    mjvScene* sc,
    const Eigen::Vector3d& pos,
    const Eigen::Matrix3d& R,
    const float rgba[4],
    double size = 0.03)
{
    if (sc->ngeom >= sc->maxgeom) return;

    mjvGeom* g = sc->geoms + sc->ngeom;

    mjv_initGeom(
        g,
        mjGEOM_BOX,
        nullptr,
        nullptr,
        nullptr,
        nullptr
    );

    g->pos[0] = pos.x();
    g->pos[1] = pos.y();
    g->pos[2] = pos.z();

    g->size[0] = size;
    g->size[1] = size * 0.6;
    g->size[2] = size * 0.35;

    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            g->mat[r*3+c] = (float)R(r, c);

    g->rgba[0] = rgba[0];
    g->rgba[1] = rgba[1];
    g->rgba[2] = rgba[2];
    g->rgba[3] = rgba[3];

    g->category = mjCAT_DECOR;
    sc->ngeom++;
}

void renderSphereMarker(
    mjvScene* sc,
    const Eigen::Vector3d& pos,
    const float rgba[4],
    double radius = 0.02)
{
    if (sc->ngeom >= sc->maxgeom) return;

    mjvGeom* g = sc->geoms + sc->ngeom;

    mjv_initGeom(
        g,
        mjGEOM_SPHERE,
        nullptr,
        nullptr,
        nullptr,
        nullptr
    );

    g->pos[0] = pos.x();
    g->pos[1] = pos.y();
    g->pos[2] = pos.z();

    g->size[0] = radius;
    g->size[1] = radius;
    g->size[2] = radius;

    g->rgba[0] = rgba[0];
    g->rgba[1] = rgba[1];
    g->rgba[2] = rgba[2];
    g->rgba[3] = rgba[3];

    g->category = mjCAT_DECOR;
    sc->ngeom++;
}

// ----------------------------------------------------------------
// MAIN ENTRY
// ----------------------------------------------------------------

int main() {
    char error[1000];

    m = mj_loadXML(
        XML_PATH,
        nullptr,
        error,
        sizeof(error)
    );

    if (!m) {
        std::cerr << "Error loading XML: " << error << std::endl;
        return 1;
    }

    d = mj_makeData(m);

    if (!glfwInit())
        return 1;

    window = glfwCreateWindow(
        1200,
        900,
        "Kinova Gen3 Voxel / Screw Motion Planner",
        nullptr,
        nullptr
    );

    // ----------------------------------------------------------------
// Collision baseline test.
//
// Check the starting robot configuration before the planner runs.
// If this is already colliding with the test obstacle, then the
// planner is not responsible for the initial collision.
// ----------------------------------------------------------------

Eigen::VectorXd q_home(N_JOINTS);

// ----------------------------------------------------------------
// Collision baseline test.
//
// Check the exact starting robot configuration used by the planner.
// If this configuration is already colliding with the obstacle,
// then the planner cannot make a valid first move.
// ----------------------------------------------------------------

Eigen::VectorXd q_start_test(N_JOINTS);

q_start_test <<
    0.0,
    0.3,
    0.0,
    -1.2,
    0.0,
    0.6,
    0.0;

std::cout << "\n=== Starting Configuration Collision Test ===\n";

bool start_collision = checkCollision(q_start_test);

if (start_collision)
    std::cout << "  START CONFIGURATION: COLLIDING\n";
else
    std::cout << "  START CONFIGURATION: CLEAR\n";

std::cout << "=============================================\n\n";
std::cout << "\n=== Starting Configuration Collision Test ===\n";

bool home_collision = checkCollision(q_home);

if (home_collision)
    std::cout << "  START CONFIGURATION: COLLIDING\n";
else
    std::cout << "  START CONFIGURATION: CLEAR\n";

std::cout << "=============================================\n\n";

    glfwMakeContextCurrent(window);

    glfwSetKeyCallback(window, keyCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetCursorPosCallback(window, mouseMoveCallback);
    glfwSetScrollCallback(window, scrollCallback);

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_makeScene(m, &scn, 50000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    cam.distance = 2.0;
    cam.azimuth  = 135;
    cam.elevation = -25;

    cam.lookat[0] = 0.2;
    cam.lookat[1] = 0.0;
    cam.lookat[2] = 0.5;

    sampler = std::make_unique<CandidateSampler>(
        CANDIDATE_VOXEL_RES,
        Eigen::Vector3d(WS_X_MIN, WS_Y_MIN, WS_Z_MIN),
        Eigen::Vector3d(WS_X_MAX, WS_Y_MAX, WS_Z_MAX)
    );

    sampler->populateFromMuJoCo(m);
    sampler->generateCandidateVoxels();

    double q_start_arr[N_JOINTS] =
        {0.0, 0.3, 0.0, -1.2, 0.0, 0.6, 0.0};

    for (int i = 0; i < N_JOINTS; ++i)
        d->qpos[i] = q_start_arr[i];

    mj_forward(m, d);

    Eigen::VectorXd q_start_vec(N_JOINTS);

    for (int i = 0; i < N_JOINTS; ++i)
        q_start_vec(i) = q_start_arr[i];

    Eigen::Vector3d p_goal(0.40, -0.15, 0.45);

    Eigen::Matrix3d R_goal =
        Eigen::AngleAxisd(
            M_PI,
            Eigen::Vector3d::UnitX()
        ).toRotationMatrix();

    // Check the requested goal pose once before starting the planner.
    // The planner now keeps one continuous IK branch from the start rather
    // than creating an independently seeded goal-side tree.
    Eigen::VectorXd q_goal_check;

    bool goal_ik_ok =
        solveIK_RMRC(
            q_start_vec,
            p_goal,
            R_goal,
            q_goal_check
        );

    if (goal_ik_ok) {
        std::cout << "  Goal pose is reachable from the start IK branch.\n";
    } else {
        std::cout << "  Goal pose was not reached directly by IK. "
                     "The graph will still attempt intermediate voxel poses.\n";
    }

    Eigen::Vector3d p_start;
    Eigen::Matrix3d R_start;
    Eigen::MatrixXd J_dummy;

    getPoseAndJacobianAt(
        q_start_vec,
        p_start,
        R_start,
        J_dummy
    );

    PlanResult plan =
        planVoxelScrew(
            q_start_vec,
            p_goal,
            R_goal
        );

    if (!plan.success) {
        mj_deleteData(d);
        mj_deleteModel(m);
        glfwTerminate();
        return 1;
    }

    auto motion_path = preparePath(plan.path);

    std::cout << "  Collision-checked path contains "
              << motion_path.size()
              << " configurations.\n";

    const float START_RGBA[4]   = {0.15f, 0.9f, 0.3f, 0.9f};
    const float GOAL_RGBA[4]    = {0.95f, 0.25f, 0.2f, 0.9f};
    const float CONNECT_RGBA[4] = {0.95f, 0.85f, 0.15f, 0.9f};

    std::cout << "\nReplaying motion path in MuJoCo. Press ESC to quit.\n";

    while (!glfwWindowShouldClose(window)) {
        for (const auto& q_step : motion_path) {
            if (glfwWindowShouldClose(window))
                break;

            for (int i = 0; i < N_JOINTS; ++i)
                d->qpos[i] = q_step[i];

            // USE KINEMATICS, NOT FORWARD
            // Disables physics engine twitching during replay.
            mj_kinematics(m, d);

            int w, h;
            glfwGetFramebufferSize(window, &w, &h);

            mjrRect viewport = {0, 0, w, h};

            mjv_updateScene(
                m,
                d,
                &opt,
                nullptr,
                &cam,
                mjCAT_ALL,
                &scn
            );

            // Visualize the fixed start and goal pose.
            renderPoseMarker(
                &scn,
                p_start,
                R_start,
                START_RGBA
            );

            renderPoseMarker(
                &scn,
                p_goal,
                R_goal,
                GOAL_RGBA
            );

            // Yellow sphere now marks the final reached node rather than
            // an artificial tree-connection seam.
            renderSphereMarker(
                &scn,
                plan.connect_point,
                CONNECT_RGBA
            );

            mjr_render(viewport, &scn, &con);

            glfwSwapBuffers(window);
            glfwPollEvents();

            std::this_thread::sleep_for(
                std::chrono::milliseconds(20)
            );
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(600)
        );
    }

    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();

    return 0;
}

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <chrono>
#include <thread>
#include <algorithm>
#include <memory>
#include <Eigen/Dense>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

// ----------------------------------------------------------------
// constants and parameters:
// ----------------------------------------------------------------

// from URDF:
static const int    N_JOINTS = 7;
static const char*  XML_PATH = "/home/ash_gomes/gplanner/mujoco_menagerie/kinova_gen3/gen3.xml"; 
static const char*  EE_SITE  = "pinch_site";

static const double Q_MID[N_JOINTS] = {0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57};  //comfort home position pose
static const double Q_LIM_LOW[N_JOINTS]  = {-6.28, -2.24, -6.28, -2.57, -6.28, -2.09, -6.28}; // minimum joint limits
static const double Q_LIM_HIGH[N_JOINTS] = { 6.28,  2.24,  6.28,  2.57,  6.28,  2.09,  6.28}; // maximum joint limits

// workspace limits (x, y, z): 
static const double WS_X_MIN = -0.2, WS_X_MAX = 0.65;
static const double WS_Y_MIN = -0.5, WS_Y_MAX = 0.5;
static const double WS_Z_MIN =  0.3, WS_Z_MAX = 0.95;

static const int    RRT_MAX_ITERS     = 6000; // max iterations, shared budget across both trees
static const double RRT_GOAL_BIAS     = 0.15; // chance to target the other tree's root exactly
static const double RRT_OBS_BIAS      = 0.30; // chance to target the edge of an obstacle 
static const double RRT_SE3_STEP_POS  = 0.03; // maximum translation (in cm)
static const double RRT_SE3_STEP_ANG  = 0.08; // maximum rotation (in rad)

// Tolerance for declaring the two trees "connected"
static const double CONNECT_TOL_POS   = 0.04; // max positional error
static const double CONNECT_TOL_ANG   = 0.12; // max angular error

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

// occupancy grid removed from previous iteration. Takes the obstacle geometry from the URDF and reconstructs it here.

struct ObstacleInfo {
    Eigen::Vector3d center; 
    mjtGeom type;
    Eigen::Vector3d halfsize; // dimensions 
};

struct CandidateSampler {
    double res; // voxel resolution 
    Eigen::Vector3d ws_min, ws_max;
    std::vector<ObstacleInfo> obstacles;

    CandidateSampler(double resolution, const Eigen::Vector3d& lo, const Eigen::Vector3d& hi)
        : res(resolution), ws_min(lo), ws_max(hi) {}
        // takes resolution and boundary and assings to internal member variables

        // goes through all the obstacles 
    void populateFromMuJoCo(mjModel* mm) { 
        obstacles.clear();
        for (int i = 0; i < mm->ngeom; ++i) {
            std::string gname = (mm->name_geomadr[i] >= 0) ? std::string(mm->names + mm->name_geomadr[i]) : "";
            if (gname.find("obs_") == 0) {
                ObstacleInfo o;
                o.center = Eigen::Vector3d(mm->geom_pos[3*i], mm->geom_pos[3*i+1], mm->geom_pos[3*i+2]);
                o.type = (mjtGeom)mm->geom_type[i];
                o.halfsize = Eigen::Vector3d(mm->geom_size[3*i], mm->geom_size[3*i+1], mm->geom_size[3*i+2]);
                obstacles.push_back(o);
            }
        }
        std::cout << "[CandidateSampler] Loaded " << obstacles.size()
                  << " obstacle(s), lattice res " << res << "m, no dense grid built.\n";
    }

    // checks if the guess point is inside an obstacle, sphere or box 
    bool nearObstacle(const Eigen::Vector3d& p) const {
        for (const auto& o : obstacles) {
            if (o.type == mjGEOM_SPHERE) {
                if ((p - o.center).norm() <= o.halfsize.x() + OBSTACLE_SAFETY_MARGIN) return true;
            } else if (o.type == mjGEOM_BOX) {
                if (std::abs(p.x() - o.center.x()) <= o.halfsize.x() + OBSTACLE_SAFETY_MARGIN &&
                    std::abs(p.y() - o.center.y()) <= o.halfsize.y() + OBSTACLE_SAFETY_MARGIN &&
                    std::abs(p.z() - o.center.z()) <= o.halfsize.z() + OBSTACLE_SAFETY_MARGIN) return true;
            }
        }
        return false;
    }

    // Aligns sampled point to a voxel grid 
    Eigen::Vector3d snapToLattice(const Eigen::Vector3d& p) const {
        auto snap1 = [&](double v, double lo) {
            return lo + res * std::floor((v - lo) / res) + res * 0.5;
        };
        Eigen::Vector3d s(snap1(p.x(), ws_min.x()), snap1(p.y(), ws_min.y()), snap1(p.z(), ws_min.z()));

        // check to not round of the edge of defined workspace 
        s.x() = std::clamp(s.x(), ws_min.x(), ws_max.x());
        s.y() = std::clamp(s.y(), ws_min.y(), ws_max.y());
        s.z() = std::clamp(s.z(), ws_min.z(), ws_max.z());
        return s;
    }

    // generates random waypoints
    bool sampleCandidate(Eigen::Vector3d& out, int max_tries = 25) const {
        for (int t = 0; t < max_tries; ++t) {
            Eigen::Vector3d raw(
                ws_min.x() + uni01(rng) * (ws_max.x() - ws_min.x()),
                ws_min.y() + uni01(rng) * (ws_max.y() - ws_min.y()),
                ws_min.z() + uni01(rng) * (ws_max.z() - ws_min.z())
            );
            Eigen::Vector3d snapped = snapToLattice(raw);
            if (!nearObstacle(snapped)) { out = snapped; return true; }
        }
        return false;
    }
    // samples near obstacle 
    bool sampleNearObstacle(Eigen::Vector3d& out) const {
        if (obstacles.empty()) return false;
        int idx = std::uniform_int_distribution<int>(0, (int)obstacles.size() - 1)(rng);
        const auto& o = obstacles[idx];
        Eigen::Vector3d dir(uni01(rng) - 0.5, uni01(rng) - 0.5, uni01(rng) - 0.5);
        if (dir.norm() < 1e-6) dir = Eigen::Vector3d::UnitZ();
        dir.normalize();
        double reach = std::max({o.halfsize.x(), o.halfsize.y(), o.halfsize.z()});
        double offset = reach + 0.12 + 0.15 * uni01(rng);
        Eigen::Vector3d p = o.center + dir * offset;
        p.x() = std::clamp(p.x(), ws_min.x(), ws_max.x());
        p.y() = std::clamp(p.y(), ws_min.y(), ws_max.y());
        p.z() = std::clamp(p.z(), ws_min.z(), ws_max.z());
        out = snapToLattice(p);
        return true;
    }
};

std::unique_ptr<CandidateSampler> sampler;


// ----------------------------------------------------------------
// GLFW CALLBACKS for user interface
// ----------------------------------------------------------------

void keyCallback(GLFWwindow* w, int key, int, int action, int) {
    if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE) glfwSetWindowShouldClose(w, GLFW_TRUE);
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
    lastx = xpos; lasty = ypos;
    int width, height; glfwGetWindowSize(w, &width, &height);
    mjtMouse action = button_right ? mjMOUSE_MOVE_V : (button_left ? mjMOUSE_ROTATE_V : mjMOUSE_ZOOM);
    mjv_moveCamera(m, action, dx/height, dy/height, &cam);
}
void scrollCallback(GLFWwindow* w, double, double yoffset) {
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &cam);
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
    return Eigen::Vector3d(d->site_xpos[3*id+0], d->site_xpos[3*id+1], d->site_xpos[3*id+2]);
}

// EE rotation from mujoco 
Eigen::Matrix3d getEERotation() {
    int id = mj_name2id(m, mjOBJ_SITE, EE_SITE);
    Eigen::Matrix3d R;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) R(i,j) = d->site_xmat[9*id + 3*i+j];
    return R;
}

// Jacobian from mujoco 
Eigen::MatrixXd getEEJacobian6() {
    int id = mj_name2id(m, mjOBJ_SITE, EE_SITE);
    std::vector<double> jacp(3 * m->nv, 0.0), jacr(3 * m->nv, 0.0);
    mj_jacSite(m, d, jacp.data(), jacr.data(), id);

    Eigen::MatrixXd J(6, N_JOINTS);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < N_JOINTS; ++c) {
            J(r, c) = jacp[r*m->nv+c];
            J(r+3, c) = jacr[r*m->nv+c];
        }
    }
    return J;
}

// finds shortest rotational path as an axis-angle vector
Eigen::Vector3d rotationError(const Eigen::Matrix3d& R_cur, const Eigen::Matrix3d& R_des) {
    Eigen::Matrix3d R_err = R_des * R_cur.transpose();
    double cos_theta = std::max(-1.0, std::min(1.0, 0.5 * (R_err.trace() - 1.0)));
    double theta = std::acos(cos_theta);
    if (std::abs(theta) < 1e-6) return Eigen::Vector3d::Zero();
    double factor = theta / (2.0 * std::sin(theta));
    return factor * Eigen::Vector3d(R_err(2,1)-R_err(1,2), R_err(0,2)-R_err(2,0), R_err(1,0)-R_err(0,1));
}

// temporary buffer to check the point 
void getPoseAndJacobianAt(const Eigen::VectorXd& q, Eigen::Vector3d& p, Eigen::Matrix3d& R, Eigen::MatrixXd& J) {
    static std::vector<double> qpos_save(m->nq);
    for (int i = 0; i < m->nq; ++i) qpos_save[i] = d->qpos[i];

    for (int i = 0; i < N_JOINTS; ++i) d->qpos[i] = q(i);
    mj_kinematics(m, d);
    mj_comPos(m, d);

    p = getEEPosition();
    R = getEERotation();
    J = getEEJacobian6();

    for (int i = 0; i < m->nq; ++i) d->qpos[i] = qpos_save[i];
    mj_kinematics(m, d);
    mj_comPos(m, d);
}

// uses mujoco collision engine to see if the configuration is valid 
bool checkCollision(const Eigen::VectorXd& q) {
    static std::vector<double> qpos_save(m->nq);
    for (int i = 0; i < m->nq; ++i) qpos_save[i] = d->qpos[i];

    for (int i = 0; i < N_JOINTS; ++i) d->qpos[i] = q(i);
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

    // if there is a collision, terminates and repeats loop 
    for (int i = 0; i < m->nq; ++i) d->qpos[i] = qpos_save[i];
    mj_kinematics(m, d);
    return collide;
}

// ----------------------------------------------------------------
// RRT NODE, ScLERP, SCREW-COUPLED ORIENTATION
// ----------------------------------------------------------------


struct RRTNode {
    Eigen::VectorXd q; // joint agnles of robot at point
    Eigen::Vector3d p; // 3d position of EE
    Eigen::Matrix3d R; // 3d rotation of EE
    int parent; // index of previous node 
};

void sclerp(const Eigen::Vector3d& p1, const Eigen::Matrix3d& R1,
            const Eigen::Vector3d& p2, const Eigen::Matrix3d& R2,
            double t, Eigen::Vector3d& p_out, Eigen::Matrix3d& R_out) {
    p_out = (1.0 - t) * p1 + t * p2;
    Eigen::Quaterniond q1(R1), q2(R2);
    R_out = q1.slerp(t, q2).toRotationMatrix();
}

// alpha = how far a sampled position sits along [p_near -> p_anchor]; the same
// alpha slerps [R_near -> R_anchor]. "Anchor" used to be a single fixed goal --
// with two trees, each tree's anchor is simply the OTHER tree's (fixed) root pose.
Eigen::Matrix3d coupledOrientation(const Eigen::Vector3d& p_near, const Eigen::Matrix3d& R_near,
                                    const Eigen::Vector3d& p_anchor, const Eigen::Matrix3d& R_anchor,
                                    const Eigen::Vector3d& p_rand) {
    double d_total = (p_anchor - p_near).norm(); // total distance of current position to root of other tree
    double alpha;
    if (d_total < 1e-6) {
        alpha = 1.0;
    } else {
        double d_partial = (p_rand - p_near).norm(); // distance from current position to new node
        alpha = std::clamp(d_partial / d_total, 0.0, 1.0); // ratio between them 
    }
    Eigen::Quaterniond q_near(R_near), q_anchor(R_anchor);
    return q_near.slerp(alpha, q_anchor).toRotationMatrix(); // applies same ratio to rotation 
}

bool checkJointLimits(const Eigen::VectorXd& q) {
    for (int i = 0; i < N_JOINTS; ++i) {
        if (q(i) < Q_LIM_LOW[i] || q(i) > Q_LIM_HIGH[i]) return false;
    }
    return true;
}

// 3d eEE pose into 7 joint angles 
bool solveIK_RMRC(const Eigen::VectorXd& q_init, const Eigen::Vector3d& p_des, const Eigen::Matrix3d& R_des, Eigen::VectorXd& q_out) {
    q_out = q_init;
    const int MAX_IK_ITERS = 30;
    const double IK_TOL = 2e-3;

    for (int i = 0; i < MAX_IK_ITERS; ++i) {
        Eigen::Vector3d p_cur; Eigen::Matrix3d R_cur; Eigen::MatrixXd J;
        getPoseAndJacobianAt(q_out, p_cur, R_cur, J);

        Eigen::VectorXd err(6);
        err.head(3) = p_des - p_cur;
        err.tail(3) = rotationError(R_cur, R_des);

        if (err.norm() < IK_TOL) return true;

        static const double lambda2 = 0.01 * 0.01;
        Eigen::MatrixXd JJT = J * J.transpose();
        JJT.diagonal().array() += lambda2;
        Eigen::MatrixXd J_pinv = J.transpose() * JJT.inverse();

        Eigen::MatrixXd N = Eigen::MatrixXd::Identity(N_JOINTS, N_JOINTS) - J_pinv * J;
        Eigen::VectorXd q_null(N_JOINTS);
        for (int k = 0; k < N_JOINTS; ++k) q_null(k) = Q_MID[k] - q_out(k);

        Eigen::VectorXd dq = J_pinv * err + 0.1 * N * q_null;

        for (int k = 0; k < N_JOINTS; ++k) {
            dq(k) = normalizeAngle(dq(k));
            dq(k) = std::max(-0.15, std::min(0.15, dq(k)));
            q_out(k) += dq(k);
        }
    }
    return false;
}

// Greedy extension: walks from tree[near_idx] toward (p_target, R_target) in ScLERP
// substeps, stopping at the first collision, joint-limit violation, or IK failure.
// Returns the index of the furthest node actually added (which equals the target if
// nothing blocked it -- this doubles as RRT-Connect's "connect" primitive when called
// with an exact target pose instead of a sampled/derived one).

//  breaks long steps into smaller steps to continue going 
int localPlannerSE3(std::vector<RRTNode>& tree, int near_idx, const Eigen::Vector3d& p_target, const Eigen::Matrix3d& R_target) {
    RRTNode node_near = tree[near_idx];

    double pos_dist = (p_target - node_near.p).norm();
    double ang_dist = rotationError(node_near.R, R_target).norm();
    int steps = std::max(1, (int)std::max(pos_dist / RRT_SE3_STEP_POS, ang_dist / RRT_SE3_STEP_ANG));

    int current_parent = near_idx;
    Eigen::VectorXd q_cur = node_near.q;

    for (int k = 1; k <= steps; ++k) {
        double t = (double)k / steps;
        Eigen::Vector3d p_step; Eigen::Matrix3d R_step;
        sclerp(node_near.p, node_near.R, p_target, R_target, t, p_step, R_step);

        if (sampler->nearObstacle(p_step)) break; // cheap pre-filter, not authoritative

        Eigen::VectorXd q_next;
        if (!solveIK_RMRC(q_cur, p_step, R_step, q_next)) break;
        if (!checkJointLimits(q_next)) break;
        if (checkCollision(q_next)) break; // authoritative check, every step

        RRTNode new_node;
        new_node.q = q_next;
        Eigen::MatrixXd J_dummy;
        getPoseAndJacobianAt(q_next, new_node.p, new_node.R, J_dummy);
        new_node.parent = current_parent;
        tree.push_back(new_node);

        current_parent = tree.size() - 1;
        q_cur = q_next;
    }

    return current_parent;
}

// Nearest neighbor by position only within a single tree.
int nearestIndex(const std::vector<RRTNode>& tree, const Eigen::Vector3d& p_query) {
    int best_idx = 0;
    double best_dist = (tree[0].p - p_query).norm();
    for (size_t i = 1; i < tree.size(); ++i) {
        double dd = (tree[i].p - p_query).norm();
        if (dd < best_dist) { best_dist = dd; best_idx = i; }
    }
    return best_idx;
}

// ===========================================================================
// DOUBLE-TREE (RRT-CONNECT) PLANNER
// ===========================================================================
struct ConnectResult {
    bool success = false;
    std::vector<Eigen::VectorXd> path;
    Eigen::Vector3d connect_point = Eigen::Vector3d::Zero();
};

ConnectResult planSE3RRTConnect(const Eigen::VectorXd& q_start, const Eigen::VectorXd& q_goal_seed,
                                 const Eigen::Vector3d& p_goal_desired, const Eigen::Matrix3d& R_goal_desired) {
    std::cout << "=== Task-Space SE(3) RRT-Connect, screw-coupled sampling ===\n";

    ConnectResult result;

    std::vector<RRTNode> treeStart, treeGoal;

    // at current joint configuration 
    RRTNode root_start;
    root_start.q = q_start;
    Eigen::MatrixXd J_dummy;
    getPoseAndJacobianAt(q_start, root_start.p, root_start.R, J_dummy);
    root_start.parent = -1;
    treeStart.push_back(root_start);

    // tree that is not growing becomes the other tree
    RRTNode root_goal;
    root_goal.q = q_goal_seed;
    getPoseAndJacobianAt(q_goal_seed, root_goal.p, root_goal.R, J_dummy);
    root_goal.parent = -1;
    treeGoal.push_back(root_goal);

    std::cout << "  Goal tree rooted at FK pose (" << root_goal.p.transpose()
              << ") vs desired (" << p_goal_desired.transpose() << ")\n";

    bool grow_from_start = true;
    int connect_idx_start = -1, connect_idx_goal = -1;

    for (int it = 0; it < RRT_MAX_ITERS; ++it) {
        std::vector<RRTNode>& grow_tree  = grow_from_start ? treeStart : treeGoal;
        std::vector<RRTNode>& other_tree = grow_from_start ? treeGoal  : treeStart;
        const RRTNode& far_anchor = other_tree[0]; // fixed root pose of the tree we're growing toward

        Eigen::Vector3d p_rand;
        bool have_sample = true;
        double roll = uni01(rng);
        if (roll < RRT_GOAL_BIAS) {
            p_rand = far_anchor.p;
        } else if (roll < RRT_GOAL_BIAS + RRT_OBS_BIAS) {
            have_sample = sampler->sampleNearObstacle(p_rand);
        } else {
            have_sample = sampler->sampleCandidate(p_rand);
        }
        if (!have_sample) continue;

        int near_idx = nearestIndex(grow_tree, p_rand);
        Eigen::Matrix3d R_rand = coupledOrientation(grow_tree[near_idx].p, grow_tree[near_idx].R,
                                                      far_anchor.p, far_anchor.R, p_rand);

        int reached_idx = localPlannerSE3(grow_tree, near_idx, p_rand, R_rand);

        // Greedy connect attempt: the other tree tries to reach straight for wherever
        // grow_tree just ended up.
        const Eigen::Vector3d& p_reach = grow_tree[reached_idx].p;
        const Eigen::Matrix3d& R_reach = grow_tree[reached_idx].R;
        int other_near = nearestIndex(other_tree, p_reach);
        int other_reached = localPlannerSE3(other_tree, other_near, p_reach, R_reach);

        double p_err = (other_tree[other_reached].p - p_reach).norm();
        double r_err = rotationError(other_tree[other_reached].R, R_reach).norm();

        if (p_err < CONNECT_TOL_POS && r_err < CONNECT_TOL_ANG) {
            if (grow_from_start) { connect_idx_start = reached_idx; connect_idx_goal = other_reached; }
            else                 { connect_idx_goal  = reached_idx; connect_idx_start = other_reached; }
            result.connect_point = p_reach;
            std::cout << "  [SUCCESS] Trees connected at iteration " << it
                      << " | start-tree nodes: " << treeStart.size()
                      << " | goal-tree nodes: " << treeGoal.size() << "\n";
            break;
        }

        if (it % 500 == 0) {
            std::cout << "  Iter " << it << " | start-tree: " << treeStart.size()
                      << " | goal-tree: " << treeGoal.size() << "\n";
        }

        grow_from_start = !grow_from_start; // alternate which tree extends
    }

    if (connect_idx_start < 0 || connect_idx_goal < 0) {
        std::cerr << "  [FAILURE] RRT-Connect failed to join the trees within iteration limit.\n";
        return result;
    }

    // start -> ... -> connect point (reverse the backtrack)
    std::vector<Eigen::VectorXd> path_from_start;
    for (int cur = connect_idx_start; cur >= 0; cur = treeStart[cur].parent)
        path_from_start.push_back(treeStart[cur].q);
    std::reverse(path_from_start.begin(), path_from_start.end());

    // connect point -> ... -> goal (this is already in that order via the backtrack)
    std::vector<Eigen::VectorXd> path_to_goal;
    for (int cur = connect_idx_goal; cur >= 0; cur = treeGoal[cur].parent)
        path_to_goal.push_back(treeGoal[cur].q);

    result.path = path_from_start;
    result.path.insert(result.path.end(), path_to_goal.begin(), path_to_goal.end());
    result.success = true;
    return result;
}


// Flattens out massive 2*PI numerical jumps in the raw trajectory
void unwrapPath(std::vector<Eigen::VectorXd>& path) {
    if (path.empty()) return;
    for (size_t i = 1; i < path.size(); ++i) {
        for (int j = 0; j < N_JOINTS; ++j) {
            double diff = path[i](j) - path[i-1](j);
            // If the jump is massive, unwrap it by shifting it 360 degrees (2*PI)
            while (diff > M_PI) {
                path[i](j) -= 2.0 * M_PI;
                diff -= 2.0 * M_PI;
            }
            while (diff < -M_PI) {
                path[i](j) += 2.0 * M_PI;
                diff += 2.0 * M_PI;
            }
        }
    }
}
// Generate smooth dense path using simple linear interpolation 
// (Safe to use now that the path is unwrapped)
std::vector<Eigen::VectorXd> smoothPath(const std::vector<Eigen::VectorXd>& raw_path, int sub_steps = 10) {
    std::vector<Eigen::VectorXd> dense_path;
    if (raw_path.empty()) return dense_path;

    for (size_t i = 0; i < raw_path.size() - 1; ++i) {
        Eigen::VectorXd q1 = raw_path[i];
        Eigen::VectorXd q2 = raw_path[i+1];
        for (int s = 0; s < sub_steps; ++s) {
            double alpha = (double)s / sub_steps;
            dense_path.push_back((1.0 - alpha) * q1 + alpha * q2);
        }
    }
    dense_path.push_back(raw_path.back());
    return dense_path;
}
// ===========================================================================
// VISUALIZATION: pose markers
// ===========================================================================
void renderPoseMarker(mjvScene* sc, const Eigen::Vector3d& pos, const Eigen::Matrix3d& R,
                       const float rgba[4], double size = 0.03) {
    if (sc->ngeom >= sc->maxgeom) return;
    mjvGeom* g = sc->geoms + sc->ngeom;
    mjv_initGeom(g, mjGEOM_BOX, nullptr, nullptr, nullptr, nullptr);

    g->pos[0] = pos.x(); g->pos[1] = pos.y(); g->pos[2] = pos.z();
    g->size[0] = size; g->size[1] = size * 0.6; g->size[2] = size * 0.35;

    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            g->mat[r*3+c] = (float)R(r, c);

    g->rgba[0] = rgba[0]; g->rgba[1] = rgba[1]; g->rgba[2] = rgba[2]; g->rgba[3] = rgba[3];
    g->category = mjCAT_DECOR;
    sc->ngeom++;
}

void renderSphereMarker(mjvScene* sc, const Eigen::Vector3d& pos, const float rgba[4], double radius = 0.02) {
    if (sc->ngeom >= sc->maxgeom) return;
    mjvGeom* g = sc->geoms + sc->ngeom;
    mjv_initGeom(g, mjGEOM_SPHERE, nullptr, nullptr, nullptr, nullptr);
    g->pos[0] = pos.x(); g->pos[1] = pos.y(); g->pos[2] = pos.z();
    g->size[0] = radius; g->size[1] = radius; g->size[2] = radius;
    g->rgba[0] = rgba[0]; g->rgba[1] = rgba[1]; g->rgba[2] = rgba[2]; g->rgba[3] = rgba[3];
    g->category = mjCAT_DECOR;
    sc->ngeom++;
}

// ===========================================================================
// MAIN ENTRY
// ===========================================================================
int main() {
    char error[1000];
    m = mj_loadXML(XML_PATH, nullptr, error, sizeof(error));
    if (!m) {
        std::cerr << "Error loading XML: " << error << std::endl;
        return 1;
    }
    d = mj_makeData(m);

    if (!glfwInit()) return 1;
    window = glfwCreateWindow(1200, 900, "Kinova Gen3 SE(3) RRT-Connect - Screw Coupled Sampling", nullptr, nullptr);
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
    cam.lookat[0] = 0.2; cam.lookat[1] = 0.0; cam.lookat[2] = 0.5;

    sampler = std::make_unique<CandidateSampler>(
        CANDIDATE_VOXEL_RES,
        Eigen::Vector3d(WS_X_MIN, WS_Y_MIN, WS_Z_MIN),
        Eigen::Vector3d(WS_X_MAX, WS_Y_MAX, WS_Z_MAX)
    );
    sampler->populateFromMuJoCo(m);

    double q_start_arr[N_JOINTS] = {0.0, 0.3, 0.0, -1.2, 0.0, 0.6, 0.0};
    for (int i = 0; i < N_JOINTS; ++i) d->qpos[i] = q_start_arr[i];
    mj_forward(m, d);

    Eigen::VectorXd q_start_vec(N_JOINTS);
    for (int i = 0; i < N_JOINTS; ++i) q_start_vec(i) = q_start_arr[i];

    Eigen::Vector3d p_goal(0.40, -0.15, 0.45);
    Eigen::Matrix3d R_goal = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();

    // --- Solve IK once to root the goal-side tree in joint space. ---
    // Try a comfortable seed first, then the start config, then a few small random
    // perturbations of Q_MID if those both fail -- the goal pose may be reachable
    // from more than one elbow configuration and a bad seed can miss all of them.
    Eigen::VectorXd q_mid_vec(N_JOINTS);
    for (int i = 0; i < N_JOINTS; ++i) q_mid_vec(i) = Q_MID[i];

    Eigen::VectorXd q_goal_seed;
    bool ik_ok = solveIK_RMRC(q_mid_vec, p_goal, R_goal, q_goal_seed);
    if (!ik_ok) ik_ok = solveIK_RMRC(q_start_vec, p_goal, R_goal, q_goal_seed);
    for (int attempt = 0; !ik_ok && attempt < 8; ++attempt) {
        Eigen::VectorXd q_perturbed = q_mid_vec;
        for (int i = 0; i < N_JOINTS; ++i) q_perturbed(i) += (uni01(rng) - 0.5) * 1.0;
        ik_ok = solveIK_RMRC(q_perturbed, p_goal, R_goal, q_goal_seed);
    }
    if (!ik_ok || !checkJointLimits(q_goal_seed) || checkCollision(q_goal_seed)) {
        std::cerr << "  [FAILURE] Could not find a valid joint configuration at the goal pose "
                     "to root the goal-side tree. Try a different seed or goal pose.\n";
        mj_deleteData(d); mj_deleteModel(m); glfwTerminate();
        return 1;
    }

    Eigen::Vector3d p_start; Eigen::Matrix3d R_start; Eigen::MatrixXd J_dummy;
    getPoseAndJacobianAt(q_start_vec, p_start, R_start, J_dummy);

    ConnectResult plan = planSE3RRTConnect(q_start_vec, q_goal_seed, p_goal, R_goal);
    if (!plan.success) {
        mj_deleteData(d); mj_deleteModel(m); glfwTerminate();
        return 1;
    }

    // 1. UNWRAP THE PATH FIRST
    unwrapPath(plan.path);

    // 2. SMOOTH IT
    auto rrt_path_smooth = smoothPath(plan.path, 12);
    std::cout << "  Dense path generated: " << rrt_path_smooth.size() << " step configurations.\n";

    const float START_RGBA[4]   = {0.15f, 0.9f, 0.3f, 0.9f};  
    const float GOAL_RGBA[4]    = {0.95f, 0.25f, 0.2f, 0.9f}; 
    const float CONNECT_RGBA[4] = {0.95f, 0.85f, 0.15f, 0.9f}; 

    std::cout << "\nReplaying motion path in MuJoCo. Press ESC to quit.\n";
    while (!glfwWindowShouldClose(window)) {
        for (const auto& q_step : rrt_path_smooth) {
            if (glfwWindowShouldClose(window)) break;

            for (int i = 0; i < N_JOINTS; ++i) d->qpos[i] = q_step[i];
            
            // 3. USE KINEMATICS, NOT FORWARD (Disables physics engine twitching during replay)
            mj_kinematics(m, d);

            int w, h; glfwGetFramebufferSize(window, &w, &h);
            mjrRect viewport = {0, 0, w, h};
            mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
            
            // ... (rest of rendering code remains the same)

            renderPoseMarker(&scn, p_start, R_start, START_RGBA);
            renderPoseMarker(&scn, p_goal, R_goal, GOAL_RGBA);
            renderSphereMarker(&scn, plan.connect_point, CONNECT_RGBA);

            mjr_render(viewport, &scn, &con);
            glfwSwapBuffers(window);
            glfwPollEvents();

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(600));
    }

    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    glfwTerminate();
    return 0;
}
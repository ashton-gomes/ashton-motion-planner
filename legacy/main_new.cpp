// -----------------------------------------------------------------------
// main_new.cpp
//
// Minimal single-node-expansion RRT for the Kinova Gen3 in MuJoCo.
// Stripped down from the RRT-Connect version: one tree, one node added
// per iteration, no obstacle-biased sampling, no bidirectional connect.
// This is meant to be a clean starting point to build back up from.
//
// Configurable:
//   MAX_NODES  - cap on nodes for a single tree
//   MAX_TREES  - how many trees get allocated. Only trees[0] is actually
//                driven toward the goal here; the rest exist so you can
//                grow this into RRT-Connect / multi-tree RRT* later
//                without changing the data model.
// -----------------------------------------------------------------------

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <Eigen/Dense>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

// ---------------------------------------------------------------------
// config -- tune these
// ---------------------------------------------------------------------
static int MAX_NODES = 4000;   // node cap per tree
static int MAX_TREES = 3;      // trees allocated (only tree 0 is grown for now)

static const int    N_JOINTS = 7;
static const char*  XML_PATH = "/home/ash_gomes/gplanner/mujoco_menagerie/kinova_gen3/gen3_obstacle.xml";
static const char*  EE_SITE  = "pinch_site";

static const double Q_MID[N_JOINTS]      = {0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57};
static const double Q_LIM_LOW[N_JOINTS]  = {-6.28, -2.24, -6.28, -2.57, -6.28, -2.09, -6.28};
static const double Q_LIM_HIGH[N_JOINTS] = { 6.28,  2.24,  6.28,  2.57,  6.28,  2.09,  6.28};

static const double WS_X_MIN = -0.2, WS_X_MAX = 0.65;
static const double WS_Y_MIN = -0.5, WS_Y_MAX = 0.5;
static const double WS_Z_MIN =  0.3, WS_Z_MAX = 0.95;

static const double RRT_GOAL_BIAS = 0.15; // chance to sample the goal exactly
// Smaller extensions create a denser path, which improves both collision
// checking resolution and the visual smoothness of the executed motion.
static const double RRT_STEP_POS  = 0.01; // single-step translation cap (m)
static const double RRT_STEP_ANG  = 0.03; // single-step rotation cap (rad)

static const double GOAL_TOL_POS = 0.02;
static const double GOAL_TOL_ANG = 0.05;

mjModel* m = nullptr;
mjData*  d = nullptr;

std::mt19937 rng(1337);
std::uniform_real_distribution<double> uni01(0.0, 1.0);

// ---------------------------------------------------------------------
// MuJoCo rendering globals + mouse/camera state
// ---------------------------------------------------------------------
mjvCamera   cam;
mjvOption   opt;
mjvScene    scn;
mjrContext  con;
GLFWwindow* window = nullptr;

bool   button_left = false, button_middle = false, button_right = false;
double lastx = 0, lasty = 0;

static const double ANIM_FRAME_DT = 0.30; // seconds to traverse one path segment
static const double POSE_AXIS_LENGTH = 0.10;
static const double POSE_AXIS_RADIUS = 0.006;

// ---------------------------------------------------------------------
// node / tree
// ---------------------------------------------------------------------
struct RRTNode {
    Eigen::VectorXd q;
    Eigen::Vector3d p;
    Eigen::Matrix3d R;
    int parent;
};
using Tree = std::vector<RRTNode>;

// ---------------------------------------------------------------------
// math helpers
// ---------------------------------------------------------------------
inline double normalizeAngle(double a) {
    while (a > M_PI)  a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

Eigen::Vector3d rotationError(const Eigen::Matrix3d& R_cur, const Eigen::Matrix3d& R_des) {
    Eigen::Matrix3d R_err = R_des * R_cur.transpose();
    double c = std::clamp(0.5 * (R_err.trace() - 1.0), -1.0, 1.0);
    double theta = std::acos(c);
    if (std::abs(theta) < 1e-6) return Eigen::Vector3d::Zero();
    double f = theta / (2.0 * std::sin(theta));
    return f * Eigen::Vector3d(R_err(2,1) - R_err(1,2), R_err(0,2) - R_err(2,0), R_err(1,0) - R_err(0,1));
}

void sclerp(const Eigen::Vector3d& p1, const Eigen::Matrix3d& R1,
            const Eigen::Vector3d& p2, const Eigen::Matrix3d& R2,
            double t, Eigen::Vector3d& p_out, Eigen::Matrix3d& R_out) {
    p_out = (1.0 - t) * p1 + t * p2;
    Eigen::Quaterniond q1(R1), q2(R2);
    R_out = q1.slerp(t, q2).toRotationMatrix();
}

// alpha = how far p_rand sits along [p_near -> p_goal]; the same alpha
// slerps [R_near -> R_goal]. Single-tree version, so the "anchor" is
// always the fixed goal pose (no other-tree swapping like in RRT-Connect).
Eigen::Matrix3d coupledOrientation(const Eigen::Vector3d& p_near, const Eigen::Matrix3d& R_near,
                                    const Eigen::Vector3d& p_goal, const Eigen::Matrix3d& R_goal,
                                    const Eigen::Vector3d& p_rand) {
    double d_total = (p_goal - p_near).norm();
    double alpha;
    if (d_total < 1e-6) {
        alpha = 1.0;
    } else {
        double d_partial = (p_rand - p_near).norm();
        alpha = std::clamp(d_partial / d_total, 0.0, 1.0);
    }
    Eigen::Quaterniond q_near(R_near), q_goal(R_goal);
    return q_near.slerp(alpha, q_goal).toRotationMatrix();
}

// ---------------------------------------------------------------------
// GLFW callbacks -- orbit camera with left drag, pan with right drag,
// zoom with middle drag or scroll, ESC to close.
// ---------------------------------------------------------------------
void keyCallback(GLFWwindow* w, int key, int, int action, int) {
    if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE) glfwSetWindowShouldClose(w, GLFW_TRUE);
}
void mouseButtonCallback(GLFWwindow* w, int, int, int) {
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
void scrollCallback(GLFWwindow*, double, double yoffset) {
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &cam);
}

// Appends an RGB coordinate frame for an end-effector pose.  Cylinders are
// used instead of position-only spheres so both location and orientation of
// the arm are visible.  Must be called after mjv_updateScene each frame.
void addPoseMarker(mjvScene* scene, const Eigen::Vector3d& pos, const Eigen::Matrix3d& R) {
    static const float AXIS_RGBA[3][4] = {
        {0.90f, 0.05f, 0.05f, 1.0f}, // X: red
        {0.05f, 0.70f, 0.05f, 1.0f}, // Y: green
        {0.05f, 0.20f, 0.90f, 1.0f}  // Z: blue
    };
    const Eigen::Matrix3d rotate_z_to_x =
        Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()).toRotationMatrix();
    const Eigen::Matrix3d rotate_z_to_y =
        Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()).toRotationMatrix();
    const Eigen::Matrix3d alignments[3] = {R * rotate_z_to_x, R * rotate_z_to_y, R};

    for (int axis = 0; axis < 3 && scene->ngeom < scene->maxgeom; ++axis) {
        const Eigen::Vector3d direction = R.col(axis);
        const Eigen::Vector3d center = pos + 0.5 * POSE_AXIS_LENGTH * direction;
        mjtNum size[3] = {POSE_AXIS_RADIUS, 0.5 * POSE_AXIS_LENGTH, 0};
        mjtNum p[3] = {center.x(), center.y(), center.z()};
        mjtNum mat[9];
        for (int row = 0; row < 3; ++row)
            for (int col = 0; col < 3; ++col)
                mat[3 * row + col] = alignments[axis](row, col);
        mjv_initGeom(&scene->geoms[scene->ngeom], mjGEOM_CYLINDER, size, p, mat, AXIS_RGBA[axis]);
        scene->ngeom++;
    }
}

// ---------------------------------------------------------------------
// kinematics / collision
// ---------------------------------------------------------------------
Eigen::Vector3d getEEPosition() {
    int id = mj_name2id(m, mjOBJ_SITE, EE_SITE);
    return Eigen::Vector3d(d->site_xpos[3*id], d->site_xpos[3*id+1], d->site_xpos[3*id+2]);
}

Eigen::Matrix3d getEERotation() {
    int id = mj_name2id(m, mjOBJ_SITE, EE_SITE);
    Eigen::Matrix3d R;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) R(i,j) = d->site_xmat[9*id + 3*i+j];
    return R;
}

Eigen::MatrixXd getEEJacobian6() {
    int id = mj_name2id(m, mjOBJ_SITE, EE_SITE);
    std::vector<double> jacp(3 * m->nv, 0.0), jacr(3 * m->nv, 0.0);
    mj_jacSite(m, d, jacp.data(), jacr.data(), id);
    Eigen::MatrixXd J(6, N_JOINTS);
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < N_JOINTS; ++c) {
            J(r, c)   = jacp[r*m->nv+c];
            J(r+3, c) = jacr[r*m->nv+c];
        }
    }
    return J;
}

void getPoseAndJacobianAt(const Eigen::VectorXd& q, Eigen::Vector3d& p, Eigen::Matrix3d& R, Eigen::MatrixXd& J) {
    std::vector<double> save(m->nq);
    for (int i = 0; i < m->nq; ++i) save[i] = d->qpos[i];

    for (int i = 0; i < N_JOINTS; ++i) d->qpos[i] = q(i);
    mj_kinematics(m, d);
    mj_comPos(m, d);

    p = getEEPosition();
    R = getEERotation();
    J = getEEJacobian6();

    for (int i = 0; i < m->nq; ++i) d->qpos[i] = save[i];
    mj_kinematics(m, d);
    mj_comPos(m, d);
}

bool checkCollision(const Eigen::VectorXd& q) {
    std::vector<double> save(m->nq);
    for (int i = 0; i < m->nq; ++i) save[i] = d->qpos[i];

    for (int i = 0; i < N_JOINTS; ++i) d->qpos[i] = q(i);
    mj_kinematics(m, d);
    mj_collision(m, d);

    bool collide = false;
    for (int i = 0; i < d->ncon; ++i) {
        int g1 = d->contact[i].geom1, g2 = d->contact[i].geom2;
        std::string n1 = (m->name_geomadr[g1] >= 0) ? std::string(m->names + m->name_geomadr[g1]) : "";
        std::string n2 = (m->name_geomadr[g2] >= 0) ? std::string(m->names + m->name_geomadr[g2]) : "";
        if (n1.find("obs_") == 0 || n2.find("obs_") == 0 || n1 == "table" || n2 == "table") {
            collide = true;
            break;
        }
    }

    for (int i = 0; i < m->nq; ++i) d->qpos[i] = save[i];
    mj_kinematics(m, d);
    return collide;
}

bool checkJointLimits(const Eigen::VectorXd& q) {
    for (int i = 0; i < N_JOINTS; ++i)
        if (q(i) < Q_LIM_LOW[i] || q(i) > Q_LIM_HIGH[i]) return false;
    return true;
}

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
            dq(k) = std::clamp(dq(k), -0.15, 0.15);
            q_out(k) += dq(k);
        }
    }
    return false;
}

// ---------------------------------------------------------------------
// sampler -- plain uniform sampling in the workspace box. No obstacle
// biasing, no lattice snap. Add those back once this loop is solid.
// ---------------------------------------------------------------------
Eigen::Vector3d sampleWorkspacePoint() {
    return Eigen::Vector3d(
        WS_X_MIN + uni01(rng) * (WS_X_MAX - WS_X_MIN),
        WS_Y_MIN + uni01(rng) * (WS_Y_MAX - WS_Y_MIN),
        WS_Z_MIN + uni01(rng) * (WS_Z_MAX - WS_Z_MIN)
    );
}

// ---------------------------------------------------------------------
// nearest neighbor, position only
// ---------------------------------------------------------------------
int nearestIndex(const Tree& tree, const Eigen::Vector3d& p_query) {
    int best = 0;
    double bestDist = (tree[0].p - p_query).norm();
    for (size_t i = 1; i < tree.size(); ++i) {
        double dd = (tree[i].p - p_query).norm();
        if (dd < bestDist) { bestDist = dd; best = (int)i; }
    }
    return best;
}

// ---------------------------------------------------------------------
// single-node expansion: exactly ONE step from tree[near_idx] toward
// (p_target, R_target), capped at RRT_STEP_POS / RRT_STEP_ANG. This is
// the whole difference from the old localPlannerSE3: that walked in a
// loop until blocked (RRT-Connect's greedy "connect"); this adds at
// most one node per call, which is what a basic RRT extend step does.
// ---------------------------------------------------------------------
bool expandOneStep(Tree& tree, int near_idx, const Eigen::Vector3d& p_target, const Eigen::Matrix3d& R_target) {
    const RRTNode& near = tree[near_idx];

    double pos_dist = (p_target - near.p).norm();
    double t_pos = (pos_dist < 1e-9) ? 1.0 : std::min(1.0, RRT_STEP_POS / pos_dist);

    double ang_dist = rotationError(near.R, R_target).norm();
    double t_ang = (ang_dist < 1e-9) ? 1.0 : std::min(1.0, RRT_STEP_ANG / ang_dist);

    double t = std::min(t_pos, t_ang); // whichever cap binds first

    Eigen::Vector3d p_step; Eigen::Matrix3d R_step;
    sclerp(near.p, near.R, p_target, R_target, t, p_step, R_step);

    Eigen::VectorXd q_next;
    if (!solveIK_RMRC(near.q, p_step, R_step, q_next)) return false;
    if (!checkJointLimits(q_next)) return false;
    if (checkCollision(q_next)) return false;

    RRTNode node;
    node.q = q_next;
    Eigen::MatrixXd J_dummy;
    getPoseAndJacobianAt(q_next, node.p, node.R, J_dummy);
    node.parent = near_idx;
    tree.push_back(node);
    return true;
}

// ---------------------------------------------------------------------
// grow a single tree from its existing root toward (p_goal, R_goal).
// One sample -> one nearest lookup -> one expandOneStep call -> repeat,
// up to MAX_NODES. Returns true and fills goal_idx if goal tolerance
// is reached.
// ---------------------------------------------------------------------
bool growTree(Tree& tree, const Eigen::Vector3d& p_goal, const Eigen::Matrix3d& R_goal, int& goal_idx) {
    while ((int)tree.size() < MAX_NODES) {
        Eigen::Vector3d p_rand;
        Eigen::Matrix3d R_rand;
        bool goal_sample = uni01(rng) < RRT_GOAL_BIAS;

        if (goal_sample) {
            p_rand = p_goal;
            R_rand = R_goal;
        } else {
            p_rand = sampleWorkspacePoint();
        }

        int near_idx = nearestIndex(tree, p_rand);
        if (!goal_sample) {
            R_rand = coupledOrientation(tree[near_idx].p, tree[near_idx].R, p_goal, R_goal, p_rand);
        }

        if (expandOneStep(tree, near_idx, p_rand, R_rand)) {
            const RRTNode& latest = tree.back();
            double dp = (latest.p - p_goal).norm();
            double da = rotationError(latest.R, R_goal).norm();
            if (dp < GOAL_TOL_POS && da < GOAL_TOL_ANG) {
                goal_idx = (int)tree.size() - 1;
                return true;
            }
        }
    }
    return false;
}

// ---------------------------------------------------------------------
// main
// ---------------------------------------------------------------------
int main() {
    char error[1000] = "";
    m = mj_loadXML(XML_PATH, nullptr, error, 1000);
    if (!m) {
        std::cerr << "Failed to load model: " << error << "\n";
        return 1;
    }
    d = mj_makeData(m);

    // trees[0] is the only one grown right now. Bumping MAX_TREES just
    // reserves the slots -- wire up a second tree (e.g. rooted at the
    // goal, swapping nearest-tree each iteration) to get RRT-Connect back.
    std::vector<Tree> trees(MAX_TREES);

    Eigen::VectorXd q_start(N_JOINTS);
    for (int i = 0; i < N_JOINTS; ++i) q_start(i) = Q_MID[i];

    RRTNode root;
    root.q = q_start;
    Eigen::MatrixXd J_dummy;
    getPoseAndJacobianAt(q_start, root.p, root.R, J_dummy);
    root.parent = -1;
    trees[0].push_back(root);

    // hardcoded goal -- swap for whatever target pose you're testing
    Eigen::Vector3d p_goal(-0.42, -0.25, 0.45);
    Eigen::Matrix3d R_goal = root.R; // same orientation as start, translation only
    // Pitch end-effector DOWN by 45 degrees (around local Y-axis)
    // Eigen::Matrix3d R_goal = root.R * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()).toRotationMatrix();

    int goal_idx = -1;
    bool reached = growTree(trees[0], p_goal, R_goal, goal_idx);

    std::vector<int> path;
    if (reached) {
        int idx = goal_idx;
        while (idx != -1) {
            path.push_back(idx);
            idx = trees[0][idx].parent;
        }
        std::reverse(path.begin(), path.end());
        std::cout << "Reached goal. Nodes in tree: " << trees[0].size()
                  << ", path length: " << path.size() << "\n";
    } else {
        std::cout << "Failed to reach goal within " << MAX_NODES
                  << " nodes. Final tree size: " << trees[0].size()
                  << " (showing start/goal only, no path to animate)\n";
    }

    // -------------------------------------------------------------
    // GLFW + MuJoCo rendering setup
    // -------------------------------------------------------------
    if (!glfwInit()) {
        std::cerr << "Could not initialize GLFW\n";
        mj_deleteData(d);
        mj_deleteModel(m);
        return 1;
    }
    window = glfwCreateWindow(1200, 900, "RRT Path Animation", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // extra maxgeom headroom over the model's own geom count for the
    // start/goal markers appended each frame
    mjv_makeScene(m, &scn, m->ngeom + 50);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    glfwSetKeyCallback(window, keyCallback);
    glfwSetCursorPosCallback(window, mouseMoveCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetScrollCallback(window, scrollCallback);

    cam.lookat[0] = p_goal.x(); cam.lookat[1] = p_goal.y(); cam.lookat[2] = p_goal.z();
    cam.distance  = 1.6;
    cam.azimuth   = 120;
    cam.elevation = -20;

    // Travel back and forth so the final pose connects continuously to the
    // next frame; looping directly from goal back to start would teleport.
    double path_position = 0.0;
    int playback_direction = 1;
    double last_frame_time = glfwGetTime();

    while (!glfwWindowShouldClose(window)) {
        if (reached && path.size() > 1) {
            double now = glfwGetTime();
            // Advance from real elapsed time, not rendering speed.  Keeping the
            // remainder avoids a visible pause when a frame takes longer than
            // one segment duration.
            double dt = std::min(now - last_frame_time, 0.1);
            last_frame_time = now;
            path_position += playback_direction * dt / ANIM_FRAME_DT;
            const double last_position = static_cast<double>(path.size() - 1);
            while (path_position > last_position || path_position < 0.0) {
                if (path_position > last_position) {
                    path_position = 2.0 * last_position - path_position;
                    playback_direction = -1;
                } else {
                    path_position = -path_position;
                    playback_direction = 1;
                }
            }

            const size_t segment_idx = std::min(
                static_cast<size_t>(path_position), path.size() - 2);
            const RRTNode& from = trees[0][path[segment_idx]];
            const RRTNode& to = trees[0][path[segment_idx + 1]];
            // smoothstep gives zero velocity at waypoint boundaries, removing
            // the abrupt direction changes from the original waypoint snapping.
            double segment_progress = path_position - segment_idx;
            double t = segment_progress * segment_progress * (3.0 - 2.0 * segment_progress);
            for (int i = 0; i < N_JOINTS; ++i)
                d->qpos[i] = (1.0 - t) * from.q(i) + t * to.q(i);
            mj_forward(m, d);
        } else if (reached && path.size() == 1) {
            for (int i = 0; i < N_JOINTS; ++i) d->qpos[i] = root.q(i);
            mj_forward(m, d);
        }

        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
        addPoseMarker(&scn, root.p, root.R);  // arm end-effector pose at start
        addPoseMarker(&scn, p_goal, R_goal);  // arm end-effector pose at goal

        mjr_render(viewport, &scn, &con);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    glfwDestroyWindow(window);
    glfwTerminate();

    mj_deleteData(d);
    mj_deleteModel(m);
    return 0;
}

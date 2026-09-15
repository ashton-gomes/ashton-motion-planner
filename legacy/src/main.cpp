#include <algorithm>
#include <iostream>

#include "config.hpp"
#include "kinematics.hpp"
#include "planner.hpp"
#include "sampler.hpp"
#include "viewer.hpp"

namespace {
constexpr double kAnimationSegmentSeconds = 0.30;
}

int main() {
    char error[1000] = "";
    mjModel* m = mj_loadXML(config::XML_PATH, nullptr, error, sizeof(error));
    if (!m) {
        std::cerr << "Failed to load model: " << error << '\n';
        return 1;
    }
    mjData* d = mj_makeData(m);
    if (!d) {
        std::cerr << "Failed to allocate MuJoCo data.\n";
        mj_deleteModel(m);
        return 1;
    }

    CandidateSampler sampler(
        Eigen::Vector3d(config::WS_X_MIN, config::WS_Y_MIN, config::WS_Z_MIN),
        Eigen::Vector3d(config::WS_X_MAX, config::WS_Y_MAX, config::WS_Z_MAX));

    const Eigen::VectorXd q_start = config::Q_MID;
    Eigen::Vector3d p_start;
    Eigen::Matrix3d R_start;
    Eigen::MatrixXd unused_jacobian;
    getPoseAndJacobianAt(m, d, q_start, p_start, R_start, unused_jacobian);

    // Keep the target and orientation used by main_new.cpp.
    const Eigen::Vector3d p_goal(-0.42, -0.25, 0.45);
    const Eigen::Matrix3d R_goal = R_start;
    SingleTreePlanResult plan = planSingleTreeRRT(m, d, sampler, q_start, p_goal, R_goal);

    if (plan.reached) {
        std::cout << "Reached goal. Nodes in tree: " << plan.tree.size()
                  << ", path length: " << plan.path.size() << '\n';
    } else {
        std::cout << "Failed to reach goal within " << config::RRT_MAX_NODES
                  << " nodes. Final tree size: " << plan.tree.size()
                  << " (showing start/goal only, no path to animate)\n";
    }

    MuJoCoViewer viewer;
    if (!viewer.init(m, 1200, 900, "RRT Path Animation")) {
        mj_deleteData(d);
        mj_deleteModel(m);
        return 1;
    }
    viewer.cam.lookat[0] = p_goal.x();
    viewer.cam.lookat[1] = p_goal.y();
    viewer.cam.lookat[2] = p_goal.z();
    viewer.cam.distance = 1.6;
    viewer.cam.azimuth = 120;
    viewer.cam.elevation = -20;

    double path_position = 0.0;
    int playback_direction = 1;
    double last_frame_time = glfwGetTime();

    while (!glfwWindowShouldClose(viewer.window)) {
        if (plan.reached && plan.path.size() > 1) {
            const double now = glfwGetTime();
            const double dt = std::min(now - last_frame_time, 0.1);
            last_frame_time = now;
            path_position += playback_direction * dt / kAnimationSegmentSeconds;

            const double last_position = static_cast<double>(plan.path.size() - 1);
            while (path_position > last_position || path_position < 0.0) {
                if (path_position > last_position) {
                    path_position = 2.0 * last_position - path_position;
                    playback_direction = -1;
                } else {
                    path_position = -path_position;
                    playback_direction = 1;
                }
            }

            const size_t segment = std::min(static_cast<size_t>(path_position), plan.path.size() - 2);
            const RRTNode& from = plan.tree[plan.path[segment]];
            const RRTNode& to = plan.tree[plan.path[segment + 1]];
            const double linear_t = path_position - segment;
            const double t = linear_t * linear_t * (3.0 - 2.0 * linear_t);
            for (int i = 0; i < config::N_JOINTS; ++i) {
                d->qpos[i] = (1.0 - t) * from.q(i) + t * to.q(i);
            }
            mj_forward(m, d);
        } else {
            for (int i = 0; i < config::N_JOINTS; ++i) d->qpos[i] = q_start(i);
            mj_forward(m, d);
        }

        int width, height;
        glfwGetFramebufferSize(viewer.window, &width, &height);
        if (width > 0 && height > 0) {
            const mjrRect viewport = {0, 0, width, height};
            mjv_updateScene(m, d, &viewer.opt, nullptr, &viewer.cam, mjCAT_ALL, &viewer.scn);
            MuJoCoViewer::renderPoseMarker(&viewer.scn, p_start, R_start);
            MuJoCoViewer::renderPoseMarker(&viewer.scn, p_goal, R_goal);
            mjr_render(viewport, &viewer.scn, &viewer.con);
            glfwSwapBuffers(viewer.window);
        }
        glfwPollEvents();
    }

    viewer.close(m, d);
    return 0;
}

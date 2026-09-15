#pragma once
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <Eigen/Dense>

struct MuJoCoViewer {
    // The model is needed by mjv_moveCamera for zooming and panning.
    // It is borrowed; the caller retains ownership until close().
    const mjModel* model = nullptr;
    GLFWwindow* window = nullptr;
    mjvCamera cam{};
    mjvOption opt{};
    mjvScene scn{};
    mjrContext con{};
    
    bool button_left = false;
    bool button_middle = false;
    bool button_right = false;
    double lastx = 0;
    double lasty = 0;

    bool glfw_initialized = false;
    bool scene_initialized = false;
    bool context_initialized = false;

    bool init(mjModel* m, int width = 1200, int height = 900, const char* title = "MuJoCo Viewer");
    void updateAndRender(mjModel* m, mjData* d);
    void close(mjModel* m, mjData* d);

    // Appends RGB cylinders representing a pose's local X, Y, and Z axes.
    static void renderPoseMarker(mjvScene* sc, const Eigen::Vector3d& pos, const Eigen::Matrix3d& R,
                                 double axis_length = 0.10, double axis_radius = 0.006);
};

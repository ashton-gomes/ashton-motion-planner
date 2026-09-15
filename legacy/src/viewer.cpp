#include "viewer.hpp"
#include <iostream>

static void keyCallback(GLFWwindow* w, int key, int, int action, int) {
    if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE) glfwSetWindowShouldClose(w, GLFW_TRUE);
}

static void mouseButtonCallback(GLFWwindow* w, int button, int action, int) {
    auto* v = static_cast<MuJoCoViewer*>(glfwGetWindowUserPointer(w));
    if (!v) return;
    v->button_left   = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT)   == GLFW_PRESS);
    v->button_middle = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    v->button_right  = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_RIGHT)  == GLFW_PRESS);
    glfwGetCursorPos(w, &v->lastx, &v->lasty);
}

static void mouseMoveCallback(GLFWwindow* w, double xpos, double ypos) {
    auto* v = static_cast<MuJoCoViewer*>(glfwGetWindowUserPointer(w));
    if (!v || (!v->button_left && !v->button_middle && !v->button_right)) return;

    double dx = xpos - v->lastx, dy = ypos - v->lasty;
    v->lastx = xpos; v->lasty = ypos;
    int width, height; glfwGetWindowSize(w, &width, &height);
    
    mjtMouse action = v->button_right ? mjMOUSE_MOVE_V : (v->button_left ? mjMOUSE_ROTATE_V : mjMOUSE_ZOOM);
    if (height > 0) {
        mjv_moveCamera(v->model, action, dx / height, dy / height, &v->cam);
    }
}

static void scrollCallback(GLFWwindow* w, double, double yoffset) {
    auto* v = static_cast<MuJoCoViewer*>(glfwGetWindowUserPointer(w));
    if (v && v->model) {
        mjv_moveCamera(v->model, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &v->cam);
    }
}

bool MuJoCoViewer::init(mjModel* m, int width, int height, const char* title) {
    if (!m) {
        std::cerr << "[ERROR] Cannot initialize viewer without a MuJoCo model.\n";
        return false;
    }
    if (window) {
        std::cerr << "[ERROR] Viewer is already initialized.\n";
        return false;
    }

    if (!glfwInit()) {
        std::cerr << "[ERROR] Could not initialize GLFW.\n";
        return false;
    }
    glfw_initialized = true;

    // Reset hints to default compatibility profile (matches your original code)
    glfwDefaultWindowHints();

    window = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (!window) {
        std::cerr << "[ERROR] Could not create GLFW window.\n";
        glfwTerminate();
        glfw_initialized = false;
        return false;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glfwSetWindowUserPointer(window, this);
    glfwSetKeyCallback(window, keyCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetCursorPosCallback(window, mouseMoveCallback);
    glfwSetScrollCallback(window, scrollCallback);

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    // mjv_makeScene/mjr_makeContext first free any existing resources.  Their
    // structs must therefore be default-initialized before their first use.
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, m->ngeom + 64);
    scene_initialized = true;
    mjr_makeContext(m, &con, mjFONTSCALE_150);
    context_initialized = true;
    model = m;

    cam.distance = 2.0;
    cam.azimuth  = 135;
    cam.elevation = -25;
    cam.lookat[0] = 0.2; 
    cam.lookat[1] = 0.0; 
    cam.lookat[2] = 0.5;

    return true;
}

void MuJoCoViewer::updateAndRender(mjModel* m, mjData* d) {
    if (!window || !m || !d) return;

    glfwMakeContextCurrent(window);
    int w, h;
    glfwGetFramebufferSize(window, &w, &h);
    if (w <= 0 || h <= 0) {
        glfwPollEvents();
        return;
    }
    mjrRect viewport = {0, 0, w, h};
    mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);
    glfwSwapBuffers(window);
    glfwPollEvents();
}

void MuJoCoViewer::close(mjModel* m, mjData* d) {
    if (window) glfwMakeContextCurrent(window);
    if (scene_initialized) {
        mjv_freeScene(&scn);
        scene_initialized = false;
    }
    if (context_initialized) {
        mjr_freeContext(&con);
        context_initialized = false;
    }
    if (window) {
        glfwDestroyWindow(window);
        window = nullptr;
    }
    if (glfw_initialized) {
        glfwTerminate();
        glfw_initialized = false;
    }
    model = nullptr;
    if (d) mj_deleteData(d);
    if (m) mj_deleteModel(m);
}

void MuJoCoViewer::renderPoseMarker(mjvScene* sc, const Eigen::Vector3d& pos, const Eigen::Matrix3d& R,
                                    double axis_length, double axis_radius) {
    static const float kAxisRgba[3][4] = {
        {0.90f, 0.05f, 0.05f, 1.0f},
        {0.05f, 0.70f, 0.05f, 1.0f},
        {0.05f, 0.20f, 0.90f, 1.0f},
    };
    const Eigen::Matrix3d z_to_x =
        Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()).toRotationMatrix();
    const Eigen::Matrix3d z_to_y =
        Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()).toRotationMatrix();
    const Eigen::Matrix3d alignments[3] = {R * z_to_x, R * z_to_y, R};

    for (int axis = 0; axis < 3 && sc->ngeom < sc->maxgeom; ++axis) {
        const Eigen::Vector3d center = pos + 0.5 * axis_length * R.col(axis);
        const mjtNum size[3] = {axis_radius, 0.5 * axis_length, 0};
        const mjtNum point[3] = {center.x(), center.y(), center.z()};
        mjtNum rotation[9];
        for (int row = 0; row < 3; ++row)
            for (int column = 0; column < 3; ++column)
                rotation[3 * row + column] = alignments[axis](row, column);
        mjv_initGeom(&sc->geoms[sc->ngeom], mjGEOM_CYLINDER, size, point, rotation, kAxisRgba[axis]);
        sc->ngeom++;
    }
}

#include "viewer.hpp"
#include <Eigen/Geometry>
#include <cstdio>
#include <memory>

int MujocoViewer::play(const std::string& scene_path,
                       const std::vector<Eigen::VectorXd>& trajectory,
                       const Eigen::Matrix<double, 7, 1>& start,
                       const Eigen::Matrix<double, 7, 1>& end) {
    if (trajectory.empty()) return 1;
    char error[1000]{};
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
        mj_loadXML(scene_path.c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
    if (!model) {
        std::cerr << error << '\n';
        return 1;
    }
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
    if (!data) return 1;
    for (const auto& joints : trajectory) {
        if (joints.size() != model->nq || !joints.allFinite()) {
            std::cerr << "Invalid viewer joint trajectory.\n";
            return 1;
        }
    }
    // Destroy the viewer before the model/data it references.
    MujocoViewer viewer(model.get(), data.get(), "ScLERP + RMRC | X: red, Y: green, Z: blue");
    if (!viewer.is_open()) return 1;
    viewer.pose_markers_ = {start, end};
    const double start_time = glfwGetTime();
    while (viewer.is_open()) {
        size_t frame = static_cast<size_t>((glfwGetTime() - start_time) * 60.0)
                       % trajectory.size();
        viewer.set_qpos(trajectory[frame]);
        viewer.render();
    }
    return 0;
}

void MujocoViewer::draw_pose_marker(const Eigen::Matrix<double, 7, 1>& pose,
                                    const char* label) {
    if (scn_.ngeom + 4 > scn_.maxgeom) return;
    const Eigen::Vector3d position = pose.head<3>();
    const Eigen::Matrix3d rotation = Eigen::Quaterniond(
        pose[3], pose[4], pose[5], pose[6]).normalized().toRotationMatrix();
    const float white[4] = {1, 1, 1, 1};
    const mjtNum size[3] = {0.008, 0.008, 0.008};
    mjvGeom* origin = &scn_.geoms[scn_.ngeom++];
    mjv_initGeom(origin, mjGEOM_SPHERE, size, position.data(), nullptr, white);
    std::snprintf(origin->label, sizeof(origin->label), "%s", label);

    const float colors[3][4] = {{1, 0.1f, 0.1f, 1}, {0.1f, 1, 0.1f, 1}, {0.1f, 0.3f, 1, 1}};
    for (int axis = 0; axis < 3; ++axis) {
        const Eigen::Vector3d tip = position + 0.08 * rotation.col(axis);
        mjvGeom* arrow = &scn_.geoms[scn_.ngeom++];
        mjv_initGeom(arrow, mjGEOM_ARROW, nullptr, nullptr, nullptr, colors[axis]);
        mjv_connector(arrow, mjGEOM_ARROW, 0.004, position.data(), tip.data());
    }
}

MujocoViewer::MujocoViewer(mjModel* model, mjData* data, const std::string& title, int width, int height)
    : m_(model), d_(data) {
    if (!m_ || !d_) {
        std::cerr << "Error: Null model or data passed to MujocoViewer!" << std::endl;
        return;
    }

    // 1. Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Error: Could not initialize GLFW!" << std::endl;
        return;
    }

    // 2. Create window with OpenGL context
    window_ = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
    if (!window_) {
        std::cerr << "Error: Could not create GLFW window!" << std::endl;
        glfwTerminate();
        return;
    }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1); // Enable v-sync (60 FPS cap)

    // Store `this` pointer in the window user pointer for callbacks
    glfwSetWindowUserPointer(window_, this);

    // 3. Register input callbacks
    glfwSetMouseButtonCallback(window_, mouse_button_callback);
    glfwSetCursorPosCallback(window_, mouse_move_callback);
    glfwSetScrollCallback(window_, scroll_callback);
    glfwSetWindowCloseCallback(window_, window_close_callback);

    // 4. Initialize MuJoCo visualization objects
    mjv_defaultCamera(&cam_);
    mjv_defaultOption(&opt_);
    mjv_defaultScene(&scn_);
    mjr_defaultContext(&con_);

    // Allocate scene geometry buffers (max 2000 geoms) and OpenGL context
    mjv_makeScene(m_, &scn_, 2000);
    mjr_makeContext(m_, &con_, mjFONTSCALE_150);

    // Set initial camera perspective
    cam_.type = mjCAMERA_FREE;
    cam_.lookat[0] = 0.0;
    cam_.lookat[1] = 0.0;
    cam_.lookat[2] = 0.5; // Look near the robot torso
    cam_.distance = 2.0;  // 2 meters away
    cam_.elevation = -20.0;
    cam_.azimuth = 90.0;
}

MujocoViewer::~MujocoViewer() {
    close();
}

bool MujocoViewer::is_open() const {
    return window_ && !glfwWindowShouldClose(window_);
}

void MujocoViewer::set_qpos(const Eigen::VectorXd& qpos) {
    int n = std::min(static_cast<int>(qpos.size()), static_cast<int>(m_->nq));
    for (int i = 0; i < n; ++i) {
        d_->qpos[i] = qpos[i];
    }
}

void MujocoViewer::render() {
    if (!is_open()) return;

    // Run forward kinematics to update link Cartesian positions
    mj_kinematics(m_, d_);

    // Get current window frame buffer dimensions
    int width, height;
    glfwGetFramebufferSize(window_, &width, &height);
    mjrRect viewport = {0, 0, width, height};

    // Update scene geometries and camera
    mjv_updateScene(m_, d_, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);

    if (pose_markers_.size() == 2) {
        draw_pose_marker(pose_markers_[0], "Start");
        draw_pose_marker(pose_markers_[1], "End");
    }

    // Render scene into OpenGL frame buffer
    mjr_render(viewport, &scn_, &con_);

    // Swap buffers and process UI events (mouse, keyboard)
    glfwSwapBuffers(window_);
    glfwPollEvents();
}

void MujocoViewer::close() {
    if (window_) {
        mjv_freeScene(&scn_);
        mjr_freeContext(&con_);
        glfwDestroyWindow(window_);
        window_ = nullptr;
        glfwTerminate();
    }
}

// ==========================================
// GLFW Mouse Callbacks for Camera Navigation
// ==========================================

void MujocoViewer::mouse_button_callback(GLFWwindow* window, int button, int act, int mods) {
    auto* self = static_cast<MujocoViewer*>(glfwGetWindowUserPointer(window));
    if (!self) return;

    self->button_left_   = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    self->button_middle_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    self->button_right_  = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    glfwGetCursorPos(window, &self->last_x_, &self->last_y_);
}

void MujocoViewer::mouse_move_callback(GLFWwindow* window, double xpos, double ypos) {
    auto* self = static_cast<MujocoViewer*>(glfwGetWindowUserPointer(window));
    if (!self) return;

    // If no mouse button is held down, do nothing
    if (!self->button_left_ && !self->button_middle_ && !self->button_right_) {
        return;
    }

    double dx = xpos - self->last_x_;
    double dy = ypos - self->last_y_;
    self->last_x_ = xpos;
    self->last_y_ = ypos;

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    bool shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                  glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    mjtMouse action;
    if (self->button_right_) {
        action = shift ? mjMOUSE_MOVE_V : mjMOUSE_ZOOM;
    } else if (self->button_left_) {
        action = shift ? mjMOUSE_MOVE_H : mjMOUSE_ROTATE_H;
    } else {
        action = mjMOUSE_ZOOM;
    }

    mjv_moveCamera(self->m_, action, dx / height, dy / height, &self->cam_);
}

void MujocoViewer::scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    auto* self = static_cast<MujocoViewer*>(glfwGetWindowUserPointer(window));
    if (!self) return;

    // Scroll wheel zooms the camera in/out
    mjv_moveCamera(self->m_, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &self->cam_);
}

void MujocoViewer::window_close_callback(GLFWwindow* window) {
    glfwSetWindowShouldClose(window, GLFW_TRUE);
}

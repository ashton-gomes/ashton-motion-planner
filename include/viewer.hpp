#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

class MujocoViewer {
public:
    MujocoViewer(mjModel* model, mjData* data, const std::string& title = "Robot Viewer", int width = 1200, int height = 900);
    ~MujocoViewer();

    // Load a scene and loop the joint trajectory with start/end coordinate frames.
    // Poses use [x, y, z, qw, qx, qy, qz]. Returns 1 if loading/opening fails.
    static int play(const std::string& scene_path,
                    const std::vector<Eigen::VectorXd>& trajectory,
                    const Eigen::Matrix<double, 7, 1>& start,
                    const Eigen::Matrix<double, 7, 1>& end);

    // Check if the window is still open
    bool is_open() const;

    // Set robot joint positions (qpos) directly from an Eigen vector
    void set_qpos(const Eigen::VectorXd& qpos);

    // Render one frame and poll mouse/keyboard events
    void render();

    // Close the viewer window manually
    void close();

private:
    std::vector<Eigen::Matrix<double, 7, 1>> pose_markers_;
    void draw_pose_marker(const Eigen::Matrix<double, 7, 1>& pose, const char* label);

    // GLFW Window Pointer
    GLFWwindow* window_ = nullptr;

    // Pointers to the shared model and data
    mjModel* m_ = nullptr;
    mjData*  d_ = nullptr;

    // MuJoCo Visualization structures
    mjvCamera  cam_;
    mjvOption  opt_;
    mjvScene   scn_;
    mjrContext con_;

    // Mouse interaction tracking
    bool button_left_   = false;
    bool button_middle_ = false;
    bool button_right_  = false;
    double last_x_      = 0.0;
    double last_y_      = 0.0;

    // Static callback proxies for GLFW
    static void mouse_button_callback(GLFWwindow* window, int button, int act, int mods);
    static void mouse_move_callback(GLFWwindow* window, double xpos, double ypos);
    static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
    static void window_close_callback(GLFWwindow* window);
};
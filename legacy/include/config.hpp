#pragma once
#include <Eigen/Dense>

namespace config {
    constexpr int N_JOINTS = 7;
    inline const char* XML_PATH = "/home/ash_gomes/gplanner/mujoco_menagerie/kinova_gen3/gen3_obstacle.xml";
    inline const char* EE_SITE  = "pinch_site";

    using Vector7d = Eigen::Matrix<double, N_JOINTS, 1>;

    // Joint Position and Limits 
    inline const Vector7d Q_MID{.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57}; // home site 
    inline const Vector7d Q_LIM_LOW{-6.28, -2.24, -6.28, -2.57, -6.28, -2.09, -6.28}; // minimum limit for each joint
    inline const Vector7d Q_LIM_HIGH{6.28,  2.24,  6.28,  2.57,  6.28,  2.09,  6.28}; // maximum limit for each joint 

    // Workspace Limits
    constexpr double WS_X_MIN = -0.2, WS_X_MAX = 0.65;
    constexpr double WS_Y_MIN = -0.5, WS_Y_MAX = 0.5;
    constexpr double WS_Z_MIN =  0.3, WS_Z_MAX = 0.95;

    // RRT Parameters
    constexpr int    RRT_MAX_NODES        = 4000;
    constexpr int    RRT_MAX_TREES        = 3;
    constexpr double RRT_GOAL_BIAS        = 0.15;
    constexpr double RRT_STEP_POS         = 0.01;
    constexpr double RRT_STEP_ANG         = 0.03;
    constexpr double GOAL_TOL_POS         = 0.02;
    constexpr double GOAL_TOL_ANG         = 0.05;
}

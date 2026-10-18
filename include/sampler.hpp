#pragma once

#include <Eigen/Dense>
#include <mujoco/mujoco.h>

#include <functional>
#include <cstdint>
#include <string>
#include <vector>

// Reset independent random streams and the shared cylinder-placement cache.
// Call before constructing models; false keeps the scene's XML placements.
void seedSampling(std::uint32_t seed, bool randomize_cylinders = true);

// Static MuJoCo obstacle geometry.  safety_clearance inflates the shape when
// deciding whether a Cartesian sample is valid.
struct SceneObstacle {
    Eigen::Vector3d center;
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d size = Eigen::Vector3d::Zero();
    int type = 0;
    double bounding_radius = 0.0;
    double safety_clearance = 0.0;
};

// Return one random Cartesian point in the spherical shell centered at
// workspace_center.  This is only a cheap workspace filter; it does not
// guarantee a collision-free or IK-reachable pose.
Eigen::Vector3d sampleWorkspacePoint(
    const Eigen::Vector3d& workspace_center,
    double inner_radius,
    double outer_radius);


// Ramdom quaternion implementation for full pose sampling later 
Eigen::Quaterniond sampleQuaternion();

// Derive position and orientation together from joints within their limits.
// is_valid can reuse the caller's workspace and collision checks. This ensures
// an accepted goal configuration exists, but does not guarantee a path to it.
Eigen::Matrix<double, 7, 1> sampleReachablePose(
    const Eigen::VectorXd& lower_limits,
    const Eigen::VectorXd& upper_limits,
    const std::function<Eigen::Matrix<double, 7, 1>(const Eigen::VectorXd&)>& forward_kinematics,
    const std::function<bool(const Eigen::VectorXd&, const Eigen::Matrix<double, 7, 1>&)>& is_valid,
    int max_attempts);

// Apply the shared per-run cylinder positions to a loaded scene model.
// Randomization can be disabled with one toggle in source/sampler.cpp.
void applyCylinderPlacements(mjModel* model);

// Load static physical obstacles from a MuJoCo scene.  This project puts
// robot collision meshes in group 3, so they and the floor plane are skipped.
std::vector<SceneObstacle> loadSceneObstacles(
    const std::string& scene_path,
    double safety_clearance);

bool isPointInsideObstacle(const Eigen::Vector3d& point,
                           const std::vector<SceneObstacle>& obstacles);

Eigen::Vector3d sampleWorkspacePointAvoidingObstacles(
    const Eigen::Vector3d& workspace_center,
    double inner_radius,
    double outer_radius,
    const std::vector<SceneObstacle>& obstacles,
    int max_attempts);

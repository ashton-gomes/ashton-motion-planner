#pragma once

#include "gplanner/obstacles.hpp"

#include <Eigen/Dense>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

// A pinhole depth camera placed in the world. The defaults are close to an
// Intel RealSense D435 depth stream (87 x 58 degree field of view), at a
// reduced resolution.
struct DepthCameraSettings {
    std::string name = "camera";
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d look_at = Eigen::Vector3d::UnitX();
    Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
    double horizontal_fov_deg = 87.0;
    double vertical_fov_deg = 58.0;
    int width = 320;
    int height = 180;
    double min_range = 0.2;   // Closer surfaces return nothing, in metres.
    double max_range = 3.0;   // Farther surfaces return nothing, in metres.
    // Depth noise along each ray: standard deviation noise_base +
    // noise_per_m2 * depth^2, in metres (depth error grows with distance).
    double noise_base = 0.001;
    double noise_per_m2 = 0.002;
    double dropout = 0.01;    // Fraction of pixels that return nothing.
};

// Anything that produces point clouds in the world frame for
// MotionPlanner::setPointCloud: the simulated sensor below, or a driver for a
// real camera that implements capture() the same way.
class DepthSensor {
public:
    virtual ~DepthSensor() = default;
    virtual PointCloud capture() = 0;
};

// Simulates depth cameras by casting one ray per pixel into a MuJoCo scene
// (headless; mj_multiRay). The robot is part of the scene, so it shows up in
// the cloud and hides what is behind it, as with a real camera; the planner
// removes the robot's own points. The floor is returned too; the planner
// crops it.
class SimulatedDepthSensor : public DepthSensor {
public:
    // edit_scene, if set, is applied to the scene first (e.g. the testbed's
    // random cylinder placements), so the sensor sees the same world as the
    // viewer. seed makes the noise and dropout reproducible.
    SimulatedDepthSensor(const std::string& scene_xml,
                         std::vector<DepthCameraSettings> cameras,
                         const SceneEditor& edit_scene = {},
                         std::uint32_t seed = 1);
    ~SimulatedDepthSensor() override;

    // Pose the scene's robot before capturing: a full scene qpos, e.g.
    // planner.toSceneQpos(planner.startJoints()). Defaults to the scene's
    // default pose.
    void setSceneQpos(const Eigen::VectorXd& qpos);

    // One capture from every camera, merged.
    PointCloud capture() override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

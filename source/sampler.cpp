#include "sampler.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <random>
#include <stdexcept>

Eigen::Vector3d sampleWorkspacePoint(
    const Eigen::Vector3d& workspace_center,
    double inner_radius,
    double outer_radius)
{
    if (inner_radius < 0.0 || outer_radius <= 0.0 ||
        inner_radius > outer_radius) {
        throw std::invalid_argument("Workspace radii are invalid.");
    }

    static std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> coordinate(-outer_radius,../
                                                       outer_radius);

    while (true) {
        const Eigen::Vector3d point = workspace_center + Eigen::Vector3d(
            coordinate(rng), coordinate(rng), coordinate(rng));
        const double radius = (point - workspace_center).norm();

        if (radius >= inner_radius && radius <= outer_radius) {
            return point;
        }
    }
}

std::vector<SceneObstacle> loadSceneObstacles(
    const std::string& scene_path,
    double safety_clearance)
{
    if (safety_clearance < 0.0) {
        throw std::invalid_argument("Obstacle safety clearance cannot be negative.");
    }

    char error[500]{};
    mjModel* model = mj_loadXML(scene_path.c_str(), nullptr, error, sizeof(error));
    if (!model) {
        throw std::runtime_error(std::string("Scene load error: ") + error);
    }
    mjData* data = mj_makeData(model);
    if (!data) {
        mj_deleteModel(model);
        throw std::runtime_error("Could not create scene data for obstacle sampling.");
    }

    mj_forward(model, data);
    std::vector<SceneObstacle> obstacles;
    for (int geom = 0; geom < model->ngeom; ++geom) {
        const bool robot_collision_mesh = model->geom_group[geom] == 3;
        const bool floor_plane = model->geom_type[geom] == mjGEOM_PLANE;
        const bool physical = model->geom_contype[geom] != 0 &&
                              model->geom_conaffinity[geom] != 0;
        if (robot_collision_mesh || floor_plane || !physical) {
            continue;
        }

        SceneObstacle obstacle;
        obstacle.center = Eigen::Map<Eigen::Vector3d>(&data->geom_xpos[3 * geom]);
        obstacle.rotation = Eigen::Map<
            const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
                &data->geom_xmat[9 * geom]);
        obstacle.size = Eigen::Map<Eigen::Vector3d>(&model->geom_size[3 * geom]);
        obstacle.type = model->geom_type[geom];
        obstacle.bounding_radius = model->geom_rbound[geom];
        obstacle.safety_clearance = safety_clearance;
        obstacles.push_back(obstacle);
    }

    mj_deleteData(data);
    mj_deleteModel(model);
    return obstacles;
}

bool isPointInsideObstacle(const Eigen::Vector3d& point,
                           const std::vector<SceneObstacle>& obstacles)
{
    for (const SceneObstacle& obstacle : obstacles) {
        const Eigen::Vector3d local =
            obstacle.rotation.transpose() * (point - obstacle.center);
        const double clearance = obstacle.safety_clearance;

        switch (obstacle.type) {
        case mjGEOM_SPHERE:
            if (local.norm() <= obstacle.size[0] + clearance) return true;
            break;
        case mjGEOM_BOX:
            if ((local.array().abs() <=
                 (obstacle.size.array() + clearance)).all()) return true;
            break;
        case mjGEOM_CYLINDER:
            if (local.head<2>().norm() <= obstacle.size[0] + clearance &&
                std::abs(local[2]) <= obstacle.size[1] + clearance) return true;
            break;
        case mjGEOM_CAPSULE: {
            const double z = std::clamp(local[2], -obstacle.size[1],
                                         obstacle.size[1]);
            if ((local - Eigen::Vector3d(0.0, 0.0, z)).norm() <=
                obstacle.size[0] + clearance) return true;
            break;
        }
        default:
            // Meshes and uncommon geom types use a conservative fallback.
            if (local.norm() <= obstacle.bounding_radius + clearance) return true;
        }
    }
    return false;
}

Eigen::Vector3d sampleWorkspacePointAvoidingObstacles(
    const Eigen::Vector3d& workspace_center,
    double inner_radius,
    double outer_radius,
    const std::vector<SceneObstacle>& obstacles,
    int max_attempts)
{
    if (max_attempts <= 0) {
        throw std::invalid_argument("Maximum obstacle-sampling attempts must be positive.");
    }

    for (int attempt = 0; attempt < max_attempts; ++attempt) {
        const Eigen::Vector3d point = sampleWorkspacePoint(
            workspace_center, inner_radius, outer_radius);
        if (!isPointInsideObstacle(point, obstacles)) {
            return point;
        }
    }
    throw std::runtime_error("Could not sample a point outside scene obstacles.");
}

#include "sampler.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <random>
#include <stdexcept>

namespace {

// Set this to false to use the positions written in scene.xml.
constexpr bool kRandomizeCylinderPlacements = true;

constexpr std::array<const char*, 3> kCylinderNames = {
    "obs_cylinder_1", "obs_cylinder_2", "obs_cylinder_3"
};

// Adjust these bounds to change where the cylinders may appear.
constexpr double kCylinderMinX = -0.20;
constexpr double kCylinderMaxX = 0.25;
constexpr double kCylinderMinY = -0.35;
constexpr double kCylinderMaxY = -0.18;
constexpr double kCylinderZ = 0.44;
constexpr double kMinimumCylinderSpacing = 0.13;

const std::vector<Eigen::Vector3d>& randomCylinderPositions()
{
    static const std::vector<Eigen::Vector3d> positions = [] {
        std::vector<Eigen::Vector3d> result;
        if (!kRandomizeCylinderPlacements) {
            return result;
        }

        std::mt19937 rng(std::random_device{}());
        std::uniform_real_distribution<double> x(kCylinderMinX, kCylinderMaxX);
        std::uniform_real_distribution<double> y(kCylinderMinY, kCylinderMaxY);

        for (size_t cylinder = 0; cylinder < kCylinderNames.size(); ++cylinder) {
            bool placed = false;
            for (int attempt = 0; attempt < 1000 && !placed; ++attempt) {
                const Eigen::Vector3d candidate(x(rng), y(rng), kCylinderZ);
                placed = std::all_of(
                    result.begin(), result.end(),
                    [&](const Eigen::Vector3d& position) {
                        return (candidate.head<2>() - position.head<2>()).norm() >=
                               kMinimumCylinderSpacing;
                    });
                if (placed) {
                    result.push_back(candidate);
                }
            }
            if (!placed) {
                throw std::runtime_error("Could not place random cylinders without overlap.");
            }
        }
        return result;
    }();
    return positions;
}

} // namespace

void applyCylinderPlacements(mjModel* model)
{
    if (!model || !kRandomizeCylinderPlacements) {
        return;
    }

    const auto& positions = randomCylinderPositions();
    for (size_t i = 0; i < kCylinderNames.size(); ++i) {
        const int geom = mj_name2id(model, mjOBJ_GEOM, kCylinderNames[i]);
        if (geom < 0) {
            throw std::runtime_error(
                std::string("Cylinder geom not found: ") + kCylinderNames[i]);
        }
        model->geom_pos[3 * geom] = positions[i].x();
        model->geom_pos[3 * geom + 1] = positions[i].y();
        model->geom_pos[3 * geom + 2] = positions[i].z();
    }
}

Eigen::Vector3d sampleWorkspacePoint(
    const Eigen::Vector3d& workspace_center,
    double inner_radius,
    double outer_radius)
{
    if (inner_radius < 0.0 || outer_radius <= 0.0 ||
        inner_radius > outer_radius) {
        throw std::invalid_argument("Workspace radii are invalid.");
    }

        // point within environment 
    static std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> coordinate(-outer_radius,
                                                       outer_radius);

        // sampling point from the center of defined workspace by URDF
    while (true) {
        const Eigen::Vector3d point = workspace_center + Eigen::Vector3d(
            coordinate(rng), coordinate(rng), coordinate(rng));
        const double radius = (point - workspace_center).norm();

        if (radius >= inner_radius && radius <= outer_radius) {
            return point;
        }
    }
}

// Random Quaternion 

Eigen::Quaterniond sampleQuaternion() {
    static std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> coordinate(-1.0, 1.0);
    double x,y,z, u,v,w, s;
    do { x = coordinate(rng); y = coordinate(rng); z = x*x + y*y; } while (z > 1);
    do { u = coordinate(rng); v = coordinate(rng); w = u*u + v*v; } while (w > 1);
    s = sqrt((1-z) / w);
    return Eigen::Quaterniond(x, y, s*u, s*v);

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
    applyCylinderPlacements(model);
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

#include "sampler.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <random>
#include <stdexcept>

namespace {

// Interactive runs randomize cylinders; benchmarks can disable this with seedSampling.
bool randomizeCylinderPlacements = true;
std::mt19937 cylinderRng(std::random_device{}());
std::mt19937 workspaceRng(std::random_device{}());
std::mt19937 quaternionRng(std::random_device{}());
std::mt19937 goalRng(std::random_device{}());
// Where one scene's cylinders may appear. Adjust these bounds to change it.
struct CylinderLayout {
    std::array<const char*, 3> names;
    double min_x, max_x, min_y, max_y, z;
    double minimum_spacing;
};

constexpr std::array<CylinderLayout, 2> kCylinderLayouts = {{
    // random_cylinders.xml (Kinova Gen3): beside the arm's base.
    {{"obs_cylinder_1", "obs_cylinder_2", "obs_cylinder_3"},
     -0.20, 0.25, -0.35, -0.18, 0.44, 0.13},
    // g1_random_cylinders.xml (G1 right arm): in front of the right shoulder,
    // clear of the hands' home poses.
    {{"g1_obs_cylinder_1", "g1_obs_cylinder_2", "g1_obs_cylinder_3"},
     0.25, 0.38, -0.40, 0.00, 1.00, 0.10},
}};

// Positions for this run, one list per layout, chosen on first use.
std::array<std::vector<Eigen::Vector3d>, kCylinderLayouts.size()> cylinderPositions;

const std::vector<Eigen::Vector3d>& randomCylinderPositions(size_t layout_index)
{
    auto& positions = cylinderPositions[layout_index];
    if (!positions.empty()) return positions;
    const CylinderLayout& layout = kCylinderLayouts[layout_index];
    std::vector<Eigen::Vector3d> result;
    std::uniform_real_distribution<double> x(layout.min_x, layout.max_x);
    std::uniform_real_distribution<double> y(layout.min_y, layout.max_y);
    for (size_t cylinder = 0; cylinder < layout.names.size(); ++cylinder) {
        bool placed = false;
        for (int attempt = 0; attempt < 1000 && !placed; ++attempt) {
            const Eigen::Vector3d candidate(x(cylinderRng), y(cylinderRng), layout.z);
            placed = std::all_of(result.begin(), result.end(),
                [&](const Eigen::Vector3d& position) {
                    return (candidate.head<2>() - position.head<2>()).norm() >=
                           layout.minimum_spacing;
                });
            if (placed) result.push_back(candidate);
        }
        if (!placed) throw std::runtime_error("Could not place random cylinders without overlap.");
    }
    positions = std::move(result);
    return positions;
}

} // namespace

void seedSampling(std::uint32_t seed, bool randomize_cylinders)
{
    cylinderRng.seed(seed + 101u);
    workspaceRng.seed(seed + 102u);
    quaternionRng.seed(seed + 103u);
    goalRng.seed(seed + 104u);
    randomizeCylinderPlacements = randomize_cylinders;
    for (auto& positions : cylinderPositions) positions.clear();
}

void applyCylinderPlacements(mjModel* model)
{
    if (!model || !randomizeCylinderPlacements) {
        return;
    }

    for (size_t layout = 0; layout < kCylinderLayouts.size(); ++layout) {
        const auto& names = kCylinderLayouts[layout].names;
        // Other environments need not contain these cylinders.
        if (mj_name2id(model, mjOBJ_GEOM, names[0]) < 0) continue;
        const auto& positions = randomCylinderPositions(layout);
        for (size_t i = 0; i < names.size(); ++i) {
            const int geom = mj_name2id(model, mjOBJ_GEOM, names[i]);
            if (geom < 0) continue;
            model->geom_pos[3 * geom] = positions[i].x();
            model->geom_pos[3 * geom + 1] = positions[i].y();
            model->geom_pos[3 * geom + 2] = positions[i].z();
        }
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
    std::uniform_real_distribution<double> coordinate(-outer_radius,
                                                       outer_radius);

        // sampling point from the center of defined workspace by URDF
    while (true) {
        const Eigen::Vector3d point = workspace_center + Eigen::Vector3d(
            coordinate(workspaceRng), coordinate(workspaceRng), coordinate(workspaceRng));
        const double radius = (point - workspace_center).norm();

        if (radius >= inner_radius && radius <= outer_radius) {
            return point;
        }
    }
}

// Random Quaternion 

Eigen::Quaterniond sampleQuaternion() {
    std::uniform_real_distribution<double> coordinate(-1.0, 1.0);
    double x,y,z, u,v,w, s;
    do { x = coordinate(quaternionRng); y = coordinate(quaternionRng); z = x*x + y*y; } while (z > 1);
    do { u = coordinate(quaternionRng); v = coordinate(quaternionRng); w = u*u + v*v; } while (w > 1 || w == 0);
    s = sqrt((1-z) / w);
    return Eigen::Quaterniond(x, y, s*u, s*v);

}

Eigen::Matrix<double, 7, 1> sampleReachablePose(
    const Eigen::VectorXd& lower_limits,
    const Eigen::VectorXd& upper_limits,
    const std::function<Eigen::Matrix<double, 7, 1>(const Eigen::VectorXd&)>& forward_kinematics,
    const std::function<bool(const Eigen::VectorXd&, const Eigen::Matrix<double, 7, 1>&)>& is_valid,
    int max_attempts)
{
    if (max_attempts <= 0 || lower_limits.size() == 0 ||
        lower_limits.size() != upper_limits.size() ||
        !lower_limits.allFinite() || !upper_limits.allFinite() ||
        (lower_limits.array() > upper_limits.array()).any()) {
        throw std::invalid_argument("Reachable-pose sampling requires valid joint limits and positive attempts.");
    }

    for (int attempt = 0; attempt < max_attempts; ++attempt) {
        Eigen::VectorXd joints(lower_limits.size());
        for (int joint = 0; joint < joints.size(); ++joint) {
            std::uniform_real_distribution<double> angle(lower_limits[joint], upper_limits[joint]);
            joints[joint] = angle(goalRng);
        }
        const auto pose = forward_kinematics(joints);
        if (pose.allFinite() && is_valid(joints, pose)) return pose;
    }
    throw std::runtime_error("Could not sample a valid reachable pose within the attempt limit.");
}

std::vector<SceneObstacle> loadSceneObstacles(
    const std::string& scene_path,
    double safety_clearance,
    const std::unordered_set<std::string>& robot_body_names)
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
        const char* body = mj_id2name(model, mjOBJ_BODY, model->geom_bodyid[geom]);
        const bool robot_collision_mesh = body && robot_body_names.count(body) > 0;
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

std::optional<Eigen::VectorXd> loadSceneKeyframe(const std::string& scene_path,
                                                 const std::string& name)
{
    char error[500]{};
    mjModel* model = mj_loadXML(scene_path.c_str(), nullptr, error, sizeof(error));
    if (!model) {
        throw std::runtime_error(std::string("Scene load error: ") + error);
    }
    std::optional<Eigen::VectorXd> qpos;
    const int key = mj_name2id(model, mjOBJ_KEY, name.c_str());
    if (key >= 0) {
        qpos = Eigen::Map<const Eigen::VectorXd>(&model->key_qpos[key * model->nq], model->nq);
    }
    mj_deleteModel(model);
    return qpos;
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

#include "testbed/scene_randomizer.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <random>
#include <stdexcept>
#include <vector>

namespace {

// Interactive runs randomize cylinders; set to false to keep the XML placements.
bool randomizeCylinderPlacements = true;
std::mt19937 cylinderRng(std::random_device{}());

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

void seedSceneRandomization(std::uint32_t seed, bool enabled)
{
    cylinderRng.seed(seed + 101u);
    randomizeCylinderPlacements = enabled;
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

#include "gplanner/obstacles.hpp"
#include "voxel_grid.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>
#include <tuple>

namespace voxel_grid {
namespace {

// Sort order (z, y, x), so cells in one x-row are next to each other.
bool rowOrder(const Eigen::Vector3i& a, const Eigen::Vector3i& b) {
    return std::make_tuple(a.z(), a.y(), a.x()) < std::make_tuple(b.z(), b.y(), b.x());
}

SceneObstacle box(const Eigen::Vector3d& center, const Eigen::Vector3d& half_size) {
    SceneObstacle obstacle;
    obstacle.center = center;
    obstacle.size = half_size;
    obstacle.type = mjGEOM_BOX;
    obstacle.bounding_radius = half_size.norm();
    return obstacle;
}

}  // namespace

std::vector<Eigen::Vector3i> occupiedCells(const PointCloud& cloud,
                                           const PointCloudSettings& settings) {
    if (!(settings.voxel_size > 0.0) || settings.min_points_per_voxel < 1)
        throw std::invalid_argument("Point-cloud voxel size and minimum points must be positive.");
    const auto order = [](const Eigen::Vector3i& a, const Eigen::Vector3i& b) { return rowOrder(a, b); };
    std::map<Eigen::Vector3i, int, decltype(order)> counts(order);
    for (const Eigen::Vector3d& point : cloud.points) {
        if (!point.allFinite()) continue;
        const Eigen::Vector3i cell = (point / settings.voxel_size).array().floor().cast<int>();
        ++counts[cell];
    }
    std::vector<Eigen::Vector3i> cells;
    for (const auto& [cell, count] : counts)
        if (count >= settings.min_points_per_voxel) cells.push_back(cell);
    return cells;
}

SceneObstacle cellBox(const Eigen::Vector3i& cell, double voxel_size) {
    return box((cell.cast<double>().array() + 0.5) * voxel_size,
               Eigen::Vector3d::Constant(voxel_size / 2));
}

std::vector<SceneObstacle> mergeCells(const std::vector<Eigen::Vector3i>& cells,
                                      double voxel_size) {
    // Cells not yet covered by a box.
    std::set<std::tuple<int, int, int>> free;
    for (const auto& cell : cells) free.emplace(cell.z(), cell.y(), cell.x());
    const auto available = [&](int x, int y, int z) { return free.count({z, y, x}) > 0; };

    std::vector<SceneObstacle> boxes;
    for (const auto& cell : cells) {
        if (!available(cell.x(), cell.y(), cell.z())) continue;
        const int x0 = cell.x(), y0 = cell.y(), z0 = cell.z();
        int x1 = x0, y1 = y0, z1 = z0;  // inclusive
        while (available(x1 + 1, y0, z0)) ++x1;
        const auto row_free = [&](int y, int z) {
            for (int x = x0; x <= x1; ++x) if (!available(x, y, z)) return false;
            return true;
        };
        while (row_free(y1 + 1, z0)) ++y1;
        const auto layer_free = [&](int z) {
            for (int y = y0; y <= y1; ++y) if (!row_free(y, z)) return false;
            return true;
        };
        while (layer_free(z1 + 1)) ++z1;
        for (int z = z0; z <= z1; ++z)
            for (int y = y0; y <= y1; ++y)
                for (int x = x0; x <= x1; ++x) free.erase({z, y, x});
        const Eigen::Vector3d low = Eigen::Vector3d(x0, y0, z0) * voxel_size;
        const Eigen::Vector3d high = Eigen::Vector3d(x1 + 1, y1 + 1, z1 + 1) * voxel_size;
        boxes.push_back(box((low + high) / 2, (high - low) / 2));
    }
    return boxes;
}

}  // namespace voxel_grid

std::vector<SceneObstacle> voxelizePointCloud(const PointCloud& cloud,
                                              const PointCloudSettings& settings) {
    return voxel_grid::mergeCells(voxel_grid::occupiedCells(cloud, settings),
                                  settings.voxel_size);
}

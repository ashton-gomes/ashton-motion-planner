#pragma once

// Internal to the planner library: voxel helpers behind voxelizePointCloud.

#include "gplanner/obstacles.hpp"

#include <vector>

namespace voxel_grid {

// Integer voxel coordinates of every voxel holding at least
// settings.min_points_per_voxel points, sorted (z, y, x).
std::vector<Eigen::Vector3i> occupiedCells(const PointCloud& cloud,
                                           const PointCloudSettings& settings);

// One axis-aligned box per voxel cell.
SceneObstacle cellBox(const Eigen::Vector3i& cell, double voxel_size);

// Boxes covering exactly the cells. Greedy merge: each box grows along x,
// then y, then z, while every cell it would cover is occupied and unused.
std::vector<SceneObstacle> mergeCells(const std::vector<Eigen::Vector3i>& cells,
                                      double voxel_size);

}  // namespace voxel_grid

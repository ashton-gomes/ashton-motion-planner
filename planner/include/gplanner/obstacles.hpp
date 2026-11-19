#pragma once

#include "gplanner/sampler.hpp"

#include <Eigen/Dense>

#include <vector>

// Where the planner learns about obstacles.
enum class ObstacleSource {
    // The scene XML's own obstacle geoms, with their exact shapes. Use this
    // when the obstacles are known in advance.
    KnownScene,
    // A point cloud from a depth sensor (simulated or real), given with
    // MotionPlanner::setPointCloud. The scene XML's obstacles are ignored.
    PointCloud,
};

// 3D points in the world frame, in metres. A sensor driver converts its
// camera-frame points to the world frame before handing them over.
struct PointCloud {
    std::vector<Eigen::Vector3d> points;
};

// How a point cloud becomes obstacles. The planner crops it, groups the points
// into voxels, removes voxels on the robot itself, and treats every remaining
// voxel as a solid box (inflated by RRTStarSettings::obstacle_clearance).
struct PointCloudSettings {
    double voxel_size = 0.02;        // Edge length of one voxel, in metres.
    int min_points_per_voxel = 1;    // Fewer points than this is treated as noise.
    // Points at or below this height are the floor and are dropped. The floor
    // itself is never an obstacle, just as plane geoms are not in KnownScene.
    double floor_height = 0.04;
    // Points farther than this from the arm's reachable workspace are dropped.
    double crop_margin = 0.30;
    // Voxels within obstacle_clearance + this distance of the robot, posed at
    // its start configuration, are the robot seeing itself and are removed.
    double self_filter_padding = 0.02;
};

struct ObstacleOptions {
    ObstacleSource source = ObstacleSource::KnownScene;
    SceneEditor edit_scene;           // KnownScene only: applied to the scene first.
    PointCloudSettings point_cloud;   // PointCloud only.
};

// Group points into voxels of settings.voxel_size and return the occupied ones
// as axis-aligned boxes. Neighbouring voxels are merged into larger boxes that
// cover exactly the same space. No cropping or robot filtering is done here.
std::vector<SceneObstacle> voxelizePointCloud(const PointCloud& cloud,
                                              const PointCloudSettings& settings);

#include "gplanner/planner.hpp"
#include "testbed/obstacle_mode.hpp"
#include "testbed/robots.hpp"
#include "testbed/sensor.hpp"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

// Every scene must load. A scene with a fixed task must also block the direct
// straight-line motion; otherwise it does not test planning. The planner itself
// rejects start or goal keyframes that are in collision.
void checkEnvironments() {
    for (const auto& entry : std::filesystem::directory_iterator(ENVIRONMENTS_PATH)) {
        if (entry.path().extension() != ".xml") continue;
        const std::string name = entry.path().filename().string();
        seedSampling(1);
        // Files named g1_*.xml include the G1; the others include the Kinova Gen3.
        const bool g1 = name.rfind("g1_", 0) == 0;
        MotionPlanner planner(g1 ? robots::g1RightArm().config : robots::kinovaGen3().config,
                              entry.path().string());
        if (!planner.sceneGoal()) continue;
        const Pose& goal = *planner.sceneGoal();
        require((goal.head<3>() - planner.homePose().head<3>()).norm() > 0.1,
                name + ": the fixed goal must be away from the start.");
        const auto direct = planner.plan(goal, PlannerMode::DirectScLERP);
        require(direct.failure_reason == "direct_edge_rejected",
                name + ": the scene must block the direct motion, got '" +
                (direct.success ? std::string("success") : direct.failure_reason) + "'.");
        // The direct screw may also hit a joint limit before reaching the obstacle.
        require(direct.stats.obstacle_collision + direct.stats.joint_path_collision +
                direct.stats.joint_limit > 0,
                name + ": the direct motion must be blocked by an obstacle or a joint limit.");
    }
}

// Voxel boxes must cover exactly the occupied voxels, merged where possible.
void checkVoxelization() {
    PointCloudSettings settings;
    settings.voxel_size = 0.02;
    PointCloud plane;  // A 3 x 3 patch of voxels, one point each.
    for (int x = 0; x < 3; ++x)
        for (int y = 0; y < 3; ++y) plane.points.emplace_back(0.01 + 0.02 * x, 0.01 + 0.02 * y, 0.51);
    const auto boxes = voxelizePointCloud(plane, settings);
    require(boxes.size() == 1 && boxes[0].size.isApprox(Eigen::Vector3d(0.03, 0.03, 0.01)) &&
            boxes[0].center.isApprox(Eigen::Vector3d(0.03, 0.03, 0.51)),
            "A full 3 x 3 voxel patch must merge into one box.");

    PointCloud shape;  // An L of 3 + 2 voxels, plus a lone noise point.
    for (const Eigen::Vector3d p : {Eigen::Vector3d(0.01, 0.01, 0.01), Eigen::Vector3d(0.03, 0.01, 0.01),
                                    Eigen::Vector3d(0.05, 0.01, 0.01), Eigen::Vector3d(0.01, 0.03, 0.01),
                                    Eigen::Vector3d(0.01, 0.05, 0.01)}) {
        shape.points.push_back(p);
        shape.points.push_back(p + Eigen::Vector3d::Constant(0.001));  // two points per voxel
    }
    shape.points.emplace_back(0.51, 0.51, 0.51);
    settings.min_points_per_voxel = 2;
    double volume = 0.0;
    const auto l_boxes = voxelizePointCloud(shape, settings);
    for (const auto& box : l_boxes) volume += 8 * box.size.prod();
    require(std::abs(volume - 5 * std::pow(0.02, 3)) < 1e-12,
            "Merged boxes must cover exactly the voxels with enough points.");
    for (size_t i = 0; i + 1 < shape.points.size(); ++i)
        require(isPointInsideObstacle(shape.points[i], l_boxes), "Every kept point must be inside a box.");
    require(!isPointInsideObstacle(shape.points.back(), l_boxes), "A lone noise point must be dropped.");
}

// Sensed obstacles: the simulated cameras, floor cropping, robot self-filter,
// and planning against the sensed cloud, judged against the true scene.
void checkPointClouds() {
    const auto scene_path = [](const std::string& name) {
        return (std::filesystem::path(ENVIRONMENTS_PATH) / name).string();
    };
    {
        MotionPlanner known(robots::kinovaGen3().config, scene_path("empty.xml"));
        bool rejected = false;
        try { known.setPointCloud({}); } catch (const std::logic_error&) { rejected = true; }
        require(rejected, "setPointCloud must be rejected for a known-geometry planner.");
    }
    {
        const auto robot = robots::kinovaGen3();
        SimulatedDepthSensor a(scene_path("wall_hole.xml"), robot.cameras, {}, 3);
        SimulatedDepthSensor b(scene_path("wall_hole.xml"), robot.cameras, {}, 3);
        SimulatedDepthSensor c(scene_path("wall_hole.xml"), robot.cameras, {}, 4);
        const PointCloud first = a.capture(), second = b.capture(), other = c.capture();
        require(!first.points.empty() && first.points.size() == second.points.size(),
                "The simulated sensor must return points, reproducibly.");
        for (size_t i = 0; i < first.points.size(); ++i)
            require(first.points[i] == second.points[i], "The same sensor seed must give the same cloud.");
        require(first.points.size() != other.points.size() || first.points[0] != other.points[0],
                "A different sensor seed must change the noise.");
    }
    for (const auto& entry : std::filesystem::directory_iterator(ENVIRONMENTS_PATH)) {
        if (entry.path().extension() != ".xml") continue;
        const std::string name = entry.path().filename().string();
        const bool g1 = name.rfind("g1_", 0) == 0;
        const robots::TestRobot robot = g1 ? robots::g1RightArm() : robots::kinovaGen3();
        seedSampling(1);
        MotionPlanner truth(robot.config, entry.path().string());
        MotionPlanner sensed(robot.config, entry.path().string(), {},
                             obstacleOptions(ObstacleMode::SimulatedSensor));
        const PointCloud cloud = senseObstacles(sensed, robot, entry.path().string());
        require(!cloud.points.empty(), name + ": the cameras must see something.");
        // The robot sees itself and the floor; neither may become an obstacle.
        const auto start = sensed.plan(sensed.homePose(), PlannerMode::DirectScLERP);
        require(start.success, name + ": the start must stay free after removing the robot's own points, got '" +
                start.failure_reason + "'.");
        if (truth.obstacles().empty()) {
            require(sensed.obstacles().empty(), name + ": an empty scene must give no sensed obstacles.");
            continue;
        }
        require(!sensed.obstacles().empty(), name + ": the cameras must see the scene's obstacles.");
        // Every sensed box lies on a real obstacle (within noise and voxel size).
        std::vector<SceneObstacle> nearby = truth.obstacles();
        for (auto& obstacle : nearby) obstacle.safety_clearance = 0.05;
        size_t on_obstacle = 0;
        for (const auto& box : sensed.obstacles()) on_obstacle += isPointInsideObstacle(box.center, nearby);
        require(on_obstacle >= 0.95 * sensed.obstacles().size(),
                name + ": sensed boxes must lie on the scene's real obstacles.");
    }
    // Plan around the sensed cloud and validate against the true scene.
    const auto robot = robots::kinovaGen3();
    MotionPlanner truth(robot.config, scene_path("random_cylinders.xml"));
    MotionPlanner sensed(robot.config, scene_path("random_cylinders.xml"), {},
                         obstacleOptions(ObstacleMode::SimulatedSensor));
    senseObstacles(sensed, robot, scene_path("random_cylinders.xml"));
    seedSampling(1);
    seedRRTSampling(1);
    const Pose goal = truth.sampleGoal();
    const auto result = sensed.plan(goal, PlannerMode::SampledScLERP);
    require(result.success && truth.validatePath(result, goal).valid,
            "A seeded plan around sensed cylinders must reach the goal and be valid in the true scene.");
}

// Plan with one arm of the G1 humanoid while its legs, waist, and other arm
// are held still, in the obstacle-free G1 environment.
void checkG1Arms() {
    const std::string scene = (std::filesystem::path(ENVIRONMENTS_PATH) / "g1_empty.xml").string();
    for (const RobotConfig& robot : {robots::g1RightArm().config, robots::g1LeftArm().config}) {
        seedSampling(1);
        seedRRTSampling(1);
        MotionPlanner planner(robot, scene);
        require(planner.dof() == 7, robot.name + ": expected 7 planned joints.");
        const auto home = planner.plan(planner.homePose(), PlannerMode::DirectScLERP);
        require(home.success && planner.validatePath(home, planner.homePose()).valid,
                robot.name + ": home must be collision-free and agree with MuJoCo FK.");
        const Eigen::VectorXd scene_qpos = planner.toSceneQpos(home.trajectory.front());
        require(scene_qpos.size() > planner.dof(),
                robot.name + ": the scene pose must include the held joints.");
        const Pose goal = planner.sampleGoal();
        const auto result = planner.plan(goal, PlannerMode::SampledScLERP);
        require(result.success, robot.name + ": a sampled goal must be reached, got '" +
                result.failure_reason + "'.");
        const auto checked = planner.validatePath(result, goal);
        require(checked.collision_free && checked.position_error < 1e-3 && checked.rotation_error < 1e-3,
                robot.name + ": the reached pose must agree with independent MuJoCo FK.");
        require(checked.joint_limits_ok, robot.name + ": the path must stay within the URDF joint limits.");
    }
    // The URDF limits the elbow to [-1.0472, 2.0944]; a home pose past it is rejected.
    RobotConfig bent = robots::g1RightArm().config;
    bent.home_joints[3] = 2.2;
    bool rejected = false;
    try { MotionPlanner planner(bent, scene); } catch (const std::runtime_error&) { rejected = true; }
    require(rejected, "A home pose outside the URDF joint limits must be rejected.");
}

int main() {
    try {
        checkEnvironments();
        checkVoxelization();
        checkPointClouds();
        checkG1Arms();
        seedSampling(1);
        seedRRTSampling(1);
        MotionPlanner planner(robots::kinovaGen3().config,
            (std::filesystem::path(ENVIRONMENTS_PATH) / "empty.xml").string());
        for (auto mode : {PlannerMode::DirectScLERP, PlannerMode::SampledScLERP}) {
            const auto result = planner.plan(planner.homePose(), mode);
            require(result.success && result.tree.size() == 1, "Home-to-home plan must succeed without growth.");
            require(planner.validatePath(result, planner.homePose()).valid,
                    "Home must agree with independent MuJoCo FK and pass constraints.");
        }
        // A valid endpoint must not hide a joint-limit violation in the path.
        auto invalid_path = planner.plan(planner.homePose(), PlannerMode::DirectScLERP);
        invalid_path.trajectory.push_back(Eigen::VectorXd::Constant(planner.dof(), 100.0));
        invalid_path.trajectory.push_back(invalid_path.tree.front().qpos);
        const auto invalid_check = planner.validatePath(invalid_path, planner.homePose(), 24);
        require(!invalid_check.joint_limits_ok && !invalid_check.valid &&
                invalid_check.position_error < 1e-3,
                "Validation must reject intermediate joint-limit violations even with a valid endpoint.");
        // Reach a nearby pose that differs only in orientation: Cartesian
        // nearest-node logic must not reject this as a zero-length task.
        Pose rotation_goal = planner.homePose();
        Eigen::Quaterniond rotation(rotation_goal[3], rotation_goal[4], rotation_goal[5], rotation_goal[6]);
        rotation = Eigen::Quaterniond(Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitZ())) * rotation;
        rotation_goal.tail<4>() << rotation.w(), rotation.x(), rotation.y(), rotation.z();
        for (auto mode : {PlannerMode::DirectScLERP, PlannerMode::SampledScLERP}) {
            const auto result = planner.plan(rotation_goal, mode);
            require(result.success, "A small pure-rotation task must converge.");
            require(planner.validatePath(result, rotation_goal).valid, "Pure rotation must validate.");
        }
        seedSampling(1);
        const Pose goal = planner.sampleGoal();
        const auto direct = planner.plan(goal, PlannerMode::DirectScLERP);
        require(direct.stats.expansions == 1 && direct.tree.size() <= 2,
                "Direct ScLERP must never create a sampled tree.");
        seedSampling(1);
        seedRRTSampling(1);
        require(goal.isApprox(planner.sampleGoal(), 1e-14), "Seed reset must reproduce the full goal pose.");
        const auto first = planner.plan(goal, PlannerMode::SampledScLERP);
        seedSampling(1);
        seedRRTSampling(1);
        planner.sampleGoal();
        const auto repeated = planner.plan(goal, PlannerMode::SampledScLERP);
        require(first.success && repeated.success, "Fixed-seed sampled task must reach its goal.");
        require(first.tree.size() == repeated.tree.size() &&
                first.trajectory.size() == repeated.trajectory.size(), "Seeded tree must reproduce.");
        for (size_t i = 0; i < first.trajectory.size(); ++i)
            require(first.trajectory[i].isApprox(repeated.trajectory[i], 1e-14), "Seeded path must reproduce.");
        const auto checked = planner.validatePath(first, goal);
        require(checked.collision_free && checked.position_error < 1e-3 && checked.rotation_error < 1e-3,
                "Reached pose must agree with independent MuJoCo FK.");
        Pose invalid = goal;
        invalid.tail<4>().setZero();
        require(!planner.plan(invalid, PlannerMode::DirectScLERP).success, "Invalid quaternion must fail.");
        std::cout << "Planner integration checks passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

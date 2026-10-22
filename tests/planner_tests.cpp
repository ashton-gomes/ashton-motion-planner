#include "planner.hpp"

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
        seedSampling(1, false);
        MotionPlanner planner(ROBOT_XML_PATH, entry.path().string());
        if (!planner.sceneGoal()) continue;
        const Pose& goal = *planner.sceneGoal();
        require((goal.head<3>() - planner.homePose().head<3>()).norm() > 0.1,
                name + ": the fixed goal must be away from the start.");
        const auto direct = planner.plan(goal, PlannerMode::DirectScLERP);
        require(direct.failure_reason == "direct_edge_rejected",
                name + ": the scene must block the direct motion, got '" +
                (direct.success ? std::string("success") : direct.failure_reason) + "'.");
        require(direct.stats.obstacle_collision + direct.stats.joint_path_collision > 0,
                name + ": the direct motion must be blocked by an obstacle.");
    }
}

int main() {
    try {
        checkEnvironments();
        seedSampling(1, false);
        seedRRTSampling(1);
        MotionPlanner planner(ROBOT_XML_PATH,
            (std::filesystem::path(ENVIRONMENTS_PATH) / "empty.xml").string());
        for (auto mode : {PlannerMode::DirectScLERP, PlannerMode::SampledScLERP}) {
            const auto result = planner.plan(planner.homePose(), mode);
            require(result.success && result.tree.size() == 1, "Home-to-home plan must succeed without growth.");
            require(planner.validatePath(result, planner.homePose()).valid,
                    "Home must agree with independent MuJoCo FK and pass constraints.");
        }
        // A valid endpoint must not hide a joint-limit violation in the path.
        auto invalid_path = planner.plan(planner.homePose(), PlannerMode::DirectScLERP);
        invalid_path.trajectory.push_back(Eigen::VectorXd::Constant(7, 100.0));
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
        seedSampling(1, false);
        const Pose goal = planner.sampleGoal();
        const auto direct = planner.plan(goal, PlannerMode::DirectScLERP);
        require(direct.stats.expansions == 1 && direct.tree.size() <= 2,
                "Direct ScLERP must never create a sampled tree.");
        seedSampling(1, false);
        seedRRTSampling(1);
        require(goal.isApprox(planner.sampleGoal(), 1e-14), "Seed reset must reproduce the full goal pose.");
        const auto first = planner.plan(goal, PlannerMode::SampledScLERP);
        seedSampling(1, false);
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

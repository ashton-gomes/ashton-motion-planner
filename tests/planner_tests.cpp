#include "planner.hpp"
#include "robots.hpp"

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
        // Files named g1_*.xml include the G1; the others include the Kinova Gen3.
        const bool g1 = name.rfind("g1_", 0) == 0;
        MotionPlanner planner(g1 ? robots::g1RightArm() : robots::kinovaGen3(),
                              entry.path().string());
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

// Plan with one arm of the G1 humanoid while its legs, waist, and other arm
// are held still, in the obstacle-free G1 environment.
void checkG1Arms() {
    const std::string scene = (std::filesystem::path(ENVIRONMENTS_PATH) / "g1_empty.xml").string();
    for (const RobotConfig& robot : {robots::g1RightArm(), robots::g1LeftArm()}) {
        seedSampling(1, false);
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
    }
}

int main() {
    try {
        checkEnvironments();
        checkG1Arms();
        seedSampling(1, false);
        seedRRTSampling(1);
        MotionPlanner planner(robots::kinovaGen3(),
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

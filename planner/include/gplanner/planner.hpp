#pragma once

#include "gplanner/robot_config.hpp"
#include "gplanner/rrt_star.hpp"

#include <memory>
#include <optional>
#include <string>
#include <vector>

// Count the first reason each candidate edge was rejected.
struct PlannerDebugStats {
    int expansions = 0;
    int no_progress = 0;
    int non_finite = 0;
    int self_collision = 0;
    int obstacle_collision = 0;
    int rmrc_not_converged = 0;
    int joint_path_collision = 0;
    int joint_limit = 0; // RMRC step left a planned joint's position limits
};

using Pose = Eigen::Matrix<double, 7, 1>;

enum class PlannerMode { DirectScLERP, SampledScLERP };

struct PlanResult {
    bool success = false; // Goal reached under the planner's collision checks.
    std::string failure_reason;
    PlannerDebugStats stats;
    std::vector<RRTNode> tree;
    // One value per planned joint (see RobotConfig::joint_names). Use
    // MotionPlanner::toSceneTrajectory before handing it to the viewer.
    std::vector<Eigen::VectorXd> trajectory;
    double planning_seconds = 0.0;
    double position_error = 0.0;
    double rotation_error = 0.0;
};

struct PathValidation {
    bool valid = false;
    bool joint_limits_ok = true;
    bool collision_free = true;
    double position_error = 0.0;
    double rotation_error = 0.0;
};

// Owns reusable kinematic/collision models. Planning has no UI or file output.
class MotionPlanner {
public:
    // scene_xml must contain the robot's model, e.g. through <include>.
    // edit_scene, if set, is applied to each copy of the scene the planner loads.
    MotionPlanner(const RobotConfig& robot, const std::string& scene_xml,
                  const RRTStarSettings& settings = {},
                  const SceneEditor& edit_scene = {});
    ~MotionPlanner();
    const RobotConfig& robot() const;
    // Number of planned joints.
    int dof() const;
    // Full scene qpos for one planned-joint vector: held joints keep their
    // fixed values. The viewer needs this when the scene has extra joints.
    Eigen::VectorXd toSceneQpos(const Eigen::VectorXd& joints) const;
    std::vector<Eigen::VectorXd> toSceneTrajectory(
        const std::vector<Eigen::VectorXd>& trajectory) const;
    // Planning start: the scene's "start" keyframe if present, else robot "home".
    const Pose& homePose() const;
    // Fixed goal from the scene's "goal" keyframe, or nullopt for random-goal scenes.
    const std::optional<Pose>& sceneGoal() const;
    Pose sampleGoal();
    PlanResult plan(const Pose& goal, PlannerMode mode);
    // Dense checks of playback, including joint limits and independent MuJoCo FK.
    PathValidation validatePath(const PlanResult& result, const Pose& goal,
                                int checks_per_segment = 240);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

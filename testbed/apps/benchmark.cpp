#include "gplanner/planner.hpp"
#include "testbed/obstacle_mode.hpp"
#include "testbed/robots.hpp"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>

namespace {
struct Condition {
    std::string name;
    std::string scene;
    PlannerMode mode;
};

const std::vector<Condition> conditions = {
    {"direct_empty", "empty.xml", PlannerMode::DirectScLERP},
    {"sampled_empty", "empty.xml", PlannerMode::SampledScLERP},
    {"sampled_random_cylinders", "random_cylinders.xml", PlannerMode::SampledScLERP},
    // Fixed-task scenes: every trial uses the scene's start and goal; only the
    // planner's random sampling changes between trials.
    {"sampled_wall_hole", "wall_hole.xml", PlannerMode::SampledScLERP},
    {"sampled_long_cylinders", "long_cylinders.xml", PlannerMode::SampledScLERP},
    {"sampled_under_table", "under_table.xml", PlannerMode::SampledScLERP},
    {"sampled_shelf", "shelf.xml", PlannerMode::SampledScLERP},
};

struct Options {
    int trials = 1000;
    std::uint32_t seed = 1;
    int max_expansions = 1000;
    int validation_checks = 240;
    std::string condition = "all";
    std::string obstacles = "known";
    std::filesystem::path output = "benchmark-results";
};

unsigned long long number(const std::string& value, unsigned long long maximum, bool allow_zero = false) {
    if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos)
        throw std::invalid_argument("Expected an unsigned integer, received: " + value);
    const auto parsed = std::stoull(value);
    if ((!allow_zero && parsed == 0) || parsed > maximum)
        throw std::invalid_argument("Numeric option is out of range: " + value);
    return parsed;
}

Options parseOptions(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        if (key == "--help") {
            std::cout << "Usage: benchmark [--trials 1000] [--seed 1] [--output benchmark-results]\n"
                         "                 [--condition all|direct_empty|sampled_empty|sampled_random_cylinders|\n"
                         "                              sampled_wall_hole|sampled_long_cylinders|\n"
                         "                              sampled_under_table|sampled_shelf]\n"
                         "                 [--max-expansions 1000] [--validation-checks 240]\n"
                         "                 [--obstacles known|sensed|both]\n"
                         "Runs N trials per selected condition; all = 7N trials. Existing CSVs are preserved.\n"
                         "--obstacles: plan around the scene's known geometry, a simulated depth-camera\n"
                         "point cloud, or both (paired: same goals and seeds). Paths are always validated\n"
                         "against the scene's true geometry.\n";
            std::exit(0);
        }
        if (i + 1 == argc) throw std::invalid_argument("Missing value for " + key);
        const std::string value = argv[++i];
        const auto int_max = static_cast<unsigned long long>(std::numeric_limits<int>::max());
        if (key == "--trials") options.trials = number(value, int_max);
        else if (key == "--seed") options.seed = number(value, UINT32_MAX, true);
        else if (key == "--max-expansions") options.max_expansions = number(value, int_max);
        else if (key == "--validation-checks") options.validation_checks = number(value, int_max);
        else if (key == "--output") options.output = value;
        else if (key == "--condition") options.condition = value;
        else if (key == "--obstacles") options.obstacles = value;
        else throw std::invalid_argument("Unknown option: " + key);
    }
    if (static_cast<unsigned long long>(options.seed) + options.trials - 1 > UINT32_MAX)
        throw std::invalid_argument("Trial seeds would overflow uint32.");
    if (options.condition != "all" && std::none_of(conditions.begin(), conditions.end(),
        [&](const Condition& c) { return c.name == options.condition; }))
        throw std::invalid_argument("Unknown condition: " + options.condition);
    if (options.obstacles != "known" && options.obstacles != "sensed" && options.obstacles != "both")
        throw std::invalid_argument("Unknown --obstacles value: " + options.obstacles);
    return options;
}

std::string csv(const std::string& value) {
    std::string result = "\"";
    for (char c : value) result += c == '"' ? "\"\"" : std::string(1, c);
    return result + '"';
}

struct Summary {
    int trials = 0, successes = 0, valid_paths = 0;
    double seconds = 0;
    std::map<std::string, int> failures;
};
} // namespace

int main(int argc, char** argv) {
    try {
        const Options options = parseOptions(argc, argv);
        std::filesystem::create_directories(options.output);
        for (const auto* name : {"trials.csv", "summary.csv", "settings.csv"}) {
            if (std::filesystem::exists(options.output / name))
                throw std::runtime_error("Output already exists; choose a new --output directory: " +
                                         (options.output / name).string());
        }
        std::ofstream trials(options.output / "trials.csv"), summary(options.output / "summary.csv"),
                      metadata(options.output / "settings.csv");
        if (!trials || !summary || !metadata) throw std::runtime_error("Cannot open result files.");
        trials << std::setprecision(17);
        summary << std::setprecision(10);
        RRTStarSettings settings;
        settings.max_expansions = options.max_expansions;
        // To benchmark another arm, change robots::active() in testbed/src/robots.cpp.
        const robots::TestRobot test_robot = robots::active();
        const RobotConfig& robot = test_robot.config;
        metadata << "setting,value\ntrials_per_condition," << options.trials
                 << "\nfirst_seed," << options.seed << "\ncondition," << options.condition
                 << "\nrobot," << csv(robot.name)
                 << "\nrobot_xml," << csv(robot.model_path)
                 << "\nenvironments_directory," << csv(ENVIRONMENTS_PATH)
                 << "\nuse_rrt_star," << settings.use_rrt_star
                 << "\nuse_goal_bias," << settings.use_goal_bias
                 << "\ngoal_bias," << settings.goal_bias
                 << "\nsamples_per_expansion," << settings.samples_per_expansion
                 << "\nmax_expansions," << settings.max_expansions
                 << "\nmaximum_step_m," << settings.maximum_step
                 << "\nrewire_radius_m," << settings.rewire_radius
                 << "\nrmrc_iterations_per_edge," << settings.rmrc_iterations_per_edge
                 << "\nsclerp_steps_per_edge," << settings.sclerp_steps_per_edge
                 << "\nanimation_frames_per_edge," << settings.animation_frames_per_edge
                 << "\nobstacle_clearance_m," << settings.obstacle_clearance
                 << "\nobstacle_sample_attempts," << settings.obstacle_sample_attempts
                 << "\nvalidation_checks_per_edge," << options.validation_checks
                 << "\nposition_tolerance_m,0.001\nrotation_tolerance_rad,0.001"
                    "\nrmrc_beta,0.1\nrandomize_cylinders,0"
                 << "\nobstacles," << options.obstacles;
        std::vector<ObstacleMode> modes;
        if (options.obstacles != "sensed") modes.push_back(ObstacleMode::KnownGeometry);
        if (options.obstacles != "known") modes.push_back(ObstacleMode::SimulatedSensor);
        const PointCloudSettings cloud_settings =
            obstacleOptions(ObstacleMode::SimulatedSensor).point_cloud;
        if (options.obstacles != "known") {
            metadata << "\nvoxel_size_m," << cloud_settings.voxel_size
                     << "\nmin_points_per_voxel," << cloud_settings.min_points_per_voxel
                     << "\nfloor_height_m," << cloud_settings.floor_height
                     << "\ncrop_margin_m," << cloud_settings.crop_margin
                     << "\nself_filter_padding_m," << cloud_settings.self_filter_padding;
            for (const DepthCameraSettings& camera : test_robot.cameras)
                metadata << "\ncamera_" << camera.name << ",\"pos " << camera.position.transpose()
                         << " look_at " << camera.look_at.transpose() << " fov " << camera.horizontal_fov_deg
                         << "x" << camera.vertical_fov_deg << " res " << camera.width << "x" << camera.height
                         << " range " << camera.min_range << "-" << camera.max_range
                         << " noise " << camera.noise_base << "+" << camera.noise_per_m2
                         << "d^2 dropout " << camera.dropout << '"';
        }
        metadata << '\n';
        metadata.flush();
        trials << "condition,scene,obstacles,trial,seed,success,valid_path_success,failure_reason,planning_seconds,"
                  "nodes,expansions,position_error_m,rotation_error_rad,joint_limits_ok,collision_free,"
                  "validated_position_error_m,validated_rotation_error_rad,no_progress,non_finite,"
                  "self_collision,obstacle_collision,rmrc_not_converged,joint_path_collision,joint_limit,"
                  "goal_x,goal_y,goal_z,goal_w,goal_qx,goal_qy,goal_qz\n";
        summary << "condition,scene,obstacles,trials,successes,success_rate_percent,valid_paths,"
                   "valid_path_rate_percent,mean_planning_seconds,obstacle_boxes,cloud_points\n";
        for (const auto& condition : conditions) {
            if (options.condition != "all" && options.condition != condition.name) continue;
            // Models are loaded once per fixed scene; all per-trial randomness
            // is reset before sampling the goal. Empty-space tasks are paired.
            // This robot's version of the scene, e.g. g1_wall_hole.xml.
            const std::string scene = test_robot.environment_prefix + condition.scene;
            const std::string scene_path = (std::filesystem::path(ENVIRONMENTS_PATH) / scene).string();
            seedSampling(options.seed);
            seedRRTSampling(options.seed);
            // The true scene: known-geometry planning, every goal, and every
            // path validation, so sensed results are judged against reality.
            MotionPlanner truth(robot, scene_path, settings);
            std::unique_ptr<MotionPlanner> sensed;
            size_t cloud_points = 0;
            if (options.obstacles != "known") {
                sensed = std::make_unique<MotionPlanner>(robot, scene_path, settings,
                    obstacleOptions(ObstacleMode::SimulatedSensor));
                cloud_points = senseObstacles(*sensed, test_robot, scene_path, {}, options.seed).points.size();
            }
            std::map<ObstacleMode, Summary> totals;
            for (int trial = 0; trial < options.trials; ++trial) {
                const std::uint32_t seed = options.seed + static_cast<std::uint32_t>(trial);
                for (const ObstacleMode mode : modes) {
                    MotionPlanner& planner = mode == ObstacleMode::KnownGeometry ? truth : *sensed;
                    Summary& total = totals[mode];
                    seedSampling(seed);
                    seedRRTSampling(seed);
                    Pose goal;
                    bool goal_sampled = false;
                    PlanResult result;
                    PathValidation validation;
                    try {
                        goal = truth.sceneGoal() ? *truth.sceneGoal() : truth.sampleGoal();
                        goal_sampled = true;
                        result = planner.plan(goal, condition.mode);
                        validation = truth.validatePath(result, goal, options.validation_checks);
                    } catch (const std::exception& error) {
                        result.success = false;
                        result.failure_reason = std::string(goal_sampled ? "trial_error: " : "goal_sampling: ") + error.what();
                    }
                    ++total.trials;
                    total.successes += result.success;
                    total.valid_paths += validation.valid;
                    total.seconds += result.planning_seconds;
                    if (!result.success) ++total.failures[result.failure_reason];
                    const auto& s = result.stats;
                    trials << condition.name << ',' << scene << ',' << obstacleModeName(mode) << ','
                           << trial + 1 << ',' << seed << ','
                           << result.success << ',' << validation.valid << ',' << csv(result.failure_reason) << ','
                           << result.planning_seconds << ',' << result.tree.size() << ',' << s.expansions << ',';
                    if (goal_sampled) trials << result.position_error << ',' << result.rotation_error;
                    else trials << ',';
                    trials << ',';
                    // Leave validation fields empty when no complete path exists.
                    if (result.success) trials << validation.joint_limits_ok << ',' << validation.collision_free
                        << ',' << validation.position_error << ',' << validation.rotation_error;
                    else trials << ",,,";
                    trials << ',' << s.no_progress << ',' << s.non_finite << ',' << s.self_collision << ','
                           << s.obstacle_collision << ',' << s.rmrc_not_converged << ',' << s.joint_path_collision
                           << ',' << s.joint_limit;
                    for (int i = 0; i < 7; ++i) {
                        trials << ',';
                        if (goal_sampled) trials << goal[i];
                    }
                    trials << '\n';
                    trials.flush();
                    if (!trials) throw std::runtime_error("Failed writing trial results.");
                    if ((trial + 1) % 25 == 0 || trial + 1 == options.trials)
                        std::cout << condition.name << " (" << obstacleModeName(mode) << "): " << trial + 1
                                  << '/' << options.trials << ", goal reached " << total.successes
                                  << ", valid paths " << total.valid_paths << std::endl;
                }
            }
            for (const ObstacleMode mode : modes) {
                const Summary& total = totals[mode];
                const MotionPlanner& planner = mode == ObstacleMode::KnownGeometry ? truth : *sensed;
                const double rate = 100.0 * total.successes / total.trials;
                const double valid_rate = 100.0 * total.valid_paths / total.trials;
                summary << condition.name << ',' << scene << ',' << obstacleModeName(mode) << ','
                        << total.trials << ',' << total.successes << ',' << rate << ','
                        << total.valid_paths << ',' << valid_rate << ',' << total.seconds / total.trials
                        << ',' << planner.obstacles().size() << ',';
                if (mode == ObstacleMode::SimulatedSensor) summary << cloud_points;
                summary << '\n';
                summary.flush();
                if (!summary) throw std::runtime_error("Failed writing summary.");
                std::cout << condition.name << " (" << obstacleModeName(mode) << "): success " << rate
                          << "%, validated paths " << valid_rate << "%\n";
                for (const auto& [reason, count] : total.failures)
                    std::cout << "  " << reason << ": " << count << '\n';
            }
        }
        std::cout << "Results saved to " << options.output << '\n';
        return 0; // Individual planner failures are benchmark data, not runner errors.
    } catch (const std::exception& error) {
        std::cerr << "Benchmark error: " << error.what() << '\n';
        return 1;
    }
}

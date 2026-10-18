#include "planner.hpp"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
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
    {"sampled_obstacles_1", "obstacles_1.xml", PlannerMode::SampledScLERP},
    {"sampled_obstacles_2", "obstacles_2.xml", PlannerMode::SampledScLERP},
    {"sampled_obstacles_3", "obstacles_3.xml", PlannerMode::SampledScLERP},
};

struct Options {
    int trials = 1000;
    std::uint32_t seed = 1;
    int max_expansions = 1000;
    int validation_checks = 240;
    std::string condition = "all";
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
                         "                 [--condition all|direct_empty|sampled_empty|sampled_obstacles_1|\n"
                         "                              sampled_obstacles_2|sampled_obstacles_3]\n"
                         "                 [--max-expansions 1000] [--validation-checks 240]\n"
                         "Runs N trials per selected condition; all = 5N trials. Existing CSVs are preserved.\n";
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
        else throw std::invalid_argument("Unknown option: " + key);
    }
    if (static_cast<unsigned long long>(options.seed) + options.trials - 1 > UINT32_MAX)
        throw std::invalid_argument("Trial seeds would overflow uint32.");
    if (options.condition != "all" && std::none_of(conditions.begin(), conditions.end(),
        [&](const Condition& c) { return c.name == options.condition; }))
        throw std::invalid_argument("Unknown condition: " + options.condition);
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
        metadata << "setting,value\ntrials_per_condition," << options.trials
                 << "\nfirst_seed," << options.seed << "\ncondition," << options.condition
                 << "\nrobot_xml," << csv(ROBOT_XML_PATH)
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
                    "\nrmrc_beta,0.1\nrandomize_cylinders,0\n";
        metadata.flush();
        trials << "condition,scene,trial,seed,success,valid_path_success,failure_reason,planning_seconds,"
                  "nodes,expansions,position_error_m,rotation_error_rad,joint_limits_ok,collision_free,"
                  "validated_position_error_m,validated_rotation_error_rad,no_progress,non_finite,"
                  "self_collision,obstacle_collision,rmrc_not_converged,joint_path_collision,"
                  "goal_x,goal_y,goal_z,goal_w,goal_qx,goal_qy,goal_qz\n";
        summary << "condition,scene,trials,successes,success_rate_percent,valid_paths,"
                   "valid_path_rate_percent,mean_planning_seconds\n";
        for (const auto& condition : conditions) {
            if (options.condition != "all" && options.condition != condition.name) continue;
            // Models are loaded once per fixed scene; all per-trial randomness
            // is reset before sampling the goal. Empty-space tasks are paired.
            seedSampling(options.seed, false);
            seedRRTSampling(options.seed);
            MotionPlanner planner(ROBOT_XML_PATH,
                (std::filesystem::path(ENVIRONMENTS_PATH) / condition.scene).string(), settings);
            Summary totals;
            for (int trial = 0; trial < options.trials; ++trial) {
                const std::uint32_t seed = options.seed + static_cast<std::uint32_t>(trial);
                seedSampling(seed, false);
                seedRRTSampling(seed);
                Pose goal;
                bool goal_sampled = false;
                PlanResult result;
                PathValidation validation;
                try {
                    goal = planner.sampleGoal();
                    goal_sampled = true;
                    result = planner.plan(goal, condition.mode);
                    validation = planner.validatePath(result, goal, options.validation_checks);
                } catch (const std::exception& error) {
                    result.success = false;
                    result.failure_reason = std::string(goal_sampled ? "trial_error: " : "goal_sampling: ") + error.what();
                }
                ++totals.trials;
                totals.successes += result.success;
                totals.valid_paths += validation.valid;
                totals.seconds += result.planning_seconds;
                if (!result.success) ++totals.failures[result.failure_reason];
                const auto& s = result.stats;
                trials << condition.name << ',' << condition.scene << ',' << trial + 1 << ',' << seed << ','
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
                       << s.obstacle_collision << ',' << s.rmrc_not_converged << ',' << s.joint_path_collision;
                for (int i = 0; i < 7; ++i) {
                    trials << ',';
                    if (goal_sampled) trials << goal[i];
                }
                trials << '\n';
                trials.flush();
                if (!trials) throw std::runtime_error("Failed writing trial results.");
                if ((trial + 1) % 25 == 0 || trial + 1 == options.trials)
                    std::cout << condition.name << ": " << trial + 1 << '/' << options.trials
                              << ", goal reached " << totals.successes << ", valid paths "
                              << totals.valid_paths << std::endl;
            }
            const double rate = 100.0 * totals.successes / totals.trials;
            const double valid_rate = 100.0 * totals.valid_paths / totals.trials;
            summary << condition.name << ',' << condition.scene << ',' << totals.trials << ','
                    << totals.successes << ',' << rate << ',' << totals.valid_paths << ','
                    << valid_rate << ',' << totals.seconds / totals.trials << '\n';
            summary.flush();
            if (!summary) throw std::runtime_error("Failed writing summary.");
            std::cout << condition.name << ": success " << rate << "%, validated paths " << valid_rate << "%\n";
            for (const auto& [reason, count] : totals.failures)
                std::cout << "  " << reason << ": " << count << '\n';
        }
        std::cout << "Results saved to " << options.output << '\n';
        return 0; // Individual planner failures are benchmark data, not runner errors.
    } catch (const std::exception& error) {
        std::cerr << "Benchmark error: " << error.what() << '\n';
        return 1;
    }
}

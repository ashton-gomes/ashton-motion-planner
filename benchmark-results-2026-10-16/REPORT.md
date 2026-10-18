# ScLERP benchmark results — October 16, 2026

The October 9 experiment was implemented and run with 1,000 trials for each of five conditions (5,000 trials total), using seeds 1–1000. All requested trials are present; no planner failures were discarded.

| Condition | Trials | Goal reached | Fully validated path |
| --- | ---: | ---: | ---: |
| Direct ScLERP, empty | 1000 | 836/1000 (83.6%) | 717/1000 (71.7%) |
| Node sampling, empty | 1000 | 991/1000 (99.1%) | 586/1000 (58.6%) |
| Node sampling, one cylinder | 1000 | 985/1000 (98.5%) | 493/1000 (49.3%) |
| Node sampling, one box | 1000 | 978/1000 (97.8%) | 624/1000 (62.4%) |
| Node sampling, two cylinders | 1000 | 946/1000 (94.6%) | 538/1000 (53.8%) |

Goal reaching uses the planner’s 1 mm and 0.001 rad endpoint thresholds and collision checks. Fully validated paths additionally pass dense playback collision checks, MuJoCo joint bounds, and independent MuJoCo endpoint kinematics. The two empty-space conditions use identical goal poses for each seed.

Sampled empty-space planning reached 99.1% of goals. Direct ScLERP reached 83.6%; a reachable endpoint does not guarantee a self-collision-free screw path or RMRC convergence. The current RMRC solver does not enforce joint limits, so endpoint completion rates exceed fully validated path rates.

| Condition | Planning failures | Reached paths violating joint limits | Reached paths failing dense collision checks |
| --- | ---: | ---: | ---: |
| Direct ScLERP, empty | 164 | 119 | 0 |
| Node sampling, empty | 9 | 405 | 0 |
| Node sampling, one cylinder | 15 | 492 | 0 |
| Node sampling, one box | 22 | 354 | 0 |
| Node sampling, two cylinders | 54 | 408 | 0 |

Direct-edge rejection counts: self_collision = 164, obstacle_collision = 0, rmrc_not_converged = 0, joint_path_collision = 0, non_finite = 0. All sampled-planner failures exhausted the 1,000-expansion budget. These outcomes do not establish that the sampled goals are unreachable.

The full run used eight independent local processes in batches of 100 trials. Seeds were reset per trial, and models used fixed obstacle placements. Planning times in the CSVs were measured under this concurrent workload and should not be interpreted as isolated serial latency. The prior partial serial run’s non-timing fields matched the corresponding completed trials exactly.

Verified: exactly 1,000 unique seeds per condition, all summary percentages against raw rows, all 1,000 paired empty-space goals, direct-mode tree sizes of at most two (start/end), and endpoint/validation thresholds. The C++ planner and Python benchmark integration tests passed.

Files: [trials.csv](trials.csv), [summary.csv](summary.csv), [settings.csv](settings.csv). See [BENCHMARK.md](../BENCHMARK.md) for implementation details, collision semantics, and the normal sequential run command. The local batch reproduction script is [run_parallel.py](run_parallel.py); copy it into a fresh sibling results directory to rerun without overwriting this experiment.

# Ashton Motion Planner

Motion planner using unit dual quaternions, screw linear interpolation (ScLERP),
resolved motion rate control (RMRC), and RRT*.

## Requirements

- CMake 3.10+ and a C++17 compiler
- Eigen 3
- MuJoCo, installed at `~/mujoco-install` by default (override with
  `-DMUJOCO_ROOT=/path/to/mujoco`)
- GLFW 3 and OpenCV (core, imgproc, videoio). These are only needed for the
  interactive viewer; pass `-DBUILD_VIEWER=OFF` to skip it.

## Quick start

Clone with submodules (the Unitree G1 model comes from `unitree_ros`):

```bash
git clone --recurse-submodules https://github.com/ashton-gomes/ashton-motion-planner
cd ashton-motion-planner
```

If you already cloned without `--recurse-submodules`, run
`git submodule update --init`.

Build:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

Run the interactive planner, then select a scenario:

```bash
./build/testing
```

Run the tests:

```bash
ctest --test-dir build --output-on-failure
```

## Project layout

| Path | Contents |
| --- | --- |
| `include/`, `source/` | Planner library (`planner_core`), viewer, and executables |
| `tests/` | C++ planner tests and benchmark smoke tests (run with `ctest`) |
| `environments/` | MuJoCo scene XMLs for the Kinova Gen3 and G1 |
| `benchmark/` | Parallel benchmark runner and recorded results; see [BENCHMARK.md](BENCHMARK.md) |
| `docs/` | Design notes, e.g. [modular robots](docs/modular_robots.md) |
| `scripts/` | Bash shortcuts |
| `mujoco_menagerie/` | Vendored Kinova Gen3 model |
| `unitree_ros/` | Unitree G1 model (git submodule) |

## Choosing the robot

The robot arm is set in `robots::active()` in `source/robots.cpp` (Kinova Gen3
by default; G1 left and right arms are also available). See
[docs/modular_robots.md](docs/modular_robots.md) for how to switch robots or
add a new one.

## Benchmark

See [BENCHMARK.md](BENCHMARK.md) for the experiment design, how to run it, and
results.

## Bash shortcuts

From the repository root, load the G1 viewer shortcut:

```bash
source scripts/bash_aliases.sh
g1
```

`g1` opens the Unitree G1 29-DOF model in MuJoCo's C++ `simulate` viewer.
The `unitree_ros` submodule must be checked out. The model path is resolved
relative to this repository. MuJoCo defaults to `~/mujoco-install`; set
`MUJOCO_ROOT` before sourcing the file to use another installation.

To load the shortcut in every Bash terminal, add the following line to
`~/.bashrc`, replacing `/path/to/ashton-motion-planner` with your clone's path:

```bash
source /path/to/ashton-motion-planner/scripts/bash_aliases.sh
```

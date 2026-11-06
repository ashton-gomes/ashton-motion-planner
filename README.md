Motion Planner using Unit Dual Quaternions, Screw Linear Interpolation (ScLERP), Resolved motion rate control (RMRC), and RRT. 

## Quick start

From the project directory, build and check the program:

To start, first clone: 
```bash 
git clone https://github.com/ashton-gomes/ashton-motion-planner'
```

Then:
```bash
mkdir build
cd build
cmake ..
make -j$(nproc)
```

To run the program, do 
```bash 
./testing
```
From there, you can select the scenario to run.

## Choosing the robot

The robot arm is set in `robots::active()` in `source/robots.cpp` (Kinova Gen3
by default; G1 left and right arms are also available). See
[docs/modular_robots.md](docs/modular_robots.md) for how to switch robots or
add a new one.

## Bash shortcuts

From the repository root, load the G1 viewer shortcut:

```bash
source scripts/bash_aliases.sh
g1
```

`g1` opens the Unitree G1 29-DOF model in MuJoCo's C++ `simulate` viewer.
The `unitree_ros` model checkout must be present in the repository directory.
The model path is resolved relative to this repository. MuJoCo defaults to
`~/mujoco-install`; set `MUJOCO_ROOT` before sourcing the file to use another
installation.

To load the shortcut in every Bash terminal, add the following line to
`~/.bashrc`, replacing `/path/to/ashton-motion-planner` with your clone's path
(currently `/home/ash_gomes/gplanner` on this machine):

```bash
source /path/to/ashton-motion-planner/scripts/bash_aliases.sh
```

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
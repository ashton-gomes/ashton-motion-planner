cmake_minimum_required(VERSION 3.10)
project(gplanner CXX)

# Enforce C++17 (required for std::clamp and modern Eigen features)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Find Eigen3
find_package(Eigen3 REQUIRED)

# 2. Find GLFW3
find_package(glfw3 REQUIRED)

# 3. Setup MuJoCo Manually
set(MUJOCO_ROOT "/home/ash_gomes/mujoco-install")

# Find the shared library (libmujoco.so) directly
find_library(MUJOCO_LIB NAMES mujoco PATHS "${MUJOCO_ROOT}/lib" REQUIRED NO_DEFAULT_PATH)

# 4. Define source files
set(SOURCES
    src/main.cpp
    src/viewer.cpp
    src/kinematics.cpp
    src/sampler.cpp
    src/planner.cpp
)

# 5. Create the executable
add_executable(my_program ${SOURCES})

# 6. Add include directories for your headers and MuJoCo
target_include_directories(my_program PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${MUJOCO_ROOT}/include"
)

# 7. Link all required libraries
target_link_libraries(my_program
    PRIVATE
    Eigen3::Eigen
    ${MUJOCO_LIB}
    glfw
)

# 8. Add RPATH so Linux can find libmujoco.so at runtime
set_target_properties(my_program PROPERTIES
    BUILD_RPATH "${MUJOCO_ROOT}/lib"
    INSTALL_RPATH "${MUJOCO_ROOT}/lib"
)
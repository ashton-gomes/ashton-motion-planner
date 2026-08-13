cmake_minimum_required(VERSION 3.10)
project(gplanner)

# 1. Find Eigen3
find_package(Eigen3 REQUIRED)

# 2. Find GLFW3
find_package(glfw3 REQUIRED)

# 3. Setup MuJoCo Manually
set(MUJOCO_ROOT "/home/ash_gomes/mujoco-install")

# Find the shared library (libmujoco.so) directly
find_library(MUJOCO_LIB NAMES mujoco PATHS "${MUJOCO_ROOT}/lib" REQUIRED NO_DEFAULT_PATH)

# 4. Create the executable
add_executable(my_program main_new.cpp)

# 5. Tell the compiler where to find <mujoco/mujoco.h>
target_include_directories(my_program PRIVATE "${MUJOCO_ROOT}/include")

# 6. Link all required libraries
target_link_libraries(my_program
    PRIVATE
    Eigen3::Eigen
    ${MUJOCO_LIB}
    glfw
)

# 7. Add RPATH so Linux can find libmujoco.so at runtime
set_target_properties(my_program PROPERTIES
    BUILD_RPATH "${MUJOCO_ROOT}/lib"
    INSTALL_RPATH "${MUJOCO_ROOT}/lib"
)
cmake_minimum_required(VERSION 3.10)
project(kinova_planner CXX)

# Enforce C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Eigen3
find_package(Eigen3 REQUIRED)
find_package(glfw3 REQUIRED)

# 2. Setup MuJoCo Manually
set(MUJOCO_ROOT "/home/ash_gomes/mujoco-install")
find_library(MUJOCO_LIB NAMES mujoco PATHS "${MUJOCO_ROOT}/lib" REQUIRED NO_DEFAULT_PATH)

# Path to the Kinova Gen 3 model in your repo (can be .xml or .urdf)
set(KINOVA_MODEL_PATH "/home/ash_gomes/gplanner/mujoco_menagerie/kinova_gen3/gen3.xml")

# 3. Define target
add_executable(testing
    source/main.cpp
    source/rrt_star.cpp
    source/sampler.cpp
    source/viewer.cpp
)

# 4. Include directories
target_include_directories(testing PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${MUJOCO_ROOT}/include"
    ${EIGEN3_INCLUDE_DIRS}
)

# 5. Link libraries
target_link_libraries(testing PRIVATE
    Eigen3::Eigen
    glfw
    ${MUJOCO_LIB}
)

# 6. Pass the model path to C++
target_compile_definitions(testing PRIVATE
    ROBOT_XML_PATH="${KINOVA_MODEL_PATH}"
    SCENE_XML_PATH="${CMAKE_CURRENT_SOURCE_DIR}/scene.xml"
)

# 7. Add RPATH so Linux can find libmujoco.so at runtime without LD_LIBRARY_PATH
set_target_properties(testing PROPERTIES
    BUILD_RPATH "${MUJOCO_ROOT}/lib"
    INSTALL_RPATH "${MUJOCO_ROOT}/lib"
)

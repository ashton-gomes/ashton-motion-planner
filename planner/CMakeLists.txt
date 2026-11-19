# gplanner: the motion planner library. It depends only on Eigen and MuJoCo
# (headless: model loading, kinematics, and collision checks), so another
# project can use it with add_subdirectory(planner) and link gplanner::planner.

find_package(Eigen3 REQUIRED)

if(NOT TARGET mujoco::mujoco)
    set(MUJOCO_ROOT "$ENV{HOME}/mujoco-install" CACHE PATH "MuJoCo installation")
    find_library(MUJOCO_LIB NAMES mujoco PATHS "${MUJOCO_ROOT}/lib" REQUIRED NO_DEFAULT_PATH)
    add_library(mujoco::mujoco UNKNOWN IMPORTED)
    set_target_properties(mujoco::mujoco PROPERTIES
        IMPORTED_LOCATION "${MUJOCO_LIB}"
        INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_ROOT}/include")
endif()

add_library(gplanner src/planner.cpp src/rrt_star.cpp src/sampler.cpp src/obstacles.cpp)
add_library(gplanner::planner ALIAS gplanner)
target_include_directories(gplanner PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(gplanner PUBLIC Eigen3::Eigen mujoco::mujoco)
target_compile_features(gplanner PUBLIC cxx_std_17)

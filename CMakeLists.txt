cmake_minimum_required(VERSION 3.10)
project(kinova_planner CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Eigen3 REQUIRED)

set(MUJOCO_ROOT "/home/ash_gomes/mujoco-install" CACHE PATH "MuJoCo installation")
find_library(MUJOCO_LIB NAMES mujoco PATHS "${MUJOCO_ROOT}/lib" REQUIRED NO_DEFAULT_PATH)
set(KINOVA_MODEL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/mujoco_menagerie/kinova_gen3/gen3.xml")

add_library(planner_core source/planner.cpp source/rrt_star.cpp source/sampler.cpp)
target_include_directories(planner_core PUBLIC include "${MUJOCO_ROOT}/include")
target_link_libraries(planner_core PUBLIC Eigen3::Eigen ${MUJOCO_LIB})
set(CMAKE_BUILD_RPATH "${MUJOCO_ROOT}/lib")

add_executable(benchmark source/benchmark.cpp)
target_link_libraries(benchmark PRIVATE planner_core)
target_compile_definitions(benchmark PRIVATE
    ROBOT_XML_PATH="${KINOVA_MODEL_PATH}"
    ENVIRONMENTS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/environments")

option(BUILD_VIEWER "Build the interactive planner and GLFW viewer" ON)
if(BUILD_VIEWER)
    find_package(glfw3 REQUIRED)
    add_executable(testing source/main.cpp source/environment_selector.cpp
                           source/planner_debug.cpp source/viewer.cpp)
    target_link_libraries(testing PRIVATE planner_core glfw)
    target_compile_definitions(testing PRIVATE
        ROBOT_XML_PATH="${KINOVA_MODEL_PATH}"
        SCENE_XML_PATH="${CMAKE_CURRENT_SOURCE_DIR}/environments/wall_hole.xml")
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(planner_tests tests/planner_tests.cpp)
    target_link_libraries(planner_tests PRIVATE planner_core)
    target_compile_definitions(planner_tests PRIVATE
        ROBOT_XML_PATH="${KINOVA_MODEL_PATH}"
        ENVIRONMENTS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/environments")
    add_test(NAME planner_tests COMMAND planner_tests)
    find_package(Python3 COMPONENTS Interpreter QUIET)
    if(Python3_Interpreter_FOUND)
        add_test(NAME benchmark_tests COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/benchmark_tests.py" $<TARGET_FILE:benchmark>)
    endif()
endif()

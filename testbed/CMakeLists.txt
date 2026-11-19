# Testbed: ready-made robots, scenes, the benchmark, and the interactive viewer
# for trying out gplanner. The planner library does not depend on any of this.

# Robots, scene randomization, scene selection, and the simulated depth
# sensor (no graphics).
add_library(testbed_core src/robots.cpp src/scene_randomizer.cpp
                         src/environment_selector.cpp src/sensor.cpp
                         src/obstacle_mode.cpp)
target_include_directories(testbed_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(testbed_core PUBLIC gplanner::planner)
# Robot models are listed in src/robots.cpp, relative to the repository root.
target_compile_definitions(testbed_core PRIVATE
    PROJECT_ROOT_PATH="${PROJECT_SOURCE_DIR}")

add_executable(benchmark apps/benchmark.cpp)
target_link_libraries(benchmark PRIVATE testbed_core)
target_compile_definitions(benchmark PRIVATE
    ENVIRONMENTS_PATH="${GPLANNER_ENVIRONMENTS_DIR}")

if(BUILD_VIEWER)
    find_package(glfw3 REQUIRED)
    find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio)

    # MuJoCo/GLFW viewer, video capture, and failed-plan replay.
    add_library(testbed_viewer src/viewer.cpp src/planner_debug.cpp)
    target_link_libraries(testbed_viewer PUBLIC testbed_core glfw PRIVATE ${OpenCV_LIBS})
    target_include_directories(testbed_viewer PRIVATE ${OpenCV_INCLUDE_DIRS})

    add_executable(testing apps/testing.cpp)
    target_link_libraries(testing PRIVATE testbed_viewer)
    target_compile_definitions(testing PRIVATE
        SAVED_TRIALS_PATH="${PROJECT_SOURCE_DIR}/saved_trials"
        SCENE_XML_PATH="${GPLANNER_ENVIRONMENTS_DIR}/wall_hole.xml")
endif()

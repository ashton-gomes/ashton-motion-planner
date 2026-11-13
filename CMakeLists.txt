cmake_minimum_required(VERSION 3.10)
project(gplanner CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Put every program directly in the build directory (build/testing, ...).
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

option(GPLANNER_BUILD_TESTBED "Build the testbed: robots, benchmark, tests, and viewer" ON)
option(BUILD_VIEWER "Build the interactive planner and GLFW viewer" ON)

# The motion planner library (gplanner::planner).
add_subdirectory(planner)

if(GPLANNER_BUILD_TESTBED)
    set(GPLANNER_ENVIRONMENTS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/environments")
    add_subdirectory(testbed)

    include(CTest)
    if(BUILD_TESTING)
        add_executable(planner_tests tests/planner_tests.cpp)
        target_link_libraries(planner_tests PRIVATE testbed_core)
        target_compile_definitions(planner_tests PRIVATE
            ENVIRONMENTS_PATH="${GPLANNER_ENVIRONMENTS_DIR}")
        add_test(NAME planner_tests COMMAND planner_tests)
        find_package(Python3 COMPONENTS Interpreter QUIET)
        if(Python3_Interpreter_FOUND)
            add_test(NAME benchmark_tests COMMAND "${Python3_EXECUTABLE}"
                "${CMAKE_CURRENT_SOURCE_DIR}/tests/benchmark_tests.py" $<TARGET_FILE:benchmark>)
        endif()
    endif()
endif()

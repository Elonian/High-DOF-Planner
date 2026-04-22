cmake_minimum_required(VERSION 3.16.3)
project(planning_hw2)

set(CMAKE_CXX_STANDARD 14)

#add_library(hw1 runtest.cpp planner.cpp)
add_executable(planner
        src/planner.cpp
        utils/planning_common.cpp
        src/planners/rrt.cpp
        src/planners/rrt_connect.cpp
        src/planners/rrt_star.cpp
        src/planners/prm.cpp)

add_executable(verifier src/verifier.cpp)

target_include_directories(planner PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/utils)

target_compile_definitions(planner PRIVATE MAPS_DIR="${CMAKE_SOURCE_DIR}/maps")
target_compile_definitions(planner PRIVATE OUTPUT_DIR="${CMAKE_SOURCE_DIR}/output")

target_compile_definitions(verifier PRIVATE MAPS_DIR="${CMAKE_SOURCE_DIR}/maps")
target_compile_definitions(verifier PRIVATE OUTPUT_DIR="${CMAKE_SOURCE_DIR}/output")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

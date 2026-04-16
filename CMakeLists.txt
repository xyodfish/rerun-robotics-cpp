cmake_minimum_required(VERSION 3.16)
project(rerun_humanoid_mesh_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(robot_runtime
    src/robot_model_runtime.cpp
    src/trajectory_dispatcher.cpp
)

add_executable(rerun_cpp_demo src/main_new.cpp)
add_executable(rerun_showcase_demo src/main_showcase.cpp)

include(FetchContent)

# 你现在本地 zip 的方式
FetchContent_Declare(
    rerun_sdk
    URL file://${CMAKE_SOURCE_DIR}/third_party/rerun_cpp_sdk.zip
)
FetchContent_MakeAvailable(rerun_sdk)

find_package(Eigen3 REQUIRED)
find_package(assimp REQUIRED)
find_package(pinocchio REQUIRED)
find_package(urdfdom REQUIRED)

target_include_directories(robot_runtime
    PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(robot_runtime
    PUBLIC
    rerun_sdk
    Eigen3::Eigen
    assimp
    pinocchio::pinocchio
    urdfdom::urdfdom_model
    urdfdom::urdfdom_world
    urdfdom::urdfdom_sensor
)

target_link_libraries(rerun_cpp_demo
    PRIVATE
    robot_runtime
)

target_link_libraries(rerun_showcase_demo
    PRIVATE
    robot_runtime
)

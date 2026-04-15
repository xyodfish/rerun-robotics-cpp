# cmake_minimum_required(VERSION 3.16)
# project(rerun_cpp_demo LANGUAGES CXX)

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# add_executable(rerun_cpp_demo src/main.cpp)

# include(FetchContent)
# FetchContent_Declare(
#     rerun_sdk
#     URL file://${CMAKE_SOURCE_DIR}/third_party/rerun_cpp_sdk.zip
# )
# FetchContent_MakeAvailable(rerun_sdk)

# target_link_libraries(rerun_cpp_demo PRIVATE rerun_sdk)

cmake_minimum_required(VERSION 3.16)
project(rerun_humanoid_mesh_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

target_link_libraries(rerun_cpp_demo
    PRIVATE
    rerun_sdk
    Eigen3::Eigen
    assimp
    pinocchio::pinocchio
)

target_link_libraries(rerun_showcase_demo
    PRIVATE
    rerun_sdk
    Eigen3::Eigen
    assimp
    pinocchio::pinocchio
)

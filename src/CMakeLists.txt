cmake_minimum_required(VERSION 3.10)
project(slam_fpga_competition)

set(CMAKE_CXX_STANDARD 17)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

file(GLOB SRC_FILES "*.cpp" "slam_algorithm/*.hpp")

add_executable(main main.cpp)
target_link_libraries(main ${PCL_LIBRARIES})

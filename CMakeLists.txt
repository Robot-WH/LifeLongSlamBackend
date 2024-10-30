cmake_minimum_required(VERSION 3.5)
project(lifelong_backend)

set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

set(BUILD_WITH_CHANNEL_AUTO ON)
set(BUILD_WITH_CHANNEL_ROS1 OFF)
set(BUILD_WITH_CHANNEL_ROS2 OFF)
option(BUILD_WITH_CHANNEL_PROTOBUF OFF)

add_subdirectory(lifelong_backend)
add_subdirectory(app)
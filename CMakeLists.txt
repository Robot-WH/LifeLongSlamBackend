cmake_minimum_required(VERSION 3.0.2)
project(lifelong_backend)

set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  pcl_conversions
  pcl_ros
  std_msgs
  pcl_msgs
  sensor_msgs
  geographic_msgs
  nmea_msgs
  nav_msgs
  message_generation
  ndt_omp
)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3  REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES
  # keyframe_info.msg
  control_info.msg
  trajectory_info.msg
  workspace_info.msg
  space_info.msg
)

add_service_files(FILES
  SaveMap.srv
  SaveData.srv
  SetSpace.srv
  SaveTraj.srv
  SetTraj.srv
  SetCommand.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
#   geographic_msgs
#   nmea_msgs
  # keyframe_info
  # control_info
)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  geographic_msgs
  sensor_msgs
  nmea_msgs

  message_runtime
  message_generation
)

add_subdirectory(lifelong_backend)

include_directories(
    include
    ${catkin_INCLUDE_DIRS} 
    lifelong_backend/include
    ${PCL_INCLUDE_DIRS}
)

add_executable(backend_node app/ros_wrapper.cpp)
target_link_libraries(backend_node
    ${catkin_LIBRARIES} 
    lifelong_backend
)
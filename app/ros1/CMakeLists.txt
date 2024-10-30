file(GLOB_RECURSE SOURCE_FILE CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR} *.cpp *.cc)
file(GLOB_RECURSE INCLUDE_FILE CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR} *.h *.hpp)

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
)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3  REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES
  keyframe_info.msg
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
)

# catkin_package(
#   CATKIN_DEPENDS
#   std_msgs
#   nav_msgs
#   geometry_msgs
#   geographic_msgs
#   sensor_msgs
#   nmea_msgs

#   message_runtime
#   message_generation
# )

# add_library(${TARGET_NAME} SHARED ${SOURCE_FILE} ${INCLUDE_FILE})

include_directories(
    include
    ${catkin_INCLUDE_DIRS} 
    ${BACKEND_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

add_executable(backend_node src/ros_wrapper.cpp)
target_link_libraries(backend_node
    ${catkin_LIBRARIES} 
    lifelong_backend
)
cmake_minimum_required(VERSION 3.10)
project(lifelong_backend_package)

set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

add_subdirectory(lifelong_backend)

if($ENV{ROS_VERSION} EQUAL 1)

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
        DIRECTORY app/ros1/msg
        FILES
        keyframe_info.msg
        control_info.msg
        trajectory_info.msg
        workspace_info.msg
        space_info.msg
    )

    add_service_files(
        DIRECTORY app/ros1/srv
        FILES
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

    include_directories(
        app/ros1/include
        ${catkin_INCLUDE_DIRS} 
    )

    add_executable(backend_node app/ros1/src/ros_wrapper.cpp)
    target_link_libraries(backend_node
        ${catkin_LIBRARIES} 
        lifelong_backend
    )
else()   # ros2
    # message(FATAL_ERROR "ros2 version...")
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(rosidl_default_generators REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(robot_msg REQUIRED)
    find_package(pcl_conversions REQUIRED)
    find_package(tf2 REQUIRED)
    find_package(tf2_geometry_msgs REQUIRED)
    find_package(tf2_ros REQUIRED)
    find_package(tf2_kdl REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(visualization_msgs REQUIRED)
    find_package(cv_bridge REQUIRED)
    find_package(Boost REQUIRED COMPONENTS thread)
    find_package(fmt REQUIRED)

    # message(FATAL_ERROR "std_msgs path: " ${std_msgs_INCLUDE_DIRS})

    
    set(dependencies
        rclcpp  
        nav_msgs 
        std_msgs
        tf2 
        tf2_geometry_msgs  
        tf2_ros 
        tf2_kdl 
        sensor_msgs 
        visualization_msgs
        cv_bridge
        pcl_conversions
        robot_msg
    )


    if(TARGET lifelong_backend)  
        message(STATUS "lib lifelong_backend exists.")  
    else()  
        message(FATAL_ERROR "lib lifelong_backend does not exist.")  
    endif()
    
    include_directories(
        app/ros2/include
    )

    add_executable(backend_node app/ros2/src/ros_wrapper.cpp)
    ament_target_dependencies(backend_node
        ${dependencies}
    )
    target_link_libraries(backend_node
        lifelong_backend
        fmt
        ${Boost_LIBRARIES}
    )

    add_executable(test_sub_node app/ros2/src/test_sub.cpp)
    ament_target_dependencies(test_sub_node
        ${dependencies}
    )
    target_link_libraries(test_sub_node
        fmt
    )

    add_executable(test2_sub_node app/ros2/src/test2_sub.cpp)
    ament_target_dependencies(test2_sub_node
        ${dependencies}
    )
    target_link_libraries(test2_sub_node
        fmt
    )

    add_executable(test_pub_node app/ros2/src/test_pub.cpp)
    ament_target_dependencies(test_pub_node
        ${dependencies}
    )
    target_link_libraries(test_pub_node
        fmt
    )

    install(
        TARGETS
            backend_node 
            test_sub_node 
            test2_sub_node 
            test_pub_node 
            lifelong_backend# 告诉ros2有这么个目标（可执行文件或者库）
        # EXPORT export_ndt_2d
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
    )

    ament_package()           #新加宏，安装项目，在CMakeLists.txt文件中的最后一个调用。
endif()

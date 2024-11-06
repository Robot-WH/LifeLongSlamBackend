cmake_minimum_required(VERSION 3.10)
project(lifelong_backend)

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

else()
    # message(FATAL_ERROR "ros2 version...")
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(rosidl_default_generators REQUIRED)
    find_package(nav_msgs REQUIRED)
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

    # 用于基于定义的接口（如消息、服务和动作）生成源代码和头文件
    # rosidl_generate_interfaces( keyframe_info_msg
    #     "app/ros2/msg/KeyframeInfo.msg"
    #     DEPENDENCIES nav_msgs sensor_msgs
    # )

    # Add the interface files to the list of files to be processed  
    set(INTERFACE_FILES  
        "app/ros2/msg/KeyframeInfo.msg"  # Make sure the path is correct relative to the CMakeLists.txt  
    )  

    # Generate interfaces for the specified files  
    rosidl_generate_interfaces(  
        my_interfaces  # This is a custom target name  
        ${INTERFACE_FILES}  
        DEPENDENCIES  
            nav_msgs  # Dependency on nav_msgs for Odometry  
            sensor_msgs  # Dependency on sensor_msgs for PointCloud2  
            # Add any other dependencies here if needed  
    )

    set(dependencies
        rclcpp  
        Boost 
        nav_msgs 
        tf2 
        tf2_geometry_msgs  
        tf2_ros 
        tf2_kdl 
        sensor_msgs 
        visualization_msgs
        cv_bridge
        pcl_conversions
    )
    
    include_directories(
        app/ros2/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../build/src/LifelongBackend/rosidl_generator_cpp
        # /code/robot_ws/build/src/LifelongBackend/rosidl_generator_cpp
    )

    if(TARGET lifelong_backend)  
        message(STATUS "lib lifelong_backend exists.")  
    else()  
        message(FATAL_ERROR "lib lifelong_backend does not exist.")  
    endif()

    add_executable(backend_node app/ros2/src/ros_wrapper.cpp)
    ament_target_dependencies(backend_node 
        ${dependencies}
    )
    target_link_libraries(backend_node
        lifelong_backend
        fmt
    )

    # ament_package()           #新加宏，安装项目，在CMakeLists.txt文件中的最后一个调用。
endif()

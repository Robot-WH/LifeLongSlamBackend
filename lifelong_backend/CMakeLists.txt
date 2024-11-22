cmake_minimum_required(VERSION 3.5)
project(lifelong_backend)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
# set(CMAKE_BUILD_TYPE "Debug")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)

find_package(Boost REQUIRED COMPONENTS thread timer)
find_package(Ceres REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(G2O REQUIRED)
find_package(Eigen3  REQUIRED)
find_package(glog  REQUIRED)
find_package(Protobuf REQUIRED)

FILE(GLOB PROTO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")

# 为什么不加入PIC控制就会报错？？
set(CMAKE_POSITION_INDEPENDENT_CODE ON)  

add_library(lifelong_backend_proto STATIC 
    proto/transform.pb.cc
    proto/graph.pb.cc
    proto/sc.pb.cc
)
target_link_libraries(lifelong_backend_proto ${Protobuf_LIBRARIES}) 
target_include_directories(lifelong_backend_proto PUBLIC 
    proto
    ${PROTOBUF_INCLUDE_DIRS}
)

# message(FATAL_ERROR "${PROTOBUF_INCLUDE_DIRS}" ${PROTOBUF_INCLUDE_DIRS})

set(CMAKE_POSITION_INDEPENDENT_CODE OFF)  

add_library(lifelong_backend SHARED
    src/backend_lifelong.cpp
    src/GraphOptimization/graph_optimization_g2o.cpp 
)

if($ENV{ROS_VERSION} EQUAL 1)
    find_package(OpenCV 3.0 REQUIRED)

    target_include_directories(lifelong_backend PUBLIC 
        include 
        ${G2O_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
    )

    if(TARGET slam_pointcloud)  
        message(STATUS "Target slam_pointcloud exists.")  
    else()  
        message(FATAL_ERROR "Target slam_pointcloud does not exist.")  
    endif()

    # message(FATAL_ERROR "slam_lib_3d_LIBRARIES: " ${slam_lib_3d_LIBRARIES})

    target_link_libraries(lifelong_backend PUBLIC
        ${Boost_LIBRARIES}
        ${EIGEN3_LIBRARIES} 
        ${PCL_LIBRARIES} 
        ${CERES_LIBRARIES}
        ${G2O_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${OpenCV_LIBS}
        slam_pointcloud
        yaml-cpp
        lifelong_backend_proto
    )
else()
    find_package(OpenCV 4.0 REQUIRED)
    find_package(slam_lib_3d REQUIRED)

    # message(FATAL_ERROR ${slam_lib_3d_INCLUDE_DIRS})

    target_include_directories(lifelong_backend PUBLIC 
        include 
        ${G2O_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${slam_lib_3d_INCLUDE_DIRS}
    )

    target_link_libraries(lifelong_backend PUBLIC
        ${Boost_LIBRARIES}
        ${EIGEN3_LIBRARIES} 
        ${PCL_LIBRARIES} 
        ${CERES_LIBRARIES}
        ${G2O_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${OpenCV_LIBS}
        ${slam_lib_3d_LIBRARIES}
        yaml-cpp
        lifelong_backend_proto
    )
endif()

cmake_minimum_required(VERSION 3.0.2)
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
find_package(OpenCV 3.0 QUIET)
find_package(GTSAM REQUIRED)
find_package(glog  REQUIRED)
find_package(Protobuf REQUIRED)

FILE(GLOB PROTO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
# PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

# PROTO_SRCS = /home/lwh/code/graph_3d_slam/build/LifelongBackend/lib/lifelong_backend/graph.pb.cc;/home/lwh/code/graph_3d_slam/build/LifelongBackend/lib/lifelong_backend/transform.pb.cc
# PROTO_HDRS = /home/lwh/code/graph_3d_slam/build/LifelongBackend/lib/lifelong_backend/graph.pb.h;/home/lwh/code/graph_3d_slam/build/LifelongBackend/lib/lifelong_backend/transform.pb.h
# message("PROTO_SRCS = ${PROTO_SRCS}")
# message("PROTO_HDRS = ${PROTO_HDRS}")

include_directories(
    include 
    ${Boost_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    ${SlamLib_INCLUDE_DIR}
    # ${PROTO_HDRS}
)
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

set(CMAKE_POSITION_INDEPENDENT_CODE OFF)  

add_library(lifelong_backend SHARED
    src/backend_lifelong.cpp
    src/GraphOptimization/graph_optimization_gtsam.cpp 
    src/GraphOptimization/graph_optimization_g2o.cpp 
    # src/parameters.cpp
)

target_link_libraries(lifelong_backend PUBLIC
    ${Boost_LIBRARIES}
    ${EIGEN3_LIBRARIES} 
    ${PCL_LIBRARIES} 
    ${CERES_LIBRARIES}
    ${G2O_LIBRARIES}
    ${GLOG_LIBRARIES}
    ${OpenCV_LIBS}
    slam_pointcloud
    gtsam
    yaml-cpp
    lifelong_backend_proto
)

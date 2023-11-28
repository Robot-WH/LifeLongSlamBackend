cmake_minimum_required(VERSION 3.0.2)
project(lifelong_backend)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)

find_package(Boost REQUIRED COMPONENTS thread timer)
find_package(Ceres REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(G2O REQUIRED)
find_package(Eigen3  REQUIRED)
find_package(OpenCV 3.0 QUIET)
find_package(GTSAM REQUIRED)
find_package(glog  REQUIRED)

include_directories(
    include 
    ${Boost_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    ${SlamLib_INCLUDE_DIR}
)


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
)

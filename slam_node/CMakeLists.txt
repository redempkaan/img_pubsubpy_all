cmake_minimum_required(VERSION 3.10)
project(slam_node)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# OpenCV
find_package(OpenCV REQUIRED)

# Eigen
find_package(Eigen3 REQUIRED)

# Pangolin
find_package(Pangolin REQUIRED)

find_package(CURL REQUIRED)

# ORB_SLAM3 root
set(ORB_SLAM3_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../..)

# Include paths
include_directories(
    ${ORB_SLAM3_DIR}
    ${ORB_SLAM3_DIR}/include
    ${ORB_SLAM3_DIR}/include/CameraModels 
    ${ORB_SLAM3_DIR}/Thirdparty
    ${ORB_SLAM3_DIR}/Thirdparty/Sophus
    ${ORB_SLAM3_DIR}/Thirdparty/g2o
    ${ORB_SLAM3_DIR}/Thirdparty/DBoW2
    ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/DBoW2
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    ${CURL_INCLUDE_DIRS}
)

# Executable
add_executable(slam_node main.cpp)

# 🔴 ASIL ÖNEMLİ SATIR BURASI
target_link_libraries(
    slam_node
    ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
    ${OpenCV_LIBS}
    Eigen3::Eigen
    ${Pangolin_LIBRARIES}
    
    CURL::libcurl
)

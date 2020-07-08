cmake_minimum_required(VERSION 2.8.12)
project(lidar_camera_fusion)

add_compile_options(-std=c++11)

set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev)


find_package(catkin REQUIRED COMPONENTS
  OpenCV
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  roscpp
  tf)

# ROS Kinetic OpenCV-3.3.1
include_directories(${OpenCV_INCLUDE_DIRS})

# PCL
#find_package(PCL REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  roscpp
  tf)

add_executable(lidar_camera_fusion
  src/lidar_camera_fusion.cpp
  src/main.cpp)

target_include_directories(lidar_camera_fusion PRIVATE
  ${catkin_INCLUDE_DIRS}
  include)

target_link_libraries(lidar_camera_fusion
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

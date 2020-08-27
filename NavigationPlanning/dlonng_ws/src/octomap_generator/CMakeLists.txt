cmake_minimum_required(VERSION 2.8)
project(octomap_generator)

add_compile_options(-std=c++11)

set(PACKAGE_DEPENDENCIES
  roscpp
  visualization_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  nav_msgs
  std_msgs
  std_srvs
  octomap_ros
  octomap_msgs
  nodelet
  scout_msgs
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

find_package(octomap REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS OCTOMAP
)

set(LINK_LIBS
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(octomap_generator
 src/octomap_generator/octomap_generator.cpp
 src/octomap_generator/octomap_generator_ros.cpp
 src/semantic_octree/semantics_bayesian.cpp
 src/semantic_octree/semantics_max.cpp
)

target_link_libraries(octomap_generator ${LINK_LIBS})

add_dependencies(octomap_generator scout_msgs_gencpp)

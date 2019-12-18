cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(CloudView)

set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
set( CMAKE_CXX_FLAGS "-std=c++11")

# pcl
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(cloud_view cloud_view.cc)

target_link_libraries (cloud_view ${PCL_LIBRARIES})

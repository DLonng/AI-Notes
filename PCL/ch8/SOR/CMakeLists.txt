cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(SOR)

set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
set( CMAKE_CXX_FLAGS "-std=c++11")

# pcl
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(sor sor.cc)

target_link_libraries (sor ${PCL_LIBRARIES})

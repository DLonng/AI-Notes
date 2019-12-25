cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(RandomSC)

set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
set( CMAKE_CXX_FLAGS "-std=c++14")

# pcl
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(random_sc random_sc.cpp)

target_link_libraries (random_sc ${PCL_LIBRARIES})

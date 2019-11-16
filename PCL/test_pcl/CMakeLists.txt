cmake_minimum_required(VERSION 2.6)
project(test_pcl)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(test_pcl test_pcl.cpp)

target_link_libraries (test_pcl ${PCL_LIBRARIES})

install(TARGETS test_pcl RUNTIME DESTINATION bin)
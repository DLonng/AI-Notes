cmake_minimum_required( VERSION 2.8 FATAL_ERROR )

project( PointCloudInter )

set( CMAKE_CXX_FLAGS "-std=c++11")

set( CMAKE_EXPORT_COMPILE_COMMANDS ON )


find_package( PCL 1.2 REQUIRED )

include_directories( ${PCL_INCLUDE_DIRS} )
link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )


add_executable ( point_cloud_inter point_cloud_inter.cc )
target_link_libraries ( point_cloud_inter ${PCL_LIBRARIES} ${OpenCV_LIBS} )

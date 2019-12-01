# min cmake version
cmake_minimum_required( VERSION 2.8 )

# project name
project( JoinMap )

set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

# add  c++11 stand
set( CMAKE_CXX_FLAGS "-std=c++11")


find_package(VTK 6.2 REQUIRED NO_MODULE)

# eigen
include_directories( "/usr/include/eigen3/" )

# opencv
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# pcl
find_package( PCL REQUIRED COMPONENT common io )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# add a exe program
add_executable( join_map join_map.cc )

# link opencv lib 
target_link_libraries( join_map ${OpenCV_LIBS} ${PCL_LIBRARIES} )

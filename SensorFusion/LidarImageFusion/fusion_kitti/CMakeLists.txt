# min cmake version
cmake_minimum_required( VERSION 2.8 )

# project name
project( Fusion )

set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

# add  c++11 stand
set( CMAKE_CXX_FLAGS "-std=c++11")


# opencv
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )


# pcl
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# add a exe program
add_executable( fusion fusion.cpp )

# link opencv lib 
target_link_libraries( fusion ${OpenCV_LIBS} ${PCL_LIBRARIES} )

# min cmake version
cmake_minimum_required( VERSION 2.8 )

# project name
project( ImageBasics )

set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

# add  c++11 stand
set( CMAKE_CXX_FLAGS "-std=c++11")

# find opencv lib
find_package( OpenCV REQUIRED )

# add opencv include 
include_directories( ${OpenCV_INCLUDE_DIRS} )

# add a exe program
add_executable( image_basics image_basics.cc )

# link opencv lib 
target_link_libraries( image_basics ${OpenCV_LIBS})

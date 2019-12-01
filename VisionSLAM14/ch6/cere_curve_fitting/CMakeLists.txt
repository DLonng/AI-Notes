# min cmake version
cmake_minimum_required( VERSION 2.8 )

# project name
project( CereCurve )

set( CMAKE_CXX_FLAGS "-std=c++11")

set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

# opencv 
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# ceres
find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )

include_directories( "/usr/include/eigen3/" )

# add a exe program
add_executable( cere_curve cere_curve.cc )

# link opencv lib 
target_link_libraries( cere_curve ${CERES_LIBRARIES} ${OpenCV_LIBS})

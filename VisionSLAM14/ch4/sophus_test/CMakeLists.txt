# min cmake version
cmake_minimum_required( VERSION 2.8 )

# project name
project( SophusTest)

set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )


# eigen
include_directories( "/usr/include/eigen3/" )

# add a exe program
add_executable( use_sophus use_sophus.cc )

target_link_libraries( use_sophus ${Sophus_LIBRARIES} )

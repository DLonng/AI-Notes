# min cmake version
cmake_minimum_required( VERSION 2.8 )

# project name
project( UsingGeometry )

# include eigen include 
include_directories( "/usr/include/eigen3" )

# add a exe program
add_executable( using_geometry using_geometry.cc )

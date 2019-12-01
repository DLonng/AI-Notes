# min cmake version
cmake_minimum_required( VERSION 2.8 )

# project name
project( EigenTest )

# include eigen include 
include_directories( "/usr/include/eigen3" )

# add a exe program
add_executable( eigen_matrix eigen_matrix.cc )

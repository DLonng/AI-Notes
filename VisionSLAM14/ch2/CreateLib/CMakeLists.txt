# min cmake version
cmake_minimum_required( VERSION 2.8 )

# project name
project( HelloSLAM )

# add a static library
add_library( hello_slam lib_hello_slam.cc )

# add a shared library
add_library( hello_slam_shared SHARED lib_hello_slam.cc )

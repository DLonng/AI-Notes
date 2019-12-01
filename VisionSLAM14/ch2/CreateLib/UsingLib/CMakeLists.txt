# min cmake version
cmake_minimum_required( VERSION 2.8 )

# project name
project( LibHelloSLAM )

# add a exe 
add_executable( hello_shared use_hello_slam_shared.cc )

# link shared lib
target_link_libraries( hello_shared hello_slam_shared )

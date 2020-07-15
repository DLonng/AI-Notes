cmake_minimum_required(VERSION 2.8.3)
project(learning_debug)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  turtlesim
)



add_executable(debug_test src/debug_test.cpp)
target_link_libraries(debug_test ${catkin_LIBRARIES} )

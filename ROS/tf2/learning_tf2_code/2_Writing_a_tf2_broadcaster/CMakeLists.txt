find_package(catkin REQUIRED COMPONENTS
             roscpp
             rospy
             turtlesim
             tf
)

add_executable(turtle_tf2_broadcaster src/turtle_tf2_broadcaster.cpp)

target_link_libraries(turtle_tf2_broadcaster ${catkin_LIBRARIES})
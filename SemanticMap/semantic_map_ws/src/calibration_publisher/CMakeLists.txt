cmake_minimum_required(VERSION 2.8.3)
project(calibration_publisher)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    tf
    std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(CATKIN_DEPENDS
    roscpp
    sensor_msgs
    tf
    std_msgs
)


include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    include
)

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

add_executable(calibration_publisher 
               src/calibration_publisher.cpp)

target_link_libraries(calibration_publisher 
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES})   

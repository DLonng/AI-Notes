cmake_minimum_required(VERSION 2.8.12)
project(lidar_camera_fusion)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        pcl_conversions
        pcl_ros
        roscpp
        tf)

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )


# PCL
find_package( PCL REQUIRED COMPONENT common io )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

catkin_package(CATKIN_DEPENDS
        cv_bridge
        image_transport
        pcl_conversions
        pcl_ros
        roscpp
        tf
        )


add_executable(lidar_camera_fusion 
               src/lidar_camera_fusion.cpp
               src/main.cpp)

target_include_directories(lidar_camera_fusion PRIVATE
        ${OpenCV_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        include
)

target_link_libraries(lidar_camera_fusion
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
)

cmake_minimum_required(VERSION 3.0.2)
project(add_snow_to_custom_msg)


find_package(catkin REQUIRED COMPONENTS
    PCL REQUIRED
    rosbag
    roscpp
    sensor_msgs
    std_msgs
    pcl_ros
    livox_ros_driver
)


include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

link_directories(
    ${PCL_LIBRARY_DIRS}
)

add_definitions(
    ${PCL_DEFINITIONS}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cloud_merging_v2
 CATKIN_DEPENDS rosbag roscpp sensor_msgs std_msgs
 DEPENDS system_lib
)


add_executable(add_snow_to_custom_msg src/add_snow_to_custom_msg.cpp)
add_executable(custom_to_pointcloud2 src/custom_to_pointcloud2.cpp)
add_executable(custom_msg_range_node src/custom_msg_range_node.cpp)
add_executable(bag_merging src/bag_merging.cpp)

target_link_libraries(add_snow_to_custom_msg 
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
)

target_link_libraries(custom_to_pointcloud2 
    ${catkin_LIBRARIES}
)

target_link_libraries(custom_msg_range_node 
    ${catkin_LIBRARIES}
)

target_link_libraries(bag_merging
    ${catkin_LIBRARIES}
)


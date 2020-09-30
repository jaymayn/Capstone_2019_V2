cmake_minimum_required(VERSION 2.8.3)
project(capstone_v2)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  geometry_msgs
)

find_package(OpenCV REQUIRED)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
##${PCL_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIRS}
)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

add_executable(depth2pointcloud src/depth2cloud_V2.cpp)
##target_link_libraries(depth2pointcloud ${OpenCV_LIBS} ${catkin_LIBRARIES})
target_link_libraries(depth2pointcloud ${PCL_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})

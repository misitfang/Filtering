cmake_minimum_required(VERSION 3.0.2)
project(ring_ground_filter)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  velodyne_pointcloud
)
catkin_package()

find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_executable(ring_ground_filter
  src/ring_ground_filter.cpp
)
target_include_directories(ring_ground_filter PRIVATE
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(ring_ground_filter
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(ring_ground_filter ${catkin_EXPORTED_TARGETS})





## Your package locations should be listed before other locations



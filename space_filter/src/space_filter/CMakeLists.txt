cmake_minimum_required(VERSION 3.0.2)
project(space_filter)

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
find_package(PCL 1.7 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES space_filter
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs velodyne_pointcloud
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(space_filter src/space_filter.cpp)
target_link_libraries(space_filter
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


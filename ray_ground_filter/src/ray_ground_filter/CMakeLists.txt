cmake_minimum_required(VERSION 3.0.2)
project(ray_ground_filter)
# set(ROSLINT_CPP_OPTS "--filter=-build/c++14")
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CMAKE_BUILD_TYPE "Debug")
find_package(catkin REQUIRED COMPONENTS
pcl_conversions
pcl_ros
roscpp
sensor_msgs
std_msgs
dynamic_reconfigure
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenMP)
generate_dynamic_reconfigure_options(
   cfg/Ray.cfg
 )
catkin_package(
   INCLUDE_DIRS include

  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs pcl_conversions dynamic_reconfigure
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(ray_ground_filter_lib src/ray_ground_filter.cpp
)
target_link_libraries(ray_ground_filter_lib 
${catkin_LIBRARIES}
${PCL_LIBRARIES}
${OpenCV_LIBRARIES}
${Qt5Core_LIBRARIES})

add_dependencies(ray_ground_filter_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



add_executable(ray_ground_filter src/ray_ground_filter_node.cpp)
add_dependencies(ray_ground_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ray_ground_filter ray_ground_filter_lib)




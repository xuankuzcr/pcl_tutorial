cmake_minimum_required(VERSION 2.8.3)
project(pcl_tutorials)
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_msgs
  pcl_ros
  sensor_msgs
)
find_package(PCL REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcl_tutorials
#  CATKIN_DEPENDS pcl_conversions pcl_msgs pcl_ros sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(pcl_test src/pcl_test.cpp)
target_link_libraries(pcl_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})

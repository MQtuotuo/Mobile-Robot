cmake_minimum_required(VERSION 2.8.3)
project(experiment_05)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
find_package( OpenCV REQUIRED )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES experiment_03
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
)
add_executable(robo_5_0 src/robo_5_0.cpp)
target_link_libraries(robo_5_0 ${OpenCV_LIBRARIES} ${roscpp_LIBRARIES})


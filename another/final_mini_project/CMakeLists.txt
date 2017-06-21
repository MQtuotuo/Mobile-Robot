cmake_minimum_required(VERSION 2.8.3)
project(final_mini_project)

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
  actionlib
  move_base_msgs
  tf
)
find_package(OpenCV REQUIRED)


catkin_package(
  CATKIN_DEPENDS 
)



include_directories(
  ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} 
)


add_executable(simple_navigation_goals src/simple_navigation_goals.cpp)
target_link_libraries(simple_navigation_goals ${catkin_LIBRARIES})

#add_executable(robo_4_6 src/robo_4_6.cpp)
#target_link_libraries(robo_4_6 ${OpenCV_LIBRARIES} ${roscpp_LIBRARIES} ${catkin_LIBRARIES})
#target_link_libraries (robo_4_6 lept tesseract)


add_executable(final_test_2 src/final_test_2.cpp)
target_link_libraries(final_test_2 ${OpenCV_LIBRARIES} ${roscpp_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries (final_test_2 lept tesseract) 


add_executable(robo_02_2 src/robo_02_2.cpp)
target_link_libraries(robo_02_2 ${roscpp_LIBRARIES})

# Declare the version of the CMake API for forward-compatibility
cmake_minimum_required(VERSION 2.8)

# Declare the name of the CMake Project
project(experiment_01)

# Find and get all the information about the roscpp package
find_package(roscpp REQUIRED)

# Find Catkin
find_package(catkin REQUIRED)
# Declare this project as a catkin package
catkin_package()

# Add the local headers and the headers from roscpp
include_directories(include ${roscpp_INCLUDE_DIRS})


# Define executables target called robo_XX
add_executable(robo_01 src/robo_01.cpp)
target_link_libraries(robo_01 ${roscpp_LIBRARIES})
#
add_executable(robo_02 src/robo_02.cpp)
target_link_libraries(robo_02 ${roscpp_LIBRARIES})
#
add_executable(robo_03 src/robo_03.cpp)
target_link_libraries(robo_03 ${roscpp_LIBRARIES})
#
add_executable(robo_04 src/robo_04.cpp)
target_link_libraries(robo_04 ${roscpp_LIBRARIES})

#thats all folks


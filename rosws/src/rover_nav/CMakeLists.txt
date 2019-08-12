cmake_minimum_required(VERSION 2.8.3)
project(rover_nav)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg pcl_conversions pcl_ros roscpp  sensor_msgs tf_conversions)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare a cpp executable
# add_executable(node src/node.cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(node
#   ${catkin_LIBRARIES}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rqt_simulation_launcher.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

cmake_minimum_required(VERSION 2.8.3)
project(navigation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
 pocketsphinx
 tf

)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lab_exams
#  CATKIN_DEPENDS geometry_msgs message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
)

## Declare ROS messages and services
#add_message_files(DIRECTORY msg FILES )
#add_service_files(DIRECTORY srv FILES )

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a catkin package
## Declare a cpp executable


add_executable(map_navigation_node src/map_navigation.cpp)
target_link_libraries(map_navigation_node ${catkin_LIBRARIES})

add_executable(input_navigation_node src/input_navigation.cpp)
target_link_libraries(input_navigation_node ${catkin_LIBRARIES})

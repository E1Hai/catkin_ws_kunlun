cmake_minimum_required(VERSION 3.0.2)
project(assigment7)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  autominy_msgs
  roscpp
  rospy
  message_generation
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs

)



## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   autominy_msgs#   geometry_msgs#   nav_msgs#   sensor_msgs#   std_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES assigment7
  CATKIN_DEPENDS autominy_msgs geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)



add_executable(subpub src/subpub.cpp)
target_link_libraries(subpub ${catkin_LIBRARIES})
add_dependencies(subpub ${catkin_EXPORTED_TARGETS})

cmake_minimum_required(VERSION 2.8.3)
project(tdoa_sonar_software)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
  visualization_msgs
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Calculated_time_stamps.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   Localization_query.srv   
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs#   visualization_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sound_localization
#  CATKIN_DEPENDS dynamic_reconfigure message_generation roscpp rospy std_msgs visualization_msgs
#  DEPENDS system_lib
)
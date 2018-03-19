cmake_minimum_required(VERSION 2.8.3)
project(advantech_pci1714_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  message_runtime
  std_msgs
)

add_service_files(DIRECTORY srv
   FILES
   Ping_received.srv
   Ping.srv
   Hydrophone_locations_service.srv
 )

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Pingdata.msg
 )

 generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  #CATKIN_DEPENDS
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(advantechPCI1714Driver src/advantechPCI1714Driver.cpp) # libraries/bdaqctrl.h libraries/compatibility.h)
add_library(libbiodaq STATIC IMPORTED)
set_property(TARGET libbiodaq PROPERTY IMPORTED_LOCATION /usr/lib/libbiodaq.so)
target_link_libraries(advantechPCI1714Driver ${catkin_LIBRARIES} libbiodaq)
add_dependencies(advantechPCI1714Driver advantech_pci1714_driver_cpp)
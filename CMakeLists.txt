cmake_minimum_required(VERSION 3.0.2)
project(naist_xarm)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  genmsg
  xarm_msgs
  sensor_msgs
  actionlib
  actionlib_msgs
  control_msgs
  xarm_sdk
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES xarm_ros_driver xarm_ros_client
)

include_directories(

   include
  
  ${xarm_sdk_INCLUDE_DIRS}
  ${xarm_sdk_INCLUDE_DIRS}/xarm_sdk
  ${catkin_INCLUDE_DIRS}
)

add_library(xarm_ros_driver
  src/xarm_driver.cpp
)

add_dependencies(xarm_ros_driver xarm_msgs_generate_messages_cpp)
target_link_libraries(xarm_ros_driver 
  ${xarm_sdk_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(xarm_ros_client
  src/xarm_ros_client.cpp
)
add_dependencies(xarm_ros_client xarm_msgs_generate_messages_cpp)

add_executable(xarm_driver_node
  src/xarm_driver_node.cpp
)
add_dependencies(xarm_driver_node xarm_msgs_generate_messages_cpp)
target_link_libraries(xarm_driver_node 
  xarm_ros_driver
  ${catkin_LIBRARIES}
)




add_executable(basic_simple_talker   src/basic_simple_talker.cpp)
#add_executable(test   src/test.cpp)
add_executable(demo   src/demo.cpp)

target_link_libraries(basic_simple_talker
    ${catkin_LIBRARIES}
)
#target_link_libraries(test
 # ${catkin_LIBRARIES}
#)
target_link_libraries(demo
 ${catkin_LIBRARIES}
)


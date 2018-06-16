cmake_minimum_required(VERSION 2.8.3)
project(can_control_node)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  can_node_msg
  tf
  )


include_directories(
  include
	${catkin_INCLUDE_DIRS} 
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs can_node_msg
  INCLUDE_DIRS include
)


#add_library(canalystii src/canalystii.cpp ${catkin_LIBRARIES})
#target_link_libraries(canalystii  ${PROJECT_SOURCE_DIR}/lib/libcontrolcan.so  ${catkin_LIBRARIES})

#add_library(can_control_node src/can_control_node.cpp ${catkin_LIBRARIES})
#target_link_libraries(can_control_node ${PROJECT_SOURCE_DIR}/lib/libcontrolcan.so ${catkin_LIBRARIES})

#add_executable(canalystii_node_ros src/canalystii_node_ros.cpp) 
#target_link_libraries(canalystii_node_ros canalystii can_control_node ${PROJECT_SOURCE_DIR}/lib/libcontrolcan.so ${catkin_LIBRARIES})

add_executable(canalystii_node_ros src/canalystii_node_ros.cpp src/canalystii.cpp src/canalystii_node.cpp src/pid.cpp)
target_link_libraries(canalystii_node_ros ${PROJECT_SOURCE_DIR}/lib/libcontrolcan.so ${catkin_LIBRARIES})

add_executable(twist_converter_node src/twist_converter.cpp)
target_link_libraries(twist_converter_node ${catkin_LIBRARIES})

add_executable(canbus_trans src/canbus_trans.cpp)
target_link_libraries(canbus_trans ${catkin_LIBRARIES})

add_executable(can_msg_receiver src/can_msg_receiver.cpp)
target_link_libraries(can_msg_receiver ${PROJECT_SOURCE_DIR}/lib/libcontrolcan.so ${catkin_LIBRARIES})

add_executable(can_msg_sender src/can_msg_sender.cpp)
target_link_libraries(can_msg_sender ${PROJECT_SOURCE_DIR}/lib/libcontrolcan.so ${catkin_LIBRARIES})


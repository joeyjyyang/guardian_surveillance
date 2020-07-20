cmake_minimum_required(VERSION 2.8.3)
project(guardian_surveillance)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_srvs
  cv_bridge
  message_generation
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(intruder_detecter_node src/intruder_detecter_node.cpp)

target_link_libraries(intruder_detecter_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

catkin_install_python(
  PROGRAMS scripts/email_alerter_node scripts/text_alerter_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

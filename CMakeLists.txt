cmake_minimum_required(VERSION 2.8.3)
project(guardian_surveillance)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_srvs
  cv_bridge
  raspicam_node
  image_view
  message_generation
)

find_package(OpenCV REQUIRED)

## Uncomment if the package has a setup.py
# catkin_python_setup()

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#  Object.msg
#)

## Generate services in the 'srv' folder
#add_service_files(
##  DIRECTORY 
#  FILES
#   Service1.srv
#   Service2.srv
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
  #DEPENDENCIES
  #sensor_msgs
  # std_msgs
#)

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES guardian_surveillance
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_srvs cv_bridge raspicam_node image_view message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(intruder_detecter_node src/intruder_detecter_node.cpp)

# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

target_link_libraries(intruder_detecter_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#add_dependencies(intruder_detect_node guardian_surveillance_generate_messages_cpp)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(
  PROGRAMS scripts/email_alerter_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

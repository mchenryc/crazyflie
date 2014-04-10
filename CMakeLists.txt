cmake_minimum_required(VERSION 2.8.3)
project(crazyflie)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  cfclient
)
catkin_python_setup()
add_message_files(
  FILES
  CFJoy.msg
  CFAltHold.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(CATKIN_DEPENDS
  message_runtime
  cfclient)
install(PROGRAMS scripts/cf_node scripts/cf_teleop
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

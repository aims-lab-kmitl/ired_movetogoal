cmake_minimum_required(VERSION 3.0.2)
project(ired_movetogoal)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs
)

###########
## Build ##
###########

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/movetogoal.py
  scripts/testservo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

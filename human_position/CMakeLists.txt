cmake_minimum_required(VERSION 3.0.2)
project(human_position)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
)

#catkin_python_setup()


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Keypoint.msg
  HumanCoordinates.msg
  HumanCoordinatesArray.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


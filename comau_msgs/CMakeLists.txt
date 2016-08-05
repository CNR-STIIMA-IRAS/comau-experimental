cmake_minimum_required(VERSION 2.8.3)
project(comau_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs industrial_msgs genmsg message_generation actionlib_msgs roscpp)

add_message_files(
  FILES
  JointTrajComau.msg
  JointTrajPointComau.msg
  MotionFeedbackComau.msg
  )

add_service_files(
  FILES
  CmdJointTrjComau.srv
  )
  
add_action_files(
#  DIRECTORY action
  FILES
  ComauJointTrajectory.action
  )  

generate_messages(
  DEPENDENCIES std_msgs industrial_msgs actionlib_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS message_runtime std_msgs industrial_msgs genmsg actionlib_msgs
)

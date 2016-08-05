cmake_minimum_required(VERSION 2.8.3)
project(comau_nj220foundry_support)

find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ns16hand_description
   CATKIN_DEPENDS comau_driver
#  DEPENDS system_lib
)



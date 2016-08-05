cmake_minimum_required(VERSION 2.8.3)
project(comau_driver)
SET(CMAKE_CXX_FLAGS "-std=c++11")
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS std_msgs roscpp urdf industrial_robot_client industrial_utils simple_message sensor_msgs comau_msgs roslint roslaunch)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)

# find_package(Doxygen)
# if(DOXYGEN_FOUND)
#   configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
#   add_custom_target(doc ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
#                     WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
#                     COMMENT "Generating API documentation with Doxygen" VERBATIM )
# endif(DOXYGEN_FOUND)

roslint_cpp()
#######################################
## Adding directories and definitions #
#######################################
include_directories(  comau_trajectory/include 
                      comau_joint_streamer/include  
                      comau_joint_trajectory_action/include 
                      comau_motion_feedback/include
                      comau_robot_status/include
                      ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} )
add_definitions(-DLINUXSOCKETS=1)  #use linux sockets for communication
add_definitions(-DROS=1)
#add_definitions(-DDUAL_ARM=1)   #comment this line if you are using a single arm robot

catkin_package(
    DEPENDS
    CATKIN_DEPENDS std_msgs roscpp simple_message urdf industrial_robot_client comau_msgs sensor_msgs industrial_utils
    INCLUDE_DIRS comau_trajectory/include comau_joint_streamer/include  comau_joint_trajectory_action/include comau_motion_feedback/include comau_robot_status/include
    LIBRARIES comau_trajectory
)

roslaunch_add_file_check(launch)

add_subdirectory( comau_trajectory )
add_subdirectory( comau_joint_streamer )
add_subdirectory( comau_joint_trajectory_action )
add_subdirectory( comau_motion_feedback )
add_subdirectory( comau_robot_status )

add_subdirectory( test )

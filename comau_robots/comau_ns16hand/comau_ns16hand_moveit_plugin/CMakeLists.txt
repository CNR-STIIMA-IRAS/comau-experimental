cmake_minimum_required(VERSION 2.8.3)
project(comau_ns16hand_moveit_plugin)

find_package(catkin REQUIRED COMPONENTS   moveit_core   pluginlib   roscpp   tf_conversions
)

include_directories(${catkin_INCLUDE_DIRS})

add_definitions("-DITIA_CNR_PATCH -std=c++14")

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
    moveit_core
    pluginlib
    roscpp
    tf_conversions
)

include_directories(include)

set(IKFAST_LIBRARY_NAME comau_ns16hand_manipulator_moveit_ikfast_plugin)

find_package(LAPACK REQUIRED)

add_library(${IKFAST_LIBRARY_NAME} src/comau_ns16hand_manipulator_ikfast_moveit_plugin.cpp)
target_link_libraries(${IKFAST_LIBRARY_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})

install(TARGETS ${IKFAST_LIBRARY_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
  FILES
  comau_ns16hand_manipulator_moveit_ikfast_plugin_description.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#library
add_library           ( comau_joint_trajectory_action       src/comau_joint_trajectory_action/comau_joint_trajectory_action.cpp)
target_link_libraries ( comau_joint_trajectory_action       comau_trajectory industrial_utils )
#Node
add_executable        ( comau_joint_trajectory_action_node  src/comau_joint_trajectory_action_node.cpp)
add_dependencies      ( comau_joint_trajectory_action_node  ${catkin_EXPORTED_TARGETS} )
target_link_libraries ( comau_joint_trajectory_action_node  comau_joint_trajectory_action ${catkin_LIBRARIES} )


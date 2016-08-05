#Library
add_library           ( comau_joint_streamer      src/comau_joint_streamer/comau_joint_trajectory_streamer.cpp )
#Node
add_executable        ( comau_joint_streamer_node src/comau_joint_streamer_node.cpp)                
add_dependencies      ( comau_joint_streamer_node ${catkin_EXPORTED_TARGETS})
target_link_libraries ( comau_joint_streamer_node comau_joint_streamer comau_trajectory simple_message ${catkin_LIBRARIES} )
set_target_properties ( comau_joint_streamer_node PROPERTIES OUTPUT_NAME comau_joint_streamer_node PREFIX "")
  

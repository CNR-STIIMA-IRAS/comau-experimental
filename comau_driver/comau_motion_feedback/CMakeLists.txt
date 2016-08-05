
set(MOTION_FEEDBACK_SRC_FILES src/comau_motion_feedback/motion_feedback_comau_relay_handler.cpp
                              src/comau_motion_feedback/motion_feedback_comau_interface.cpp
                              src/comau_motion_feedback/motion_feedback_comau_message.cpp
                              src/comau_motion_feedback/motion_feedback_comau.cpp
                              src/comau_motion_feedback/cartesian_data.cpp)

#Library
add_library           ( comau_motion_feedback ${MOTION_FEEDBACK_SRC_FILES})
add_dependencies      ( comau_motion_feedback ${catkin_EXPORTED_TARGETS}  )
target_link_libraries ( comau_motion_feedback industrial_utils simple_message )
#Node
add_executable        ( motion_feedback_node  src/motion_feedback_node.cpp    )
add_dependencies      ( motion_feedback_node  ${catkin_EXPORTED_TARGETS}  )

message("${catkin_LIBRARIES}")

target_link_libraries ( motion_feedback_node  comau_motion_feedback ${catkin_LIBRARIES})

  

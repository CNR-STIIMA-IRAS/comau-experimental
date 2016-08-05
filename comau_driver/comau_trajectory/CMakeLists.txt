set(TRJ_SRC_FILES src/comau_trajectory/comau_joint_trajectory_interface.cpp
                  src/comau_trajectory/joint_traj_pt_comau.cpp
                  src/comau_trajectory/joint_traj_pt_comau_message.cpp )

#Library
add_library           ( comau_trajectory ${TRJ_SRC_FILES} )
add_dependencies      ( comau_trajectory ${catkin_EXPORTED_TARGETS} )
target_link_libraries ( comau_trajectory ${catkin_LIBRARIES} )
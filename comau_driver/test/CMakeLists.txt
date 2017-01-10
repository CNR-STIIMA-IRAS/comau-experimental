add_executable( comau_action_client_trj_test
                comau_action_client_trj_test.cpp)
add_dependencies( comau_action_client_trj_test ${catkin_EXPORTED_TARGETS} )
target_link_libraries(comau_action_client_trj_test 
                      comau_trajectory ${catkin_LIBRARIES})
  
add_executable( standard_action_client_trj_test
                standard_action_client_trj_test.cpp)
add_dependencies( standard_action_client_trj_test ${catkin_EXPORTED_TARGETS} )
target_link_libraries(standard_action_client_trj_test ${catkin_LIBRARIES})
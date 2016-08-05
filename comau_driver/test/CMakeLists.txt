add_executable( action_client_trj_test
                action_client_trj_test.cpp)
add_dependencies( action_client_trj_test ${catkin_EXPORTED_TARGETS} )
target_link_libraries(action_client_trj_test 
                      comau_trajectory ${catkin_LIBRARIES})
  

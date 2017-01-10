## comau_robot_status

Feedback from the controller giving information about the state of the robot.  
This package contains a simple implementation of the standard robot_state_interface.  

External interface:

  * topic _**robot_status**_ of type _industrial_msgs::RobotStatus_  
  ROS-I standard feedback:
  
    - header (of type _std_msgs/Header_)  
    - mode (of type _industrial_msgs::RobotMode_)  
    - e_stopped (of type _industrial_msgs::TriState_)  
    - drives_powered (of type _industrial_msgs::TriState_)  
    - motion_possible (of type _industrial_msgs::TriState_)  
    - in_motion (of type _industrial_msgs::TriState_)  
    - in_error (of type _industrial_msgs::TriState_)  
    - error_code (of type _int32_)  


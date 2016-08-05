## comau_trajectory 

Data definition and interface methods for the trajectory handling.

External interface:

  * service _**"start_motion"**_ of type _industrial_msgs::StartMotion_  
  Send to the robot the start motion command

  * service _**"stop_motion"**_ of type _industrial_msgs::StopMotion_   
  Send to the robot the stop motion command, the motion can be resumed

  * service _**"cancel_motion"**_ of type _industrial_msgs::StopMotion_   
  Send to the robot the cancel motion command, the motion cannot be resumed and the trajectory on the controller is deleted

  * service _**"joint_path_command"**_ of type _comau_msgs::CmdJointTrjComau_   
  Load a new trajectory and send it to the robot controller


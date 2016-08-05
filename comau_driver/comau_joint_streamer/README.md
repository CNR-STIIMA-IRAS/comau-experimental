# comau_joint_streamer

It handles the trajectory loading and the dispatching of the points to the robot controller.  
It contains the thread in charge of the trajectory data exchange with the robot controller.

External interface:

  * service _**"reload_trj"**_ of type _industrial_msgs::StartMotion_  
  Re-sends the previously loaded trajectory to the robot controller

  * topic _**"comau_trajectory_sts"**_ of type _std_msgs::Int32_  
  Status of the trajectory: 
    + 0 -> trajectory is downloading to the robot controller
    + 1 -> cancel_motion request while the trajectory downloading was not completed
    + 2 -> trajectory downloading completed without any cancel_motion request
    + 3 -> cancel_motion request while the trajectory downloading was already finished.


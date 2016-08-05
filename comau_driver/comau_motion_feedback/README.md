## comau_motion_feedback

Package to deal with the motion feedback data from the robot controller (data types definitions, data exchange and packing).

External interface:

  * topic _**"joint_states"**_ of type _sensor_msgs::JointState_  
  ROS-I standard feedback:
    - Header with timestamp
    - Joint names
    - Joint positions [in rad]
    - Motor currents [in A]
  
  * topic _**"motion_feedback"**_ of type _comau_msgs::MotionFeedbackComau_  
  Feedback with some additional informations:
    - Header with timestamp
    - Arm number
    - Base frame [Comau style, in mm and deg]
    - User frame [Comau style, in mm and deg]
    - Tool frame [Comau style, in mm and deg]
    - Joint positions [in deg]
    - Motor currents [in A]
    - Cartesian position [Comau style, in mm and deg]
    - Sequential number of the last point processed by the controller
    - Time of the controller
    - Flag saying if the robot is in motion or not.


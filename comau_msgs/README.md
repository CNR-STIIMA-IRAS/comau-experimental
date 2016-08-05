# comau_msgs

The package handles some messages used by the `comau_driver`.
Short description of the messages is below reported.

## Messages

```
JointTrajPointComau.msg:

  int8 arm_number
  int8 queue_status
  int16 sequence_number
  float32[] linear_velocity
  float32[] joint_positions
  int16 digital_output
```

The message represents the target/feedback position of the robot. With the joint position values, other fields are necessary to identify correctly the robot state.  
The fields of the message are:  
- _arm_number_:  number of the arm used. The COMAU controller allows to use up to 2 different arms. The `comau_driver` allows the control of 1 arm. The extensio to multiple arms will be delivered soon.  
- _queue_status_: status of the controller's circular buffer. This field says if the buffer is full (hence it is not possible to send a new point of the trajectory) or if it is ready to accept the next position. 
  Refer to `include/comau_msgs/comau_simple_message.h` for the field value description.  
- _sequence_number_: sequential number of the actual point in the trajectory. This field has to be incremented for each point of the same trajectory, otherwise the PDL program will give an error.  
- _linear_velocity_: is the cartesian velocity used by the Comau interpolator to move the robot on the specific point (expressed in m/s).
- _joint_positions_: is the position of the point in joint coordinates (expressed in degrees). The angular position of each joint of the arm has to be specified.  
- _digital_output_: it is a decimal base representation of a 16 bit string. Each bit could be used to control a digital output of the Comau controller (handling of this feature on the controller side is coming soon).  
  

``` 
JointTrajComau.msg:

  std_msgs/Header header
  string[] joint_names
  comau_msgs/JointTrajPointComau[] points
```
The message represents a robot trajectory. The message fields are:  
- _header_: is the std_msgs header of the trajectory message.  
- _joint_names_: is a string array with the joint names of the robot. The same names of the URDF robot description have to be used.  
- _points_: is the array of points that define the trajectory.  




```
MotionFeedbackComau.msg:

  std_msgs/Header header
  int8 arm_number
  float32[] base_frame
  float32[] user_frame
  float32[] tool_frame
  float32[] joint_positions
  float32[] joint_motor_currents
  float32[] cartesian_positions
  # TODO: add the flag for the cartesian flag
  int32 last_point_processed
  int32 cntrl_time
  int8 is_moving

```
The message encapsulates some useful information on the robot state (online monitoring). The message fields are:  
- _Header_:  is the std_msgs header with timestamp.  
- _Arm number_: is the arm number. All the informations contained into the message are referred to this arm.  
- _Base frame [Comau style, in mm and deg]_: is the base_frame position.  
- _User frame [Comau style, in mm and deg]_: is the user_frame position.  
- _Tool frame [Comau style, in mm and deg]_: is the tool_frame position.  
- _Joint positions [in deg]_: is the actual joint position of the arm.  
- _Joint motor currents [in A]_: is the measure of the current of each joint motor.  
- _Cartesian position [Comau style, in mm and deg]_: is the actual cartesian position of the TCP of the robot (the value takes into account the tool definition).  
- _Last point processed_: is the sequential number of the last point processed by the controller.  
- _Cntrl time [in ms]_: is the time of the controller.  
- _Is moving_: is a flag that describes if the robot is in motion or not.  

## Service Descriptors

```
CmdJointTrjComau.srv:

  comau_msgs/JointTrajComau trajectory
  ---
  industrial_msgs/ServiceReturnCode code
```

it is used in order to send a new trajectory to the robot controller. The message encapsulates a `JointTrajComau` for the request part, and an industrial_msgs/ServiceReturnCode
for the response (telling if the call has been correctly processed or not).  


## Action Description  
The package provides the following actions:  

```
ComauJointTrajectory.action:

  comau_msgs/JointTrajComau goal     
  ---   
  industrial_msgs/ServiceReturnCode result     
  ---
```
It provides the interface of the Action Server/Client structure.

**Note:**  
> A bridge node to convert comau_msgs::ComauJointTrajectoryAction into control_msgs::FollowJointTrajectoryAction will be ready soon.

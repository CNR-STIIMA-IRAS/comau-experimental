# Preliminaries: Package Organization
This package groups all the ROS code of the Comau driver.  
The following table lists the libraries and nodes provided by the package.

**`comau_driver/`**  
**`comau_driver/comau_trajectory`** ([README](comau_trajectory/README.md))  
**`comau_driver/comau_joint_streamer`** ([README](comau_joint_streamer/README.md))  
**`comau_driver/comau_joint_trajectory_action`** ([README](comau_joint_trajectory_action/README.md))  
**`comau_driver/comau_motion_feedback`** ([README](comau_motion_feedback/README.md))  
**`comau_driver/comau_robot_status`** ([README](comau_robot_status/README.md))  


| Libraries   | Description |
| :---------  |:------------|
| `comau_trajectory` |  It provides the `class JointTrajectoryInterface`. It is the message handler that relays joint trajectories to the robot controller. |
| `comau_joint_streamer` | It provides the `class ComauJointTrajectoryStreamer`. it is the message handler that streams joint trajectories to the robot controller. |
| `comau_joint_trajectory_action` | It provides the `class ComauJointTrajectoryAction`. It is the TrajcetoryActionServer. |
| `comau_motion_feedback` |   	It provides the `class MotionFeedbackComau`. It is the message handler that receives the motion feedback from the robot. |


| Nodes | Description |
| :------|:--------------|
| `comau_joint_streamer_node` | It handles the trajectory loading and the dispatching of the points to the robot controller. |
| `comau_joint_trajectory_action_node` | It handles the trajectory action server. | 
| `comau_motion_feedback_node` | It handles the robot feedback and the trajectory execution status |


| Services | Type | Description | 
|:--- | :----  | :------------------ | 
| `/start_motion` | industrial_msgs::StartMotion | Send to the robot the start motion command. It trigger the start of a new trajectory, or a previous stopped trajectory |
| `/stop_motion` | industrial_msgs::StopMotion | Send to the robot the stop motion command, the motion can be resumed calling /start_motion |
| `/cancel_motion` | industrial_msgs::StopMotion | Send to the robot the cancel motion command, the motion cannot be resumed and the trajectory on the controller is deleted |
| `/joint_path_command` | comau_msgs::CmdJointTrjComau | Load a new trajectory and send it to the robot controller | 
| `/reload_trj` | _industrial_msgs::StartMotion | Re-send the previously loaded trajectory to the robot controller |

| Topics | Type | Description | 
|:--- | :----  | :------------------ | 
| `/joint_states` | sensor_msgs::JointState | ROS-I standard feedback |
| `/motion_feedback` | comau_msgs::MotionFeedbackComau | Feedback with some additional information ([doc here](comau_motion_feedback/README.md))|
| `/comau_trajectory_sts` | std_msgs::Int32 | Status of the trajectory ([doc here](comau_joint_streamer/README.md))|


# Usage Notes
1. **Driver Architecture:** The driver architecture is both a _"streamer"_ and a _"downloader"_ as defined at http://wiki.ros.org/Industrial/supported_hardware.   

2. **Starting the Motion Execution:** The **start_motion** command is given by the user (on purpose, for sake of security), otherwise the robot will not start the movement.
> * The **start_motion** is a ROS-service provided by `comau_joint_streamer_node` (class `JointTrajectoryInterface`, declaration in ```comau_joint_trajectory_interface.h```).
> The ROS-service **start_motion** is of type `industrial_msgs::StartMotion`. 
> The callback `start_motion` in the COMAU Robot Controller is declared in `../PDL_programs/Single_Arm/ROS_COMAU_motion.pdl`
>
> * The service **start_motion** can be called _**before**_ the trajectory download is finished.
> A thread on the ROS side sends the points of the trajectory, and the robot movement is allowed also during the trajectory download (whether unprocessed points are still in the circular buffer).

3. **Trajectory Processing:** The ROS-node stores the trajectory in a queue (no fixed capacity, and therefore the trajectory length is not limited), while the the trajectory on the robot controller side is stored in a circular buffer of capacity of 200 points.  
> * The ROS-node and the COMAU robot controller communicate through a TCP/IP connection.  
> * The ROS-node extracts one point at time from the trajectory queue.   
> * The ROS-node keeps sending the point to the COMAU robot controller as long as the circular buffer in the robot controller is not completely full.  
> * The movement interpolation is always a Cartesian Linear movement (_extension to joint movement asap_). 
> * The `MOVEFLY` command from COMAU is used to interpolate the sequence of points.   
> * The movement on the last trajectory point is a simple `MOVE` command from COMAU interpolator.  
> * The linear speed value to reach each point is mandatory and have to be set by the user in the trajectory definition.  

4. **Robot State Monitoring:** the `comau_driver` can be used just to online monitor the robot state.  
> * The robot positions are in joints coordinates (in degrees, with COMAU conventions). 
> * When a parallogram robot is used (e.g. NJ220), the joint values correspond to the joint vaules of the URDF kinematics. 
>   Pay attention that the actual values in COMAU Joint Convention are therefore different due to the coupling of the kinematics of joint 2 and 3   

5. **Configuration:** The configuration files (YAML) could be different for each robot. Therefore, they are stored in the available `comau_ROBOTNAME_support` packages.  
> * The IP address of the robot controller is loaded in the param server in launching the driver execution 

**Warning:**
Sometimes, when the ROS part of the driver crashes unexpectedly, the internal mutex is not canceled from the memory, hence it must be done manually before restarting the driver.  
To do so, type `rm /dev/shm/sem.jnt_trj_shared_mtx` in a command shell.  


# Launch driver

The package provides a launch file, `launch/comau_driver.launch`  

The launch file loads the generic version of comau_driver (while the proper configurations are loaded by each specific robot implementation, see `comau_robots`).  
Therefore, it is strongly suggeste to launch `comau_ns16hand_driver.launch` or `comau_nj220hand_driver.launch` or other specific robot launch files  

**Usage**:  
`comau_driver.launch robot_ip:=<value> joint_names_config_file:=<value> [only_monitoring:=<true|false>]`  

Arguments:  
- `robot_ip`: is the address of the robot controller (check ping before)  
- `joint_names_config_file`: yaml file with the joint names (check that the names list is equivalent to the set in the URDF)  
- `only_monitoring`: if true, the driver allows just the monitoring of the robot data  

Defaults provided:  
- only_monitoring = false


# Example of Usage

A example of usage of the ROS-I Comau driver is in `comau_driver/test`.  

The trajectory used in the example is defined in the config file `comau_driver/test/example_trj.yaml` that has to be load in the parameter server 
*before* running the code.  

**WARNING!**   
- The example trajectory is made for a Comanu NS16-Hand robot with a free workspace!   
  Change the joint positions if you have a different robot or some constraints on robot movements.   

> Trajectory points are sent using a parallel thread which doesn't interfere with the main program execution.  
> The `comau_trajectory_sts` topic is available to check the status of the trajectory execution.  

**IMPORTANT!**    
- For safety reasons, the `start_motion` command is not given automatically.  

- To start the motion call the service `start_motion` manually in the command shell.  
  `rosservice call /start_motion {} `  

- To cancel the motion call the service `cancel_motion` manually in the command shell.  
  `rosservice call /cancel_motion {} `  

- To stop the motion call the service `stop_motion` manually in the command shell.  
  `rosservice call /stop_motion {} `  

**Note:**  
> A bridge node to convert comau_msgs::ComauJointTrajectoryAction into control_msgs::FollowJointTrajectoryAction will be ready soon. 
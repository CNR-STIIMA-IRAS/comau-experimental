# ROS-I COMAU Driver by CNR-ITIA (www.itia.cnr.it)

The repository contains the implementation of the ROS-Industrial standard for Comau robots, 
developed by the Institute of Industrial Technologies and Automation, of the National Research Council of Italy (CNR-ITIA).

A short description of what each different gathers is reported. See the README.md file stored in each directory for further information.

 1. **comau_driver** ([README](comau_driver/README.md)): all the ROS code.  
 2. **comau_msgs** ([README](comau_msgs/README.md)): all the msgs used by the comau_driver  
 3. **comau_robots** ([README](comau_robots/README.md)): all the stuff to move and simulate different COMAU robots  
 5. **PDL_programs** ([README](PDL_programs/README.md)): all the COMAU programs (PDL2 is the COMAU propretary programming language)  

## How to use the driver to move the robot

The driver needs different configuration for each robot are using. 
Therefore, the launch files should load different configuration files corresponding to the different COMAU robot.

### Robot Side

1. Load all the PDL programs on the robot controller  
2. Activate all of them   
3. Put the robot in "Drive On" state  
4. Press the "Start" button on the robot's TP.  

### PC Side

1. Check that all the PDL programs are running, 
2. Launch the `roscore` (not necessary)
3. Launch the driver of the robot you need to move, e.g.:  
  ` roslaunch comau_ns16arc_support comau_ns16arc_driver.launch`  
  ` roslaunch comau_ns16hand_support comau_ns16hand_driver.launch`  
  ` roslaunch comau_nj220foundry_support comau_nj220foundry_driver.launch`  

The ROS-I Comau driver is running!  
Use services and actions to move the robot!  
All the information about the services, actions and topics provided by the `comau_driver` are [here](comau_driver/README.md)

## How to use the driver to monitoring the robot

In the case the ROS nodes are used just to get the robot feedback, the procedure is slightly simpler to the previous.

### Robot Side

1. Load the PDL programs ```ROS_COMAU_tcp_utils``` and ```ROS_COMAU_motion_feedback``` on the COMAU robot controller   
2. Activate them.  

### PC-Side 

1. Check that all the PDL programs are running,   
2. Launch the ```roscore```  
3. Launch the driver of the robot you need to move specifying that is used just for monitoring the hardware, e.g.:  
   `roslaunch comau_ns16arc_support comau_ns16arc_driver.launch only_monitoring:=true`  
   `roslaunch comau_ns16hand_support comau_ns16hand_driver.launch only_monitoring:=true`  
   `roslaunch comau_nj220foundry_support comau_nj220foundry_driver.launch only_monitoring:=true`  


## Installation and ROS-Version 

This package contains the implementation of the ROS-Industrial standard for Comau robots, developed using the ROS Jade 
(also tested on ROS Kinetic) distribution. 

Clone the repository into your catkin working directory and make it with ```catkin_make```. 

**Note:**  

- The ROS-I (industrial_core) package is needed.
- A bridge node to convert comau_msgs::ComauJointTrajectoryAction into control_msgs::FollowJointTrajectoryAction will be ready soon. 


## Developer Contact

**Authors:**   
- Alberto Marini (alberto.marini@itia.cnr.it)  
- Enrico Villagrossi (enrico.villagrossi@itia.cnr.it)  
- Manuel Beschi (manuel.beschi@itia.cnr.it)  
- Nicola Pedrocchi (nicola.pedrocchi@itia.cnr.it)  
 
_Software License Agreement (BSD License)_    
_Copyright (c) 2016, National Research Council of Italy, Institute of Industrial Technologies and Automation_    
_All rights reserved._
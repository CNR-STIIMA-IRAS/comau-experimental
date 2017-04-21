# PDL Programs

The repository contains the Comau controller proprietary code, 
developed by the Institute of Industrial Technologies and Automation, of the National Research Council of Italy (CNR-ITIA).  

At this time just the Single_Arm package is present, but a Dual_Arm handling package will come soon. The code is developed and tested on a Comau C5G controller; in order to use it
on a C4G machine, some modifications are needed.  

**Warning:**  
- To run all the PDL functionalities it is require the ''socket comunication'' software option installed on the robot controller. 
- It is needed to run the PDL programs before starting the ROS nodes.  
- When ROS nodes are interrupted roughly, the communication to the controller is lost and an error will appear. Before the ROS nodes restart, reset the error, deactivate and activate 
  again all the PDL programs in order to restart the TCP/IP channels.  
- If a PDL program crashes, the communication channel memory is not freed properly. Before restarting the driver, you must deactivate all the PDLs and erease all the program variables
  on the robot controller.  
  
**Note:**  
This is the code where changes are needed in order to use the driver on a Comau C4G controller.  
Some builtin routines for TCP/IP communication have different names between the two controller versions (e.g. DV_CNTRL has to be substituted with DV4_CNTRL); refer to the PDL code manual 
for further informations.  

## ROS_COMAU_tcp_utils

The code is a collection of the TCP/IP communication routines needed by the rest of the programs for data exchange.  

## ROS_COMAU_trajectory

This program is in charge of the trajectory management. The data exchange between this program and its ROS couterpart is based on a structure which copies the `JointTrajPointComau.msg` 
([README](../comau_msgs/README.md)) message plus a ROS-I standard header.  
The code processes the data from the ROS side where, changing the msg_type field value in the header (refer to `../comau_msgs/include/comau_msgs/comau_simple_message.h` for the allowed values) permit to perform
different actions related to the reajectory handling (e.g. add a new point, check the queue, start/stop/cancel the motion).  

## ROS_COMAU_motion

This is an holdable program where the motion along the trajectory is performed. Inside the code, all the motion commands of the driver are present.  
The motion between the trajectory points (except the last one) is made by a MOVEFLY command with cartesian linear interpolation and tolerance on the position reach set to the value defined in each point's definition.  

**Note:**  
Robot frames are not set to zero at the program beginnig, hence the interpolator will use the last frames set by the user.  

## ROS_COMAU_motion_feedback

The program gives useful feedback about the robot motion. Data are passed to the ROS node via a TCP/IP connection on a different port respect to the one of the trajectory communication.  
The data structure is based on the `MotionFeedbackComau.msg` message ([README](../comau_msgs/README.md)) plus a ROS-I standard header.  

## ROS_COMAU_robot_status

The program sends feedback about some important values of the controller state. The information are exchanged with a ROS node over a TCP/IP connection (the port used for this feedback is different from the ones of trajectory and motion_feedback communications).  
The data structure is based on the standard ROS-I `RobotStatus.msg` message.  


## Developer Contact

**Authors:**   
- Enrico Villagrossi (enrico.villagrossi@itia.cnr.it)  
- Manuel Beschi (manuel.beschi@itia.cnr.it)  
- Nicola Pedrocchi (nicola.pedrocchi@itia.cnr.it)  
- Alberto Marini (alberto.marini@unibs.it)  
 
_Software License Agreement (BSD License)_  
_Copyright (c) 2016, National Research Council of Italy, Institute of Industrial Technologies and Automation_  
_All rights reserved._  

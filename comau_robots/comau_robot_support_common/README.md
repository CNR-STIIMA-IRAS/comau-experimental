# COMAU Robot Support Common

The Package is just a collection of launch files.  
Three launch files are provided:  
1. **comau_robot_description.launch**   
2. **comau_robot_driver.launch**  
3. **comau_robot_driver_and_rviz.launch**  

## Note on Usage

### comau_robot_description.launch

The file is the manipulator generic version to load the robot description.  

__Usage__:  
`comau_ns16hand_description.launch  xacro_file_name:=<value> [ test:=<true|false> ]`  


__Arguments__:  
- If `test=false`, the `xacro` generates the urdf file and it loads the resulting file in the parameter  `/robot_description` of the rosparam server.  
- If `test=true`, the visualization is also activated. Therefore, RVIZ is opened with the option `use_gui=true`. Furthermore, the nodes `joint_state_publisher` and `robot_state_publisher` are activated.  


Defaults provided for COMAU ROBOT:
- test = false


### comau_robot_driver.launch

The file is the manipulator generic version of comau_driver's launcher.  
The file is just a wrapper to configure and launch the `comau_driver.launch`, provided by the packages `comau_driver`.  

**Usage**:
`comau_robot_driver.launch robot_ip:=<value> joint_names_config_file:=<value> [only_monitoring:=<true|false>]`  

__Arguments__:   
- `robot_ip`: is the address of the robot controller (check ping before)  
- `joint_names_config_file`: yaml file with the joint names (check that the names are the set in the URDF)  
- `only_monitoring`: if true, the driver allows just the monitoring of the robot data.  

Defaults provided for COMAU ROBOT:  
- `only_monitoring=false`  

### comau_robot_driver_and_rviz.launch

The file is the manipulator specific version of the actual robot state visualizer and driver.

**Usage**:
`robot_state_visualize_comau_ns16hand.launch robot_ip:=<value> xacro_file_name:=<value> joint_names_config_file:=<value> [only_monitoring:=<true|false>] `  

__Arguments__:   
- `robot_ip`: is the address of the robot controller (check ping before)  
- `joint_names_config_file`: yaml file with the joint names (check that the names are the set in the URDF)  
- `only_monitoring`: if true, the driver allow just the monitoring of the robot data.  

Defaults provided for COMAU ROBOT:  
- only_monitoring = false  


## Developer Contact

**Authors:**   
- Alberto Marini (alberto.marini@itia.cnr.it)  
- Enrico Villagrossi (enrico.villagrossi@itia.cnr.it)  
- Manuel Beschi (manuel.beschi@itia.cnr.it)  
- Nicola Pedrocchi (nicola.pedrocchi@itia.cnr.it)  
 
_Software License Agreement (BSD License)_  
_Copyright (c) 2016, National Research Council of Italy, Institute of Industrial Technologies and Automation_  
_All rights reserved._  

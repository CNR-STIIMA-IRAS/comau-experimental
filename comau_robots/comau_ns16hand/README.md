# COMAU NS16-HAND

This package groups all the ROS code to use the COMAU NS16-HAND.  

Three packages are provided:  
1. **comau_ns16hand_support**  
2. **comau_ns16hand_moveit_config**  
3. **comau_ns16hand_moveit_plugin**  



## comau_ns16hand_support

The package provides:   
     - the meshes of the COMAU NS16-HAND    
     - the xacro files with the macro of the COMAU NS16-HAND  
     - the urdf and collada files  
     - the config files (yaml) of the NS16-HAND  
     - the launch files that wrap the launch files stored `comau_robot_support_common/launch`  
     
## comau_ns16hand_moveit_config

The package has been auto-generated from the `setup_assistant` of the package `moveit_setup`

## comau_ns16hand_moveit_plugin

The package provides the analytical inverse kinematics calculated thorugh `ikfast`, and it provides the plugin for moveit. 
The package, therefore has been generated through:  
   1. IKfast solution generated as at http://wiki.ros.org/Industrial/Tutorials/Create_a_Fast_IK_Solution  
   2. Plugin generation as explained at http://wiki.ros.org/Industrial/Tutorials/Create_a_Fast_IK_Solution/moveit_plugin  

**ATTENTION**: two patches have been applied both to the automated-generated files from `ikfast.py` and from `moveit_ikfast`.  

In order to compile the file with the patch, please define the key `ITIA_CNR_PATCH` in the `CMakeLists.txt` (e.g., `add_definitions( "-DITIA_CNR_PATCH=1" )` )  
In order to compile the file without the patch, just check that the key `ITIA_CNR_PATCH` is not defined in `CMakeLists.txt` (or whenever in other places)  

The two patches are:  
1. the method `ComputeIk` defined in the file `comau_ns16hand_manipulator_ikfast_solver.cpp` has been partially modified.   
   The function gets as input a matrix of float, but it is not guaranteed that the matrix is homogeneous.   
   The patch simply uses Eigen to make the input matrix homogeneous.  

2. the patch takes inspiration from the `ur_kinematics` ( https://github.com/ros-industrial/universal_robot ). The patch checks in the `robot_description` the name of the links, and it assumes that the ikfast solution is just the solver from the base to the center of the flange. Therefore, it compensates the fixed rototraslation from the generic world to the robot base and from the active end effector frame to the robot flange.  


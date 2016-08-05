# COMAU NJ220-FOUNDRY

This package groups all the ROS code to use the COMAU NJ220-FOUNDRY.  

Three packages are provided:  
1. **comau_nj220foundry_support**  
2. **comau_nj220foundry_moveit_config**  
3. **comau_nj220foundry_moveit_plugin**:The package cannot be compiled in jade due to error in the `moveit_ikfast` distribution. Therefore, just the zip file is provided. If you want to use it you have to unzip the file and follow the below reported instructions.



## comau_nj220foundry_support

The package provides:   
     - the meshes of the COMAU NJ220-FOUNDRY  
     - the xacro files with the macro of the COMAU NJ220-FOUNDRY
     - the urdf, and collada files
     - the config files (yaml) of the NJ220-FOUNDRY
     - the launch files that wrap the launch files stored `comau_robot_support_common/launch`
     
## comau_nj220foundry_moveit_config

The package has been auto-generated from the `setup_assistant` of the package `moveit_setup`

## comau_nj220foundry_moveit_plugin

The package provides the analytical inverse kinematics calculated thorugh `ikfast`, and it provides the plugin for moveit. 
The package, therefore has been generated through:  
   1. IKfast solution generated as at http://wiki.ros.org/Industrial/Tutorials/Create_a_Fast_IK_Solution  
   2. Plugin generation as explained at http://wiki.ros.org/Industrial/Tutorials/Create_a_Fast_IK_Solution/moveit_plugin  

**ATTENTION**: two patches have been applied both to the automated-generated files from `ikfast.py` and from `moveit_ikfast`.  

In order to compile the file with the patch, please define the key `ITIA_CNR_PATCH` in the `CMakeLists.txt` (e.g., `add_definitions( "-DITIA_CNR_PATCH=1" )` )  
In order to compile the file without the patch, just check that the key `ITIA_CNR_PATCH` is not defined in `CMakeLists.txt` (or whenever in other places)  

The two patches are:  
1. the method `ComputeIk` defined in the file `comau_nj220foundry_manipulator_ikfast_solver.cpp` has been partially modified.   
   The function gets as input a matrix of float, but it is not guaranteed that the matrix is homogeneous.   
   The patch simply uses Eigen to make the input matrix homogeneous.  

2. the patch takes inspiration from the `ur_kinematics` ( https://github.com/ros-industrial/universal_robot ). The patch checks in the `robot_description` the name of the links, and it assumes that the ikfast solution is just the solver from the base to the center of the flange. Therefore, it compensates the fixed rototraslation from the generic world to the robot base and from the active end effector frame to the robot flange.  


**ISSUE**: The package cannot be compiled in jade due to issues in the `moveit_core` distribution.   Specifically:

1. The package `ros-jade-moveit-core` is not coherent with https://github.com/ros-planning/moveit_core/tree/jade-devel. Indeed, the file `kinematics_base.h` given with the debian package does not provide the class `KinematicsResult` that is actually provided by https://github.com/ros-planning/moveit_core/blob/jade-devel/kinematics_base/include/moveit/kinematics_base/kinematics_base.h 

2. The debian package `ros-jade-moveit-ikfast` does not exit, and the last updated debian package is `ros-indigo-moveit-ikfast`. It you download the package from https://github.com/ros-planning/moveit_ikfast (branch `indigo-devel` or `jade-devel` ) they need the definition of `KinematicsResult` in `kinematics_base.h`.

Therefore, in order to compile the plugin, you need to download from source `moveit_core` and its dependencies and compile it.




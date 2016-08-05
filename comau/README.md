# comau

The comau metapackage contains the implementation of the ROS-Industrial standard for Comau robots, 
developed by the Institute of Industrial Technologies and Automation, of the National Research Council of Italy (CNR-ITIA).

 * comau_driver: all the ROS code.
 * comau_msgs: all the msgs used by the comau_driver
 * comau_ns16: all the stuff to move and simulate the ns16 robots
 * PDL_programs: all the COMAU programs (PDL2 is the COMAU propretary programming language)

## Installation

Clone the repository into your catkin working directory and make it with ```catkin_make```.   
**Note:**  The ROS-I package is needed.
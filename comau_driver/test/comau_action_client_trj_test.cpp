/*********************************************************************
 *
 * Authors: Alberto Marini (alberto.marini@itia.cnr.it)
 *          Enrico Villagrossi (enrico.villagrossi@itia.cnr.it)
 *          Manuel Beschi (manuel.beschi@itia.cnr.it)
 *          Nicola Pedrocchi (nicola.pedrocchi@itia.cnr.it)
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, National Research Council of Italy, Institute of Industrial Technologies and Automation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the National Research Council of Italy nor the 
 *     names of its contributors may be used to endorse or promote products 
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ros/ros.h>
#include <simple_message/shared_types.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <industrial_utils/param_utils.h>

#include <comau_msgs/ComauJointTrajectoryAction.h>
#include <comau_msgs/ComauJointTrajectoryGoal.h>
#include <comau_msgs/ComauJointTrajectoryResult.h>
#include <comau_msgs/JointTrajPointComau.h>

// Before running this code, load the example trajectory (or your own) in the parameter server with the command
// rosparam load example_trj.yaml
// from the linux terminal (be careful that the example trajectory is made for a Comanu NS16-Hand robot!!!).
// The example_trj.yaml file is placed in the test folder of the comau_driver.

int main (int argc, char **argv)
{
  ros::init(argc, argv, "comau_action_client_trj_test");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<comau_msgs::ComauJointTrajectoryAction> ac("/comau_joint_trajectory_action", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); // no argument means it will wait for infinite time
  
  std::string traj;
  traj = "/test_trj"; // name of the trajectory structure in the parameter server
   if(!ros::param::has(traj))    // check if the speficied field is present in the ROS parameter server
  {
    ROS_ERROR("Trajectory is not present in the parameter server!");
    return false;
  }
  
  int nPoints = 0;
  ros::param::param<int>(traj+"/nPoints", nPoints, -1);  // copy the value of the field in the parameter server into the 
                                                         // variable, assign value -1 if not found
  if(nPoints==0)
  {
    ROS_ERROR("The trajectory is empty!");
    return false;
  }
  if(nPoints==-1)
  {
    ROS_ERROR("Impossible to find '/nPoints' in the parameter server!");
    return false;
  }

  #ifdef DUAL_ARM   // definition in the CMakeLists file of comau-jade-devel, used to distinguish if the configuration of the robot is single-arm or dual-arm
    static const industrial::shared_types::shared_int MAX_NUM_ARMS = 2;
  #else
    static const industrial::shared_types::shared_int MAX_NUM_ARMS = 1;
  #endif

  // variables declarations
  std::string tmp_str;
  industrial::shared_types::shared_int arm_number, digital_output, sequence;
  industrial::shared_types::shared_real fly_tolerance;
  std::vector<industrial::shared_types::shared_real> velocity, joint_pos;
  comau_msgs::ComauJointTrajectoryGoal msg;
  std::vector<std::string> joint_names;
  // retrive joint names from the the parameter server structure (first argument) or the robot description in the 
  // parameter server (second argument), otherwise it sets default values
  industrial_utils::param::getJointNames("controller_joint_names", "robot_description", joint_names);
  // fill the goal message structure for the action
  msg.goal.joint_names = joint_names;
  msg.goal.points.resize(nPoints);
  for (int i=0; i<nPoints; ++i)
  {
    tmp_str = traj+"/point_"+std::to_string(i);
    // retrive parameters values
    if(!ros::param::get(tmp_str+"/arm_number", arm_number))
    {
      ROS_WARN("Arm number not defined for point_%d. Default value (1) set.", i);
      arm_number = 1;
    }
    if(!ros::param::get(tmp_str+"/fly_tolerance", fly_tolerance))
    {
      ROS_WARN("Fly tolerance not defined for point_%d. Default value (1.0) set.", i);
      fly_tolerance = 1.0;
    }
    if(!ros::param::get(tmp_str+"/digital_output",digital_output))
    {
      ROS_WARN("Digital output not defined for point_%d. Default value (0) set.", i);
      digital_output = 0;
    }
    if(!ros::param::get(tmp_str+"/velocity",velocity))
    {
      ROS_ERROR("Velocity not defined for point_%d. Trajectory loading aborted!", i);
      return false;
    }
    if(!ros::param::get(tmp_str+"/joint_position",joint_pos))
    {
      ROS_ERROR("Joint position not defined for point_%d. Trajectory loading aborted!", i);
      return false;
    }
    sequence = i+1;

    velocity.resize(MAX_NUM_ARMS,0.0);
    joint_pos.resize((MAX_NUM_ARMS*10),0.0);

    msg.goal.points.at(i).arm_number = 1;
    msg.goal.points[i].queue_status = 0;
    msg.goal.points[i].sequence_number = sequence;
    msg.goal.points[i].fly_tolerance = fly_tolerance;
    msg.goal.points[i].linear_velocity = velocity;
    msg.goal.points[i].joint_positions = joint_pos;
    msg.goal.points[i].digital_output = digital_output;
    
  }
  
  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  ac.sendGoal(msg);

  // wait for the action response
  ac.waitForResult(); // no argument means it will wait for infinite time
  comau_msgs::ComauJointTrajectoryResultConstPtr res = ac.getResult();
  
  std::cout << "Result: " << res->result << std::endl;
  
  //exit
  return 0;
}
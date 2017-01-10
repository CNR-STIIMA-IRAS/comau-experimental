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

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryResult.h>

// Before running this code, load the example trajectory (or your own) in the parameter server with the command
// rosparam load example_trj.yaml
// from the linux terminal (be careful that the example trajectory is made for a Comanu NS16-Hand robot!!!).
// The example_trj.yaml file is placed in the test folder of the comau_driver.

int main (int argc, char **argv)
{
  ros::init(argc, argv, "standard_action_client_trj_test");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/joint_trajectory_action", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); // no argument means it will wait for infinite time
  ROS_INFO("Action server attached.");
  
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
  std::vector<std::string> joint_names;
  std::vector<double> velocity, joint_pos;
  control_msgs::FollowJointTrajectoryGoal msg;

  
  // retrive joint names from the the parameter server structure (first argument) or the robot description in the 
  // parameter server (second argument), otherwise it sets default values
  industrial_utils::param::getJointNames("controller_joint_names", "robot_description", joint_names);
  // fill the goal message structure for the action
  msg.trajectory.joint_names = joint_names;
  msg.trajectory.points.resize(nPoints);
  for (int i=0;i<nPoints; ++i)
  {
    tmp_str = traj+"/point_"+std::to_string(i);
    ros::param::get(tmp_str+"/velocity",velocity);
    ros::param::get(tmp_str+"/joint_position",joint_pos);
    
    velocity.resize(MAX_NUM_ARMS,0.0);
    joint_pos.resize((MAX_NUM_ARMS*10),0.0);
    msg.trajectory.points[i].positions.resize(joint_pos.size()); 
    msg.trajectory.points[i].velocities.resize(velocity.size());
    
    msg.trajectory.points[i].positions = joint_pos;
    msg.trajectory.points[i].velocities = velocity;
  }
  
  
  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  ac.sendGoal(msg);

  // wait for the action response
  ac.waitForResult(); // no argument means it will wait for infinite time
  control_msgs::FollowJointTrajectoryResultConstPtr res = ac.getResult();
  
  if (res->error_code == 0)
    std::cout << "Result: Successful!" << std::endl;
  else
    std::cout << "Result: " << res->error_string << " (error_code: " << res->error_code << " )" << std::endl;
  
  //exit
  return 0;
}
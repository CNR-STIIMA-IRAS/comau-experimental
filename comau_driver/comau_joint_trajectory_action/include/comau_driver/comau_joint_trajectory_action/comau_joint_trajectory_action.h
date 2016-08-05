
/*********************************************************************
 *
 * Provides the ComauJointTrajectoryAction
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

#ifndef COMAU_JOINT_TRAJECTORY_ACTION_H
#define COMAU_JOINT_TRAJECTORY_ACTION_H

#include <ros/ros.h>
#include <vector>
#include <string>
#include <actionlib/server/simple_action_server.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <comau_msgs/JointTrajComau.h>
#include <comau_msgs/CmdJointTrjComau.h>
#include <comau_msgs/ComauJointTrajectoryAction.h>
#include <industrial_msgs/ServiceReturnCode.h>

namespace comau
{
  /** @ns comau_joint_trajectory_action */
namespace comau_joint_trajectory_action
{

class ComauJointTrajectoryAction
{

public:
  /**
   * \brief Constructor
   *
   */
  ComauJointTrajectoryAction();

  /**
   * \brief Destructor
   *
   */
  ~ComauJointTrajectoryAction();

  /**
     * \brief Begin processing messages and publishing topics.
     */
    void run() { ros::spin(); }

private:

  typedef actionlib::SimpleActionServer<comau_msgs::ComauJointTrajectoryAction> JointTractoryActionServer;

  /**
   * \brief Internal ROS node handle
   */
  ros::NodeHandle node_;
  
  ros::ServiceClient comau_trajectory_client_;

  /**
   * \brief Internal action server
   */
  JointTractoryActionServer action_server_;
  /**
   * \brief Cache of the current active goal
   */
//   JointTractoryActionServer::GoalConstPtr active_goal_;
  /**
   * \brief Cache of the current active trajectory
   */
  comau_msgs::JointTrajComau current_traj_;
  /**
   * \brief The joint names associated with the robot the action is
   * interfacing with.  The joint names must be the same as expected
   * by the robot driver.
   */
  std::vector<std::string> joint_names_;
  /**
   * \brief Action server goal callback method
   *
   * \param gh goal handle
   *
   */
  void goalCB(const JointTractoryActionServer::GoalConstPtr& gh);
  
  };

} //comau_joint_trajectory_action
} //comau

#endif /* COMAU_JOINT_TRAJECTORY_ACTION_H */

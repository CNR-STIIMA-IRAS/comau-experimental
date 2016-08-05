
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

#include <industrial_robot_client/utils.h>
#include <industrial_utils/param_utils.h>
#include <industrial_utils/utils.h>
#include <comau_driver/comau_joint_trajectory_action/comau_joint_trajectory_action.h>

namespace comau
{
namespace comau_joint_trajectory_action
{

// const double JointTrajectoryAction::WATCHDOG_PERIOD_ = 1.0;
// const double JointTrajectoryAction::DEFAULT_GOAL_THRESHOLD_ = 0.01;

ComauJointTrajectoryAction::ComauJointTrajectoryAction() :
    action_server_(node_, "joint_trajectory_action", boost::bind(&ComauJointTrajectoryAction::goalCB, this, _1),
                   /*boost::bind(&ComauJointTrajectoryAction::cancelCB, this, _1),*/ false)
{
  ros::NodeHandle pn("~");
  
  if (!industrial_utils::param::getJointNames("controller_joint_names", "robot_description", joint_names_))
    ROS_ERROR("Failed to initialize joint_names.");
  
  comau_trajectory_client_ = pn.serviceClient<comau_msgs::CmdJointTrjComau>("/joint_path_command");
  
  ROS_INFO("Action Server on topic /joint_trajectory_action created! ");
  
  action_server_.start();
}

ComauJointTrajectoryAction::~ComauJointTrajectoryAction()
{
}

void ComauJointTrajectoryAction::goalCB(const JointTractoryActionServer::GoalConstPtr & gh)
{
  ROS_INFO("Received new goal");

  if (!gh.get()->goal.points.empty())
  {
    if (industrial_utils::isSimilar(joint_names_, gh.get()->goal.joint_names))
    {
//       gh.setAccepted();
//       active_goal_ = gh;      
      current_traj_ = gh.get()->goal;
      comau_msgs::CmdJointTrjComau client_srv;
      client_srv.request.trajectory = current_traj_;

      comau_msgs::ComauJointTrajectoryResult res;

      if ( comau_trajectory_client_.call( client_srv ) )
      {
        res.result = client_srv.response.code;
        action_server_.setSucceeded(res);
      }
      else
      {
        ROS_ERROR("Fail to call client_srv");
        res.result.val = industrial_msgs::ServiceReturnCode::FAILURE;
        action_server_.setAborted(res);
      }    
    }
    else
    {
      ROS_ERROR("Joint trajectory action failing on invalid joints");
      comau_msgs::ComauJointTrajectoryResult res;
      res.result.val = industrial_msgs::ServiceReturnCode::FAILURE;
      action_server_.setAborted(res, "Joint names do not match");
    }
  }
  else
  {
    ROS_ERROR("Joint trajectory action failed on empty trajectory");
    comau_msgs::ComauJointTrajectoryResult res;
    res.result.val = industrial_msgs::ServiceReturnCode::FAILURE;
    action_server_.setAborted(res, "Empty trajectory");
  }

  return;
}


} //comau_joint_trajectory_action
} //comau


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

#include <industrial_utils/param_utils.h>
#include <comau_driver/comau_motion_feedback/motion_feedback_comau_interface.h>

using industrial::smpl_msg_connection::SmplMsgConnection;
using industrial_utils::param::getJointNames;
namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;

namespace comau
{
namespace motion_feedback_comau_interface
{

MotionFeedbackComauInterface::MotionFeedbackComauInterface()
{
  this->connection_ = NULL;
//   this->add_handler(&default_joint_handler_);
  this->add_handler(&motion_feedback_handler_);
}

bool MotionFeedbackComauInterface::init(std::string default_ip, int default_port)
{
  std::string ip;
  int port;

  // override IP/port with ROS params, if available
  ros::param::param<std::string>("robot_ip_address", ip, default_ip);
  ros::param::param<int>("~motion_feedback_port", port, default_port);

  // check for valid parameter values
  if (ip.empty())
  {
    ROS_ERROR("No valid robot IP address found.  Please set ROS 'robot_ip_address' param");
    return false;
  }
  if (port <= 0)
  {
    ROS_ERROR("No valid robot IP port found.  Please set ROS '~motion_feedback_port' param");
    return false;
  }

  char* ip_addr = strdup(ip.c_str());  // connection.init() requires "char*", not "const char*"
  ROS_INFO("Motion feedback connecting to IP address: '%s:%d'", ip_addr, port);
  default_tcp_connection_.init(ip_addr, port);
  free(ip_addr);

  return init(&default_tcp_connection_);
}

bool MotionFeedbackComauInterface::init(SmplMsgConnection* connection)
{
  std::vector<std::string> joint_names;
  if (!getJointNames("controller_joint_names", "robot_description", joint_names))
  {
    ROS_ERROR("Failed to initialize joint_names.  Aborting");
    return false;
  }

  return init(connection, joint_names);
}

bool MotionFeedbackComauInterface::init(SmplMsgConnection* connection, std::vector<std::string>& joint_names)
{
  this->joint_names_ = joint_names;
  this->connection_ = connection;
  connection_->makeConnect();

  // initialize message-manager
  if (!manager_.init(connection_))
    return false;

  // initialize handlers
  if (!motion_feedback_handler_.init(connection_, joint_names_))
      return false;
  this->add_handler(&motion_feedback_handler_);

  return true;
}

void MotionFeedbackComauInterface::run()
{
  manager_.spin();
}

} // motion_feedback_comau_interface
} // comau

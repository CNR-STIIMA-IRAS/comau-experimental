
/*********************************************************************
 *
 * Provides the MotionFeedbackComauInterface Structures
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

#ifndef MOTION_FEEDBACK_COMAU_INTERFACE_H
#define MOTION_FEEDBACK_COMAU_INTERFACE_H

#include <vector>
#include <string>
#include <simple_message/smpl_msg_connection.h>
#include <simple_message/message_manager.h>
#include <simple_message/message_handler.h>
#include <simple_message/socket/tcp_client.h>
#include <industrial_robot_client/joint_relay_handler.h>
#include <comau_driver/comau_motion_feedback/motion_feedback_comau_relay_handler.h>
#include <comau_msgs/comau_simple_message.h>

namespace comau
{
  /** @ns motion_feedback_comau_interface */
namespace motion_feedback_comau_interface
{

using industrial::smpl_msg_connection::SmplMsgConnection;
using industrial::message_manager::MessageManager;
using industrial::message_handler::MessageHandler;
using industrial::tcp_client::TcpClient;
using industrial_robot_client::joint_relay_handler::JointRelayHandler;
using comau::motion_feedback_comau_relay_handler::MotionFeedbackComauRelayHandler;


/**
 * \brief Generic template that reads motion-feedback-data from a robot controller
 * and publishes matching messages to various ROS topics.
 */
class MotionFeedbackComauInterface
{

public:

  /**
   * \brief Default constructor.
   */
  MotionFeedbackComauInterface();

  /**
   * \brief Initialize robot connection using default method.
   *
   * \param default_ip default IP address to use for robot connection [OPTIONAL]
   *                    - this value will be used if ROS param "robot_ip_address" cannot be read
   * \param default_port default port to use for robot connection [OPTIONAL]
   *                    - this value will be used if ROS param "~port" cannot be read
   *
   * \return true on success, false otherwise
   */
  bool init(std::string default_ip = "", int default_port = comau::ComauControllerPorts::ComauControllerPort::MOTION_FEEDBACK);


  /**
   * \brief Initialize robot connection using specified method.
   *
   * \param connection new robot-connection instance (ALREADY INITIALIZED).
   *
   * \return true on success, false otherwise
   */
  bool init(SmplMsgConnection* connection);

  /**
   * \brief Initialize robot connection using specified method and joint-names.
   *
   * \param connection new robot-connection instance (ALREADY INITIALIZED).
   * \param joint_names list of joint-names for ROS topic
   *   - Count and order should match data sent to robot connection.
   *   - Use blank-name to skip (not publish) a joint-position
   *
   * \return true on success, false otherwise
   */
  bool init(SmplMsgConnection* connection, std::vector<std::string>& joint_names);

  /**
   * \brief Begin processing messages and publishing topics.
   */
  void run();

  /**
   * \brief get current robot-connection instance.
   *
   * \return current robot connection object
   */
  SmplMsgConnection* get_connection()
  {
    return this->connection_;
  }

  /**
   * \brief get active message-manager object
   *
   * \return current message-manager object
   */
  MessageManager* get_manager()
  {
    return &this->manager_;
  }

  /**
   * \brief get robot joint names
   *
   * \return vector of joint names
   */
  std::vector<std::string> get_joint_names()
  {
    return this->joint_names_;
  }


  /**
   * \brief Add a new handler.
   *
   * \param handler new message-handler for a specific msg-type (ALREADY INITIALIZED).
   * \param allow_replace replace existing handler (of same msg-type), if exists
   */
  void add_handler(MessageHandler* handler, bool allow_replace = true)
  {
    this->manager_.add(handler, allow_replace);
  }

protected:
  TcpClient default_tcp_connection_;
  MotionFeedbackComauRelayHandler motion_feedback_handler_;

  SmplMsgConnection* connection_;
  MessageManager manager_;
  std::vector<std::string> joint_names_;

}; //class MotionFeedbackComauInterface

}  //motion_feedback_comau_interface
}  //comau


#endif /* MOTION_FEEDBACK_COMAU_INTERFACE_H */

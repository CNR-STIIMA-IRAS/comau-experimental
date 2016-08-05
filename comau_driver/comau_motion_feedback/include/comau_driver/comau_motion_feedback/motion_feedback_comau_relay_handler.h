
/*********************************************************************
 *
 * Provides the MessageHandler Structures
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

#ifndef MOTION_FEEDBACK_COMAU_RELAY_HANDLER_H
#define MOTION_FEEDBACK_COMAU_RELAY_HANDLER_H

#include <cmath>
#include <ros/ros.h>
#include <simple_message/message_handler.h>
#include <comau_driver/comau_motion_feedback/motion_feedback_comau_message.h>

namespace comau
{
  /** @ns motion_feedback_comau_relay_handler */
namespace motion_feedback_comau_relay_handler
{

/**
 * \brief Message handler that relays motion feedbacks (converts simple message
 * types to ROS message types and publishes them)
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class MotionFeedbackComauRelayHandler : public industrial::message_handler::MessageHandler
{
  // since this class defines a different init(), this helps find the base-class init()
  using industrial::message_handler::MessageHandler::init;

public:

  /**
   * \brief Constructor
   */
  MotionFeedbackComauRelayHandler() {};

  /**
   * \brief Class initializer
   *
   * \param connection: simple message connection that will be used to send replies.
   * \param joint_names: list of robot joint names.
   *
   * \return true on success, false otherwise (an invalid message type)
   */
  bool init(industrial::smpl_msg_connection::SmplMsgConnection* connection, std::vector<std::string>& joint_names);

protected:

  std::vector<std::string> all_joint_names_;

  ros::Publisher pub_joint_state_;
  ros::Publisher pub_motion_feedback_;
  ros::NodeHandle node_;

  /**
   * \brief Callback executed upon receiving a motion feedback message
   *
   * \param in: incoming message
   *
   * \return true on success, false otherwise
   */
  bool internalCB(comau::motion_feedback_comau_message::MotionFeedbackComauMessage & in);

private:
  
  /**
    * \brief maximum number of robot arms that can be held.
    */
  #ifdef DUAL_ARM
    static const industrial::shared_types::shared_int MAX_NUM_ARMS = 2;
  #else
    static const industrial::shared_types::shared_int MAX_NUM_ARMS = 1;
  #endif
  
  /**
   * \brief Callback executed upon receiving a message
   *
   * \param in: incoming message
   *
   * \return true on success, false otherwise
   */
  bool internalCB(industrial::simple_message::SimpleMessage& in);
};

} // motion_feedback_comau_relay_handler
} // comau

#endif /* MOTION_FEEDBACK_COMAU_RELAY_HANDLER_H */

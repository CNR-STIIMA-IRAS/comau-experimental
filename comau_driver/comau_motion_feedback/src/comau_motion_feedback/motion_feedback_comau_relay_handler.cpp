


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

#include <vector>
#include <sensor_msgs/JointState.h>
#include <simple_message/log_wrapper.h>

#include <comau_msgs/MotionFeedbackComau.h>
#include <comau_driver/comau_motion_feedback/motion_feedback_comau_relay_handler.h>


using namespace industrial::shared_types;
using namespace industrial::smpl_msg_connection;
using namespace industrial::simple_message;
using namespace comau::motion_feedback_comau;
using namespace comau::motion_feedback_comau_message;

namespace comau
{
namespace motion_feedback_comau_relay_handler
{

bool MotionFeedbackComauRelayHandler::init(SmplMsgConnection* connection, std::vector<std::string>& joint_names)
{
  this->pub_motion_feedback_ = this->node_.advertise<comau_msgs::MotionFeedbackComau>("motion_feedback", 1);
  this->pub_joint_state_ = this->node_.advertise<sensor_msgs::JointState>("joint_states",1);

  this->all_joint_names_ = joint_names;
  
  return init((int)comau::simple_message::ComauMsgTypes::MOTION_FEEDBACK, connection);
}

bool MotionFeedbackComauRelayHandler::internalCB(SimpleMessage& in)
{
  MotionFeedbackComauMessage feedback_msg;

  if (!feedback_msg.init(in))
  {
    LOG_ERROR("Failed to initialize motion feedback message");
    return false;
  }

  return internalCB(feedback_msg);
}

bool MotionFeedbackComauRelayHandler::internalCB(MotionFeedbackComauMessage & in)
{
  comau_msgs::MotionFeedbackComau feedback;
  sensor_msgs::JointState joint_state;
  bool rtn = true;
  std::vector<industrial::shared_types::shared_real> vec;

  feedback.header.stamp = ros::Time::now();
  joint_state.header.stamp = ros::Time::now();
  joint_state.name = this->all_joint_names_;
  feedback.arm_number = in.feedback_.getArmNumber();
  for(int i=0; i<MAX_NUM_ARMS; i++)
  {
    for(int j=0; j<in.feedback_.getBaseFrame().at(i).getMaxNumCoords(); j++)
      vec.push_back(in.feedback_.getBaseFrame().at(i).getCoord(j));
  }
  feedback.base_frame = vec;
  vec.clear();
  for(int i=0; i<MAX_NUM_ARMS; i++)
  {
    for(int j=0; j<in.feedback_.getUserFrame().at(i).getMaxNumCoords(); j++)
      vec.push_back(in.feedback_.getUserFrame().at(i).getCoord(j));
  }
  feedback.user_frame = vec;
  vec.clear();
  for(int i=0; i<MAX_NUM_ARMS; i++)
  {
    for(int j=0; j<in.feedback_.getToolFrame().at(i).getMaxNumCoords(); j++)
      vec.push_back(in.feedback_.getToolFrame().at(i).getCoord(j));
  }
  feedback.tool_frame = vec;
  vec.clear();
  for(int i=0; i<MAX_NUM_ARMS; i++)
  {
    for(int j=0; j<in.feedback_.getJointPos().at(i).getMaxNumJoints(); j++)
      vec.push_back(in.feedback_.getJointPos().at(i).getJoint(j));
  }
  feedback.joint_positions = vec;
  std::vector<double> d_vec;
  for (auto it=vec.begin(); it!=vec.end(); it++)
    d_vec.push_back( *it * M_PI/180 );
  joint_state.position = d_vec;
  
  vec.clear();
  for(int i=0; i<MAX_NUM_ARMS; i++)
  {
    for(int j=0; j<in.feedback_.getJointMtrCurr().at(i).getMaxNumJoints(); j++)
      vec.push_back(in.feedback_.getJointMtrCurr().at(i).getJoint(j));
  }
  feedback.joint_motor_currents = vec;
  d_vec.clear();
  for (auto it=vec.begin(); it!=vec.end(); it++)
    d_vec.push_back( *it );
  joint_state.effort = d_vec;
  
  vec.clear();
  d_vec.clear();
  for(int i=0; i<MAX_NUM_ARMS; i++)
  {
    for(int j=0; j<in.feedback_.getCartesianPos().at(i).getMaxNumCoords(); j++)
      vec.push_back(in.feedback_.getCartesianPos().at(i).getCoord(j));
  }
  feedback.cartesian_positions = vec;
  vec.clear();
  // aggiungere flag
  
  feedback.last_point_processed = in.feedback_.getLastPointProcessed(); 
  feedback.cntrl_time = in.feedback_.getTime(); 
  feedback.is_moving = in.feedback_.getIsMoving();
  
  
  this->pub_motion_feedback_.publish(feedback);
  this->pub_joint_state_.publish(joint_state);

  // Reply back to the controller if the sender requested it.
  if (CommTypes::SERVICE_REQUEST == in.getCommType())
  {
    SimpleMessage reply;
    in.toReply(reply, rtn ? ReplyTypes::SUCCESS : ReplyTypes::FAILURE);
    this->getConnection()->sendMsg(reply);
  }
  return rtn;
}

} // motion_feedback_comau_relay_handler
} // comau


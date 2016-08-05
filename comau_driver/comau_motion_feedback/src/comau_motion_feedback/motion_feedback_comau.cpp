
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

#include <simple_message/shared_types.h>
#include <simple_message/log_wrapper.h>
#include <comau_driver/comau_motion_feedback/motion_feedback_comau.h>

using namespace industrial::shared_types;

namespace comau
{
namespace motion_feedback_comau
{

MotionFeedbackComau::MotionFeedbackComau(void) : base_frame_(MAX_NUM_ARMS), user_frame_(MAX_NUM_ARMS), tool_frame_(MAX_NUM_ARMS), 
                                                 joint_positions_(MAX_NUM_ARMS), joint_motor_currents_(MAX_NUM_ARMS), 
                                                 cartesian_positions_(MAX_NUM_ARMS)
{
  this->init();
}

MotionFeedbackComau::~MotionFeedbackComau(void)
{

}

void MotionFeedbackComau::init()
{
  for(int i=0;i<MAX_NUM_ARMS;i++)
    this->base_frame_.at(i).init();
  for(int i=0;i<MAX_NUM_ARMS;i++)
    this->user_frame_.at(i).init();
  for(int i=0;i<MAX_NUM_ARMS;i++)
    this->tool_frame_.at(i).init();
  for(int i=0;i<MAX_NUM_ARMS;i++)
    this->joint_positions_.at(i).init();
  for(int i=0;i<MAX_NUM_ARMS;i++)
    this->joint_motor_currents_.at(i).init();
  for(int i=0;i<MAX_NUM_ARMS;i++)
    this->cartesian_positions_.at(i).init();
  
  this->arm_number_ = 0;
  this->config_flags_ = "";
  this->last_point_processed_ = 0;
  this->time_ = 0;
  this->is_moving_ = 0;
}

void MotionFeedbackComau::copyFrom(MotionFeedbackComau &src)
{
  this->setArmNumber(src.getArmNumber());
  this->setBaseFrame(src.getBaseFrame());
  this->setUserFrame(src.getUserFrame());
  this->setToolFrame(src.getToolFrame());
  this->setJointPos(src.getJointPos());
  this->setJointMtrCurr(src.getJointMtrCurr());
  this->setCartesianPos(src.getCartesianPos());
  this->setConfigFlags(src.getConfigFlags());
  this->setLastPointProcessed(src.getLastPointProcessed());
  this->setTime(src.getTime());
  this->setIsMoving(src.getIsMoving());
}

bool MotionFeedbackComau::operator==(MotionFeedbackComau &rhs)
{
  if(MAX_NUM_ARMS==2)
  {
    return (this->arm_number_ == rhs.arm_number_ && this->base_frame_.at(0) == rhs.base_frame_.at(0) && this->base_frame_.at(1) == rhs.base_frame_.at(1)
      && this->user_frame_.at(0) == rhs.user_frame_.at(0) && this->user_frame_.at(1) == rhs.user_frame_.at(1) && this->tool_frame_.at(0) == rhs.tool_frame_.at(0) 
      && this->tool_frame_.at(1) == rhs.tool_frame_.at(1) && this->joint_positions_.at(0) == rhs.joint_positions_.at(0)
      && this->joint_positions_.at(1) == rhs.joint_positions_.at(1) && this->joint_motor_currents_.at(0) == rhs.joint_motor_currents_.at(0)
      && this->joint_motor_currents_.at(1) == rhs.joint_motor_currents_.at(1) && this->cartesian_positions_.at(0) == rhs.cartesian_positions_.at(0)
      && this->cartesian_positions_.at(1) == rhs.cartesian_positions_.at(1) && this->config_flags_ == rhs.config_flags_ 
      && this->last_point_processed_ == rhs.last_point_processed_ && this->time_ == rhs.time_ && this->is_moving_ == rhs.is_moving_);
  }
  else
  {
    return (this->arm_number_ == rhs.arm_number_ && this->base_frame_.at(0) == rhs.base_frame_.at(0) && this->user_frame_.at(0) == rhs.user_frame_.at(0) 
      && this->tool_frame_.at(0) == rhs.tool_frame_.at(0) && this->joint_positions_.at(0) == rhs.joint_positions_.at(0)
      && this->joint_motor_currents_.at(0) == rhs.joint_motor_currents_.at(0) && this->cartesian_positions_.at(0) == rhs.cartesian_positions_.at(0) 
      && this->config_flags_ == rhs.config_flags_ && this->last_point_processed_ == rhs.last_point_processed_ 
      && this->time_ == rhs.time_ && this->is_moving_ == rhs.is_moving_);
  }
}

bool MotionFeedbackComau::load(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = false;
  shared_int bytes_length = this->config_flags_.size();

  LOG_COMM("Executing motion feedback load");

  if(MAX_NUM_ARMS==2)
  {
    if (buffer->load(this->arm_number_) && buffer->load(this->base_frame_.at(0)) && buffer->load(this->base_frame_.at(1)) && buffer->load(this->user_frame_.at(0))
        && buffer->load(this->user_frame_.at(1)) && buffer->load(this->tool_frame_.at(0)) && buffer->load(this->tool_frame_.at(1)) 
        && buffer->load(this->joint_positions_.at(0)) && buffer->load(this->joint_positions_.at(1)) && buffer->load(this->joint_motor_currents_.at(0)) 
        && buffer->load(this->joint_motor_currents_.at(1)) && buffer->load(this->cartesian_positions_.at(0)) && buffer->load(this->cartesian_positions_.at(1)) 
        /*&& buffer->load(this->config_flags_,bytes_length)*/ && buffer->load(this->last_point_processed_) 
        && buffer->load(this->time_) && buffer->load(this->is_moving_))
    {

      LOG_COMM("Motion feedback successfully loaded");
      rtn = true;
    }
    else
    {
      LOG_COMM("Motion feedback not loaded");
      rtn = false;
    }
  }
  else
  {
    if (buffer->load(this->arm_number_) && buffer->load(this->base_frame_.at(0)) && buffer->load(this->user_frame_.at(0)) && buffer->load(this->tool_frame_.at(0)) 
        && buffer->load(this->joint_positions_.at(0)) && buffer->load(this->joint_motor_currents_.at(0)) && buffer->load(this->cartesian_positions_.at(0)) 
        /*&& buffer->load(this->config_flags_,bytes_length)*/ && buffer->load(this->last_point_processed_) && buffer->load(this->time_) 
        && buffer->load(this->is_moving_))
    {

      LOG_COMM("Motion feedback successfully loaded");
      rtn = true;
    }
    else
    {
      LOG_COMM("Motion feedback not loaded");
      rtn = false;
    }
  }

  return rtn;
}

bool MotionFeedbackComau::unload(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = false;
  this->data_size_ = buffer->getBufferSize();
/*
  shared_int bytes_length = this->data_size_ - (2*sizeof(industrial::shared_types::shared_int) + 4*MAX_NUM_ARMS*this->base_frame_.at(0).byteLength() +
                                                MAX_NUM_ARMS*this->joint_positions_.at(0).byteLength());
  if(bytes_length <= 0)
  {
    ROS_ERROR("Invalid size for configuration flags in motion feedback unload!");
    return false;
  }
*/           // da reinserire quando sarannno presenti tutti i dati
  LOG_COMM("Executing motion feedback unload");
  
  if(MAX_NUM_ARMS==2)
  {
    if (buffer->unload(this->is_moving_) && buffer->unload(this->time_) && buffer->unload(this->last_point_processed_) /*&& buffer->unload(this->config_flags_,bytes_length)*/ 
        && buffer->unload(this->cartesian_positions_.at(1)) && buffer->unload(this->cartesian_positions_.at(0)) && buffer->unload(this->joint_motor_currents_.at(1)) 
        && buffer->unload(this->joint_motor_currents_.at(0)) && buffer->unload(this->joint_positions_.at(1)) && buffer->unload(this->joint_positions_.at(0)) 
        && buffer->unload(this->tool_frame_.at(1)) && buffer->unload(this->tool_frame_.at(0)) && buffer->unload(this->user_frame_.at(1)) 
        && buffer->unload(this->user_frame_.at(0)) && buffer->unload(this->base_frame_.at(1)) && buffer->unload(this->base_frame_.at(0)) 
        && buffer->unload(this->arm_number_))
    {

      rtn = true;
      LOG_COMM("Motion feedback successfully unloaded");
    }

    else
    {
      LOG_ERROR("Failed to unload motion feedback");
      rtn = false;
    }
  }
  else
  {
    if (buffer->unload(this->is_moving_) && buffer->unload(this->time_) && buffer->unload(this->last_point_processed_) /*&& buffer->unload(this->config_flags_,bytes_length)*/ 
        && buffer->unload(this->cartesian_positions_.at(0)) && buffer->unload(this->joint_motor_currents_.at(0)) && buffer->unload(this->joint_positions_.at(0))
        && buffer->unload(this->tool_frame_.at(0)) && buffer->unload(this->user_frame_.at(0)) && buffer->unload(this->base_frame_.at(0)) 
        && buffer->unload(this->arm_number_))
    {

      rtn = true;
      LOG_COMM("Motion feedback successfully unloaded");
    }

    else
    {
      LOG_ERROR("Failed to unload motion feedback");
      rtn = false;
    }
  }

  return rtn;
}

} // motion_feedback_comau
} // comau

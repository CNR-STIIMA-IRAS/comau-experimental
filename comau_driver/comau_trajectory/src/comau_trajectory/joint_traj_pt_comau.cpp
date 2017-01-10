
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
#include <simple_message/shared_types.h>
#include <simple_message/log_wrapper.h>

#include <comau_driver/comau_trajectory/joint_traj_pt_comau.h>

using namespace industrial::joint_data;
using namespace industrial::shared_types;

namespace comau
{
namespace joint_traj_pt_comau
{

JointTrajPtComau::JointTrajPtComau(void) : joint_position_(MAX_NUM_ARMS), velocity_(MAX_NUM_ARMS)
{
  this->init();
}

JointTrajPtComau::~JointTrajPtComau(void)
{
}

void JointTrajPtComau::init()
{
  for(int i=0;i<MAX_NUM_ARMS;i++)
    this->joint_position_.at(i).init();
  for(int i=0;i<MAX_NUM_ARMS;i++)
    this->velocity_.at(i) = 0.0;
  this->sequence_ = 0;
  this->arm_number_ = 0;
  this->queue_status_ = 0;
  this->fly_tolerance_ = 0;
  this->digital_output_ = 0;
}

void JointTrajPtComau::init(shared_int arm_number, shared_int queue_status, shared_int sequence, shared_real fly_tolerance,
                            std::vector<shared_real> & linear_velocity, std::vector<JointData> & position, shared_int digital_output)
{
  ROS_ASSERT(position.size() == MAX_NUM_ARMS);
  ROS_ASSERT(linear_velocity.size() == MAX_NUM_ARMS);
  this->init();
  this->setArmNumber(arm_number);
  this->setQueueStatus(queue_status);
  this->setSequence(sequence);
  this->setFlyTolerance(fly_tolerance);
  this->setVelocity(linear_velocity);
  this->setJointPosition(position);
  this->setDigitalOutputs(digital_output);
}

void JointTrajPtComau::copyFrom(JointTrajPtComau &src)
{
  this->setArmNumber(src.getArmNumber());
  this->setQueueStatus(src.getQueueStatus());
  this->setSequence(src.getSequence());
  this->setFlyTolerance(src.getFlyTolerance());
  src.getVelocity(this->velocity_);
  src.getJointPosition(this->joint_position_);
  this->setDigitalOutputs(src.getDigitalOutputs());
}

bool JointTrajPtComau::operator==(JointTrajPtComau &rhs)
{
  bool rtn = true;
  
  for(int i=0;i<MAX_NUM_ARMS;i++)
    rtn = rtn && (this->joint_position_.at(i) == rhs.joint_position_.at(i));
  for(int i=0;i<MAX_NUM_ARMS;i++)
    rtn = rtn && (this->velocity_.at(i) == rhs.velocity_.at(i));
  rtn = rtn && this->sequence_ == rhs.sequence_ && this->fly_tolerance_ == rhs.fly_tolerance_ 
	&& this->arm_number_ == rhs.arm_number_ && this->queue_status_ == rhs.queue_status_ 
	&& this->digital_output_ == rhs.digital_output_;
        
  return rtn;
}

bool JointTrajPtComau::load(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = true;

  LOG_COMM("Executing joint trajectory point load");

  if (buffer->load(this->arm_number_))
  {
    if (buffer->load(this->queue_status_))
    {
      if (buffer->load(this->sequence_))
      {
	if (buffer->load(this->fly_tolerance_))
	{
	  for(int i=0;i<MAX_NUM_ARMS;i++)
	  {
	    if (!buffer->load(this->velocity_.at(i)))
	    {
	      rtn = false;
	      LOG_ERROR("Failed to load joint traj. pt. linear velocity");
	    }
	  }
	  for(int i=0;i<MAX_NUM_ARMS;i++)
	  {
	    if (!this->joint_position_.at(i).load(buffer))
	    {
	      rtn = false;
	      LOG_ERROR("Failed to load joint traj. pt. position data");
	    }
	  }
	  if (rtn && buffer->load(this->digital_output_))
	  {
	    LOG_COMM("Trajectory point successfully loaded");
	    rtn = true;
	  }
	  else
	  {
	    if(rtn)
	    {
	      rtn = false;
	      LOG_ERROR("Failed to load joint traj. pt. digital outputs");
	    }
	  }
	}
	else
	{
	  rtn = false;
	  LOG_ERROR("Failed to load joint traj. pt. fly tolerance");
	}
      }
      else
      {
        rtn = false;
        LOG_ERROR("Failed to load joint traj. pt. sequence number");
      }
    }
    else
    {
      rtn = false;
      LOG_ERROR("Failed to load joint traj. pt. queue status");
    }
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to load joint traj. pt. arm number");
  }

  return rtn;
}

bool JointTrajPtComau::unload(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = true;

  LOG_COMM("Executing joint traj. pt. unload");
  
  if (buffer->unload(this->digital_output_))
  {
    for(int i=MAX_NUM_ARMS-1;i>=0;i--)
    {
      if (!this->joint_position_.at(i).unload(buffer))
      {
        rtn = false;
        LOG_ERROR("Failed to unload joint traj. pt. position data");
      }
    }
    for(int i=MAX_NUM_ARMS-1;i>=0;i--)
    {
      if (!buffer->unload(this->velocity_.at(i)))
      {
        rtn = false;
        LOG_ERROR("Failed to load joint traj. pt. linear velocity");
      }
    }
    if (rtn && buffer->unload(this->fly_tolerance_))
    {
      if (rtn && buffer->unload(this->sequence_))
      {
	if (buffer->unload(this->queue_status_))
	{
	  if (buffer->unload(this->arm_number_))
	  {
	    rtn = true;
	    LOG_COMM("Joint traj. pt successfully unloaded");
	  }
	  else
	  {
	    LOG_ERROR("Failed to unload joint traj. pt. arm number");
	    rtn = false;
	  }
	}
	else
	{
	  LOG_ERROR("Failed to unload joint traj. pt. queue status");
	  rtn = false;
	}
      }
      else
      {
	LOG_ERROR("Failed to unload joint traj. pt. sequence number");
	rtn = false;
      }
    }
    else
    {
      if (rtn)
      {
	LOG_ERROR("Failed to unload joint traj. pt. fly tolerance");
	rtn = false;
      }
    }
  }
  else
  {
    LOG_ERROR("Failed to unload joint traj. pt. digital output");
    rtn = false;
  }  
  
  return rtn;
}

} // joint_traj_pt_comau
} // comau


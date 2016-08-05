
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

#include <simple_message/byte_array.h>
#include <simple_message/log_wrapper.h>

#include <comau_driver/comau_trajectory/joint_traj_pt_comau_message.h>
#include <comau_msgs/comau_simple_message.h>

using namespace industrial::shared_types;
using namespace industrial::byte_array;
using namespace industrial::simple_message;
using namespace comau::joint_traj_pt_comau;
using namespace comau::joint_traj_pt_comau_message;

namespace comau
{
namespace joint_traj_pt_comau_message
{

JointTrajPtComauMessage::JointTrajPtComauMessage(void) : point_()
{
  this->init();
}

JointTrajPtComauMessage::~JointTrajPtComauMessage(void)
{
}

bool JointTrajPtComauMessage::init(industrial::simple_message::SimpleMessage & msg)
{
  bool rtn = false;
  ByteArray data = msg.getData();
  this->init();

  if (data.unload(this->point_))
  {
    rtn = true;
  }
  else
  {
    LOG_ERROR("Failed to unload joint traj pt data");
  }
  return rtn;
}

void JointTrajPtComauMessage::init(comau::joint_traj_pt_comau::JointTrajPtComau & point)
{
  this->init();
  this->point_.copyFrom(point);
}

void JointTrajPtComauMessage::init(int message_type)
{
  this->setMessageType(message_type);
  this->point_.init();
}

void JointTrajPtComauMessage::init()
{
  this->setMessageType(comau::simple_message::ComauMsgTypes::JNT_TRJ_PT_COMAU);
  this->point_.init();
}

bool JointTrajPtComauMessage::load(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing joint traj. pt. message load");
  if (buffer->load(this->point_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to load joint traj. pt data");
  }
  return rtn;
}

bool JointTrajPtComauMessage::unload(ByteArray *buffer)
{
  bool rtn = false;
  LOG_COMM("Executing joint traj pt message unload");

  if (buffer->unload(this->point_))
  {
    rtn = true;
  }
  else
  {
    rtn = false;
    LOG_ERROR("Failed to unload joint traj pt data");
  }
  return rtn;
}

} // joint_traj_pt_comau_message
} // comau


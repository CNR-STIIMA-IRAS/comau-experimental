/*********************************************************************
 *
 * Provides the Class encapsulated Comau joint trajectory point data, allowing a dual-arm configuration. 
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

#ifndef JOINT_TRAJ_PT_COMAU_H
#define JOINT_TRAJ_PT_COMAU_H

#include <vector>
#include <simple_message/joint_data.h>
#include <simple_message/simple_message.h>
#include <simple_message/simple_serialize.h>
#include <simple_message/shared_types.h>

namespace comau
{
  /** @ns joint_traj_pt_comau */
namespace joint_traj_pt_comau
{

/**
 * \brief Class encapsulated Comau joint trajectory point data, allowing a dual-arm configuration.  
 * The point data serves as a waypoint along a trajectory and is meant to mirror the
 * JointTrajPointComau message.
 *
 * This point differs from the ROS trajectory point in the following ways:
 *
 *  - The arm number to be considered
 *  - The status of the trajectory queue on the robot controller (meaningful as response value)
 *  - The sequence number of the current point in the trajectory
 *  - The cartesian linear velocity of the TCP
 *  - The joints values for both robots (all of them could be considered or not, depending on the arm_number  value)
 *  - The integer representation of a set of digital outputs of the robot controller
 *
 * The byte representation of a joint trajectory point is as follow (in order lowest index
 * to highest). The standard sizes are given, but can change based on type sizes:
 *
 *   member:             type                                      size
 *   arm_number          (industrial::shared_types::shared_int)    4  bytes
 *   queue_status        (industrial::shared_types::shared_int)    4  bytes
 *   sequence_number     (industrial::shared_types::shared_int)    4  bytes
 *   linear_velocity     (industrial::shared_types::shared_real)   8  bytes
 *   joint_positions     (industrial::shared_types::shared_real)   80 bytes
 *   digital_output      (industrial::shared_types::shared_int)    4  bytes
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class JointTrajPtComau : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  JointTrajPtComau(void);
  /**
   * \brief Destructor
   *
   */
  ~JointTrajPtComau(void);

  /**
   * \brief Initializes a empty joint trajectory point
   *
   */
  void init();

  /**
   * \brief Initializes a complete trajectory point
   *
   */
  void init(industrial::shared_types::shared_int arm_number, industrial::shared_types::shared_int queue_status, 
            industrial::shared_types::shared_int sequence, std::vector<industrial::shared_types::shared_real> & linear_velocity,
            std::vector<industrial::joint_data::JointData> & position, industrial::shared_types::shared_int digital_output);

  /**
   * \brief Sets joint position data
   *
   * \param position position data
   */
  void setJointPosition(std::vector<industrial::joint_data::JointData> &position)
  {
    for(int i=0;i<MAX_NUM_ARMS;i++)
      this->joint_position_.at(i).copyFrom(position.at(i));
  }

  /**
   * \brief Returns a copy of the position data
   *
   * \param dest position dest
   */
  void getJointPosition(std::vector<industrial::joint_data::JointData> &dest)
  {
    for(int i=0;i<MAX_NUM_ARMS;i++)
      dest.at(i).copyFrom(this->joint_position_.at(i));
  }

  /**
   * \brief Sets joint trajectory point sequence number
   *
   * \param sequence value
   */
  void setSequence(industrial::shared_types::shared_int sequence)
  {
    this->sequence_ = sequence;
  }

  /**
   * \brief Returns joint trajectory point sequence number
   *
   * \return joint trajectory sequence number
   */
  industrial::shared_types::shared_int getSequence()
  {
    return this->sequence_;
  }
  
  /**
   * \brief Sets joint trajectory point arm number
   *
   * \param arm_number value
   */
  void setArmNumber(industrial::shared_types::shared_int arm_number)
  {
    this->arm_number_ = arm_number;
  }

  /**
   * \brief Returns joint trajectory point arm number
   *
   * \return joint trajectory arm number
   */
  industrial::shared_types::shared_int getArmNumber()
  {
    return this->arm_number_;
  }
  
  /**
   * \brief Sets joint trajectory point queue status
   *
   * \param queue_status value
   */
  void setQueueStatus(industrial::shared_types::shared_int queue_status)
  {
    this->queue_status_ = queue_status;
  }

  /**
   * \brief Returns joint trajectory point queue status
   *
   * \return joint trajectory queue status
   */
  industrial::shared_types::shared_int getQueueStatus()
  {
    return this->queue_status_;
  }

  /**
   * \brief Sets joint trajectory point linear velocity
   *
   * \param velocity value
   */
  void setVelocity(std::vector<industrial::shared_types::shared_real> &velocity)
  {
    for(int i=0;i<MAX_NUM_ARMS;i++)
      this->velocity_.at(i) = velocity.at(i);
  }

  /**
   * \brief Returns joint trajectory point linear velocity
   *
   * \return joint trajectory linear velocity
   */
  void getVelocity(std::vector<industrial::shared_types::shared_real> &velocity)
  {
    for(int i=0;i<MAX_NUM_ARMS;i++)
      velocity.at(i) = this->velocity_.at(i);
  }
  
  /**
   * \brief Sets joint trajectory point digital outputs
   *
   * \param digital_out value
   */
  void setDigitalOutputs(industrial::shared_types::shared_int digital_out)
  {
    this->digital_output_ = digital_out;
  }

  /**
   * \brief Returns joint trajectory point digital outputs
   *
   * \return joint trajectory digital outputs
   */
  industrial::shared_types::shared_int getDigitalOutputs()
  {
    return this->digital_output_;
  }

  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(JointTrajPtComau &src);

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(JointTrajPtComau &rhs);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    return 4*sizeof(industrial::shared_types::shared_int) + MAX_NUM_ARMS*(industrial::shared_types::shared_real)
        + MAX_NUM_ARMS*this->joint_position_.at(0).byteLength();
  }

private:
  
  /**
   * \brief maximum number of robot arms that can be held in the message.
   */
  #ifdef DUAL_ARM
    static const industrial::shared_types::shared_int MAX_NUM_ARMS = 2;
  #else
    static const industrial::shared_types::shared_int MAX_NUM_ARMS = 1;
  #endif
  
  /**
   * \brief active arm number
   */
  industrial::shared_types::shared_int arm_number_;
  /**
   * \brief status of the queue of the trajectory on the robot controller
   */
  industrial::shared_types::shared_int queue_status_;
  /**
   * \brief trajectory sequence number
   */
  industrial::shared_types::shared_int sequence_;
  /**
   * \brief tcp linear velocity for each arm
   */
  std::vector<industrial::shared_types::shared_real> velocity_;
  /**
   * \brief joint point positional data
   */
  std::vector<industrial::joint_data::JointData> joint_position_;
  /**
   * \brief int representation of a binary array; digital outputs of the controller
   */
  industrial::shared_types::shared_int digital_output_;

};

} // joint_traj_pt_comau
} // comau

#endif /* JOINT_TRAJ_PT_COMAU_H */

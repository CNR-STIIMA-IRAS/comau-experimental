
/*********************************************************************
 *
 * Provides the Structures to manage the MotionFeedbackComau.msg
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

#ifndef MOTION_FEEDBACK_COMAU_H
#define MOTION_FEEDBACK_COMAU_H

#include <vector>
#include <string>

#include <simple_message/simple_serialize.h>
#include <simple_message/shared_types.h>
#include <simple_message/joint_data.h>

#include <comau_msgs/comau_simple_message.h>
#include <comau_driver/comau_motion_feedback/cartesian_data.h>

namespace comau
{
  
  /** @ns motion_feedback_comau */
namespace motion_feedback_comau
{

/**
 * \brief Class encapsulated motion feedback data.  The motion feedback data is
 * meant to mirror the comau_msgs/MotionFeedbackComau message.
 *
 *
 * The byte representation of a motion feedback is as follows (in order lowest index
 * to highest). The standard sizes are given, but can change based on type sizes:
 *
 *   member:               type                                      size
 *   arm_number            (industrial::shared_types::shared_int)    4  bytes
 *   base_frame            (industrial::shared_types::shared_real)   48 bytes
 *   user_frame            (industrial::shared_types::shared_real)   48 bytes
 *   tool_frame            (industrial::shared_types::shared_real)   48 bytes
 *   joint_positions       (industrial::shared_types::shared_real)   80 bytes
 *   joint_motor_currents  (industrial::shared_types::shared_real)   80 bytes
 *   cartesian_positions   (industrial::shared_types::shared_real)   48 bytes
 *   last_point_processed  (industrial::shared_types::shared_int)    4  bytes
 *   cntrl_time            (industrial::shared_types::shared_int)    4  bytes
 *   is_moving             (industrial::shared_types::shared_int)    4  bytes
 *
 * 
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class MotionFeedbackComau : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
  * \brief Default constructor
  *
  * This method creates empty data.
  *
  */
  MotionFeedbackComau(void);
  /**
  * \brief Destructor
  *
  */
  ~MotionFeedbackComau(void);

  /**
  * \brief Initializes an empty motion feedback
  *
  */
  void init();

  /**
   * \brief Returns motion arm number
   *
   * \return motion arm number
   */
  industrial::shared_types::shared_int getArmNumber()
  {
    return arm_number_;
  }

  /**
   * \brief Returns motion base frame
   *
   * \return motion base frame
   */
  std::vector<comau::cartesian_data::CartesianData> getBaseFrame()
  {
    return base_frame_;
  }

  /**
   * \brief Returns motion user frame
   *
   * \return motion user frame
   */
  std::vector<comau::cartesian_data::CartesianData> getUserFrame()
  {
    return user_frame_;
  }

  /**
   * \brief Returns motion tool frame
   *
   * \return motion tool frame
   */
  std::vector<comau::cartesian_data::CartesianData> getToolFrame()
  {
    return tool_frame_;
  }

  /**
   * \brief Returns motion joint positions
   *
   * \return motion joint positions
   */
  std::vector<industrial::joint_data::JointData> getJointPos()
  {
    return joint_positions_;
  }
  
  /**
   * \brief Returns the current absorbed by each axis in ampere
   *
   * \return motor currents in ampere
   */
  std::vector<industrial::joint_data::JointData> getJointMtrCurr()
  {
    return joint_motor_currents_;
  }

  /**
   * \brief Returns motion cartesian positions
   *
   * \return motion cartesian positions
   */
  std::vector<comau::cartesian_data::CartesianData> getCartesianPos()
  {
    return cartesian_positions_;
  }

  /**
   * \brief Returns motion cartesian position configuration flags
   *
   * \return motion cartesian position configuration flags
   */
  std::string getConfigFlags()
  {
    return config_flags_;
  }

  /**
   * \brief Returns the last point processed during motion
   *
   * \return last point processed during motion
   */
  industrial::shared_types::shared_int getLastPointProcessed()
  {
    return last_point_processed_;
  }

  /**
   * \brief Returns controller motion time (the time definition is set in the controller code)
   *
   * \return controller motion time
   */
  industrial::shared_types::shared_int getTime()
  {
    return time_;
  }
  
  /**
   * \brief Returns if the robot is moving or not
   *
   * \return moving state of the robot
   */
  industrial::shared_types::shared_int getIsMoving()
  {
    return is_moving_;
  }


  /**
   * \brief Sets motion arm number
   *
   * \param arm_num arm number value
   */
  void setArmNumber(industrial::shared_types::shared_int arm_num)
  {
    this->arm_number_ = arm_num;
  }

  /**
   * \brief Sets motion base frame
   *
   * \param base base frame value
   */
  void setBaseFrame(std::vector<comau::cartesian_data::CartesianData> base)
  {
    this->base_frame_ = base;
  }

  /**
   * \brief Sets motion user frame
   *
   * \param user user frame value
   */
  void setUserFrame(std::vector<comau::cartesian_data::CartesianData> user)
  {
    this->user_frame_ = user;
  }

  /**
   * \brief Sets motion tool frame
   *
   * \param tool tool frame value
   */
  void setToolFrame(std::vector<comau::cartesian_data::CartesianData> tool)
  {
    this->tool_frame_ = tool;
  }

  /**
   * \brief Sets motion joint positions
   *
   * \param jnt_pos joint positions value
   */
  void setJointPos(std::vector<industrial::joint_data::JointData> jnt_pos)
  {
    this->joint_positions_ = jnt_pos;
  }
  
  /**
   * \brief Sets motor currents
   *
   * \param jnt_mtr_curr motor currents value
   */
  void setJointMtrCurr(std::vector<industrial::joint_data::JointData> jnt_mtr_curr)
  {
    this->joint_motor_currents_ = jnt_mtr_curr;
  }

  /**
   * \brief Sets motion cartesian positions
   *
   * \param cart_pos cartesian positions value
   */
  void setCartesianPos(std::vector<comau::cartesian_data::CartesianData> cart_pos)
  {
    this->cartesian_positions_ = cart_pos;
  }

  /**
   * \brief Sets motion cartesian positions configuration flags
   *
   * \param cnfg cartesian positions configuration flags value
   */
  void setConfigFlags(std::string cnfg)
  {
    this->config_flags_ = cnfg;
  }

  /**
   * \brief Sets motion last point processed
   *
   * \param last_point last point processed value
   */
  void setLastPointProcessed(industrial::shared_types::shared_int last_point)
  {
    this->last_point_processed_ = last_point;
  }

  /**
   * \brief Sets motion controller time
   *
   * \param time controller time value
   */
  void setTime(industrial::shared_types::shared_int time)
  {
    this->time_ = time;
  }
  
  /**
   * \brief 
   *
   * \param 
   */
  void setIsMoving(industrial::shared_types::shared_int is_moving)
  {
    this->is_moving_ = is_moving;
  }


  /**
  * \brief Copies the passed in value
  *
  * \param src (value to copy)
  */
  void copyFrom(MotionFeedbackComau &src);

  /**
  * \brief == operator implementation
  *
  * \return true if equal
  */
  bool operator==(MotionFeedbackComau &rhs);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    if(data_size_>0)
      return data_size_;
    else 
      return 0;
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

  industrial::shared_types::shared_int data_size_;

  /**
  * \brief Arm number (see ComauArmNumbers::ComauArmNumber)
  */
  industrial::shared_types::shared_int arm_number_;

  /**
  * \brief Base frame of both arms
  */
  std::vector<comau::cartesian_data::CartesianData> base_frame_;

  /**
  * \brief User frame of both arms
  */
  std::vector<comau::cartesian_data::CartesianData> user_frame_;

  /**
  * \brief Tool frame of both arms
  */
  std::vector<comau::cartesian_data::CartesianData> tool_frame_;

  /**
  * \brief Joint positions of both arms
  */
  std::vector<industrial::joint_data::JointData> joint_positions_;
  
  /**
  * \brief Joint motor currents of both arms
  */
  std::vector<industrial::joint_data::JointData> joint_motor_currents_;

  /**
  * \brief Cartesian position of both arms
  */
  std::vector<comau::cartesian_data::CartesianData> cartesian_positions_;


  /**
   * \brief Cartesian position configuration flags of both arms
   */
  std::string config_flags_;

  /**
  * \brief The last point that has been processed by the robot (the same for both arms)
  */
  industrial::shared_types::shared_int last_point_processed_;

  /**
  * \brief Controller time indicator
  */
  industrial::shared_types::shared_int time_;
  
  /**
  * \brief Robot is moving or not
  */
  industrial::shared_types::shared_int is_moving_;

};

} // motion_feedback_comau
} // comau

#endif /* MOTION_FEEDBACK_COMAU_H */

/*********************************************************************
 *
 * Provides the specifications of the free-fileds of the SImpleMessage for the COMAU robot
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
 *   * Neither the name of the National Research Council of Italy nor the 
 *     from this software without specific prior written permission.
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

#ifndef COMAU_SIMPLE_MESSAGE_H
#define COMAU_SIMPLE_MESSAGE_H


#include <simple_message/simple_message.h>

namespace comau
{
  
/**
 * \brief Enumeration of Comau socket ports.
 */
namespace ComauControllerPorts
{
enum ComauControllerPort
{
  TRJ_MOTION = 1212, MOTION_FEEDBACK = 1213
};
}
typedef ComauControllerPorts::ComauControllerPort ComauControllerPort;
  

namespace simple_message
{
/**
 * \brief Enumeration of comau-specific message types.
 *        See simple_message.h for a listing of "standard" message types
 */
namespace ComauMsgTypes
{
typedef enum ComauMsgType 
{
  INVALID = 0,
  PING = 1,
  JNT_TRJ_PT_COMAU = 3001,
  TRJ_FINISHED = 3002,
  ROBOT_STATUS = 3003,
  MOTION_FEEDBACK = 3004,
  START_MOTION = 3010,
  STOP_MOTION = 3011,
  CANCEL_MOTION = 3012,
  SHUTDOWN = 3020
} ComauMsgType;
}

/**
 * \brief Enumeration of arm numbers allowed (sigle arm or dual arm).
 */
namespace ComauArmNumbers
{
typedef enum ComauArmNumber 
{
  INVALID = 0,
  ARM_1 = 1,
  ARM_2 = 2,
  ARM_1_2 = 12
} ComauArmNumber;
}

/**
 * \brief Enumeration of queue state indicators.
 */
namespace ComauQueueStatus
{
typedef enum ComauQueueStatus 
{
  INVALID = 0,
  READY = 1,
  BUSY = 2,
  ERROR = 3
} ComauQueueStatus;
}


}  // namespace simple_message
}  // namespace comau

#endif  /* COMAU_SIMPLE_MESSAGE_H */

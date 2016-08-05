
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

#include <comau_driver/comau_motion_feedback/cartesian_data.h>

using namespace industrial::shared_types;

namespace comau
{
namespace cartesian_data
{

CartesianData::CartesianData(void)
{
  this->init();
}

CartesianData::~CartesianData(void)
{

}

void CartesianData::init()
{
  for (int i = 0; i < this->getMaxNumCoords(); i++)
  {
    this->setCoord(i, 0.0);
  }
}

bool CartesianData::setCoord(shared_int index, shared_real value)
{
  bool rtn = false;

  if (index < this->getMaxNumCoords())
  {
    this->cart_[index] = value;
    rtn = true;
  }
  else
  {
    LOG_ERROR("Coordinate index: %d is out of range.", index);
    rtn = false;
  }
  return rtn;
}

bool CartesianData::getCoord(shared_int index, shared_real & value) const
{
  bool rtn = false;

  if (index < this->getMaxNumCoords())
  {
    value = this->cart_[index];
    rtn = true;
  }
  else
  {
    LOG_ERROR("Coordinate index: %d is out of range.", index);
    rtn = false;
  }
  return rtn;
}

shared_real CartesianData::getCoord(shared_int index) const
{
  shared_real rtn = 0.0;
  this->getCoord(index, rtn);
  return rtn;
}
  
void CartesianData::copyFrom(CartesianData &src)
{
  shared_real value = 0.0;

  for (int i = 0; i < this->getMaxNumCoords(); i++)
  {
    src.getCoord(i, value);
    this->setCoord(i, value);
  }
}

bool CartesianData::operator==(CartesianData &rhs)
{
  bool rtn = true;

  shared_real lhsvalue, rhsvalue;

  for (int i = 0; i < this->getMaxNumCoords(); i++)
  {
    this->getCoord(i, lhsvalue);
    rhs.getCoord(i, rhsvalue);
    if (lhsvalue != rhsvalue)
    {
      rtn = false;
      break;
    }
  }
  return rtn;

}

bool CartesianData::load(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = false;
  shared_real value = 0.0;

  LOG_COMM("Executing cartesian position load");
  for (int i = 0; i < this->getMaxNumCoords(); i++)
  {
    this->getCoord(i, value);
    rtn = buffer->load(value);
    if (!rtn)
    {
      LOG_ERROR("Failed to load cartesian position data");
      break;
    }
  }
  return rtn;
}

bool CartesianData::unload(industrial::byte_array::ByteArray *buffer)
{
  bool rtn = false;
  shared_real value = 0.0;

  LOG_COMM("Executing cartesian position unload");
  for (int i = (this->getMaxNumCoords()-1); i >= 0; i--)
  {
    rtn = buffer->unload(value);
    if (!rtn)
    {
      LOG_ERROR("Failed to unload message coordinate: %d from data[%d]", i, buffer->getBufferSize());
      break;
    }
    this->setCoord(i, value);
  }
  return rtn;
}

} // cartesian_data
} // comau


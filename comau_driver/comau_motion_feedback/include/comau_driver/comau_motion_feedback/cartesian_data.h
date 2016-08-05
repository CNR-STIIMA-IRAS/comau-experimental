
/*********************************************************************
 *
 * Provides the CartesianData Structure
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

#ifndef CARTESIAN_DATA_H
#define CARTESIAN_DATA_H


#include <comau_msgs/comau_simple_message.h>
#include <simple_message/simple_serialize.h>
#include <simple_message/shared_types.h>


namespace comau
{
  /** @ns cartesian_data */
namespace cartesian_data
{

/**
 * \brief Class encapsulated cartesian data (positions).
 *
 * For simplicity and cross platform compliance, this is implemented as a
 * fixed size array.
 *
 * The byte representation of a cartesian data is as follows. The standard sizes
 * are given, but can change based on type sizes:
 *
 *   member:             type                                      size
 *   cart                (industrial::shared_types::shared_real)   4 * MAX_NUM_COORDS
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class CartesianData : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  CartesianData(void);
  /**
   * \brief Destructor
   *
   */
  ~CartesianData(void);

  /**
   * \brief Initializes a empty cartesian data
   *
   */
  void init();

  /**
   * \brief Sets a coordinate value within the buffer
   *
   * \param coordinate index
   * \param coordinate value
   *
   * \return true if value set, otherwise false (index greater than max)
   */
  bool setCoord(industrial::shared_types::shared_int index, industrial::shared_types::shared_real value);

  /**
   * \brief Gets a coordinate value within the buffer
   *
   * \param coordinate index
   * \param coordinate value (passed by reference)
   *
   * \return true if value valid, otherwise false (index greater than max)
   */
  bool getCoord(industrial::shared_types::shared_int index, industrial::shared_types::shared_real & value) const;

  /**
   * \brief Gets a coordinate value within the buffer (Only use this form if you are
   * sure the index is within bounds).
   *
   * \param coordinate index
   *
   * \return coordinate value (returns 0.0 if index is out of bounds)
   */
  industrial::shared_types::shared_real getCoord(industrial::shared_types::shared_int index) const;

  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(CartesianData &src);
  
  /**
   * \brief returns the maximum number of coordinates the message holds
   *
   * \return max number of coordinates
   */
  int getMaxNumCoords() const
  {
    return MAX_NUM_COORDS;
  }

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(CartesianData &rhs);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    return MAX_NUM_COORDS * sizeof(industrial::shared_types::shared_real);
  }

private:
  /**
   * \brief maximum number of coordinates that can be held in the message.
   */
  static const industrial::shared_types::shared_int MAX_NUM_COORDS = 6;

  /**
   * \brief internal data buffer
   */
  industrial::shared_types::shared_real cart_[MAX_NUM_COORDS];

};

} // cartesian_data
} // comau

#endif /* CARTESIAN_DATA_H */

/*********************************************************************
 *
 * Provides the JointTrajectoryInterface for COMAU robots
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


#ifndef COMAU_JOINT_TRAJECTORY_STREAMER_H
#define COMAU_JOINT_TRAJECTORY_STREAMER_H

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/thread.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <industrial_msgs/StartMotion.h>

#include <comau_msgs/JointTrajComau.h>
#include <simple_message/smpl_msg_connection.h>
#include <comau_driver/comau_trajectory/comau_joint_trajectory_interface.h>
#include <comau_driver/comau_trajectory/joint_traj_pt_comau_message.h>

namespace comau
{
  /** @ns joint_trajectory_streamer */
namespace joint_trajectory_streamer
{

using comau::joint_trajectory_interface::JointTrajectoryInterface;
using comau::joint_traj_pt_comau_message::JointTrajPtComauMessage;
using industrial::smpl_msg_connection::SmplMsgConnection;

/**
 * \brief Enumeration of transfer states.
 */
namespace TransferStates
{
enum TransferState
{
  IDLE = 0, STREAMING =1 //,STARTING, //, STOPPING
};
}
typedef TransferStates::TransferState TransferState;

/**
 * \brief Message handler that streams joint trajectories to the robot controller
 */
class ComauJointTrajectoryStreamer : public JointTrajectoryInterface
{

public:

  // since this class defines a different init(), this helps find the base-class init()
  using JointTrajectoryInterface::init;

  /**
   * \brief Default constructor
   *
   * \param min_buffer_size minimum number of points as required by robot implementation
   */
  ComauJointTrajectoryStreamer(int min_buffer_size = 1) : min_buffer_size_(min_buffer_size)
                                                        , force_thread_stop_( false )
                                                        { };
  
  /**
   * \brief Default destructor
   */
  ~ComauJointTrajectoryStreamer();

  /**
   * \brief Class initializer
   *
   * \param connection simple message connection that will be used to send commands to robot (ALREADY INITIALIZED)
   * \param joint_names list of expected joint-names.
   *   - Count and order should match data sent to robot connection.
   *   - Use blank-name to insert a placeholder joint position (typ. 0.0).
   *   - Joints in the incoming JointTrajectory stream that are NOT listed here will be ignored.
   * \param velocity_limits map of maximum velocities for each joint
   *   - leave empty to lookup from URDF
   * \return true on success, false otherwise (an invalid message type)
   */
  virtual bool init(SmplMsgConnection* connection, const std::vector<std::string> &joint_names,
                    const std::map<std::string, double> &velocity_limits = std::map<std::string, double>());

  /**
   * \brief Callback function registered to ROS topic-subscribe.
   *   Transform message into SimpleMessage objects and send commands to robot.
   *
   * \param msg JointTrajComau message from ROS trajectory-planner
   */
  virtual bool jointTrajectoryCB(const comau_msgs::JointTrajComauConstPtr &msg);
  
  /**
   * \brief Convert JointTrajComau trajectory message into stream of JointTrajPtComauMessages for sending to robot.
   *   Also includes various joint transforms that can be overridden for robot-specific behavior.
   *
   * \param[in] traj ROS JointTrajComau message
   * \param[out] msgs list of JointTrajPtComauMessages for sending to robot
   *
   * \return true on success, false otherwise
   */
  virtual bool trajectory_to_msgs(const comau_msgs::JointTrajComauConstPtr &traj, std::vector<JointTrajPtComauMessage>* msgs);
  /**
   * \brief Convert JointTrajComau trajectory message into stream of JointTrajPtComauMessages for sending to robot.
   *   Also includes various joint transforms that can be overridden for robot-specific behavior.
   *
   * \param[in] traj string indicating the namespace in the parameter server where the trajectory is placed
   * \param[out] msgs list of JointTrajPtComauMessages for sending to robot
   *
   * \return true on success, false otherwise
   */
  virtual bool trajectory_to_msgs(const std::string traj, std::vector<JointTrajPtComauMessage>* msgs);
  
  /**
   *  \brief Function to load the trajectory directly from the parmeter server.
   * 
   *  \param trj_name string indicating the namespace in the parameter server where the trajectory is placed
   * 
   *  \return true on success, false otherwise
   */
  bool loadTrjFromParam(const std::string &trj_name); //&trj_name
  
  /**
   *  \brief Spinning function handling different states of the system
   */
  void streamingThread();

  /**
   * \brief Send trajectory to robot, using this node's robot-connection.
   *
   * \param messages List of SimpleMessage JointTrajPtComauMessage to send to robot.
   *
   * \return true on success, false otherwise
   */
  bool send_to_robot(const std::vector<JointTrajPtComauMessage>& messages);
  
  /**
   * \brief Function to pause the streming thread.
   *
   * \param time_sec Optional time (in seconds) before the thread is restored.
   */
  void streamingPause(const double time_sec=0.0);
  
  /**
   * \brief Function to unpause the streming thread.
   */
  void streamingResume();
  
  bool reloadTrjCB( industrial_msgs::StartMotion::Request &req,
                    industrial_msgs::StartMotion::Response &res);
  
  /**
   * \brief Function to get the status on trajectory execution.
  */
  int getTrajectorySts();
  
   /**
   * \brief Function to set/unset the cyclic trajectory execution.
   *
   * \param is_cyclic Flag: true to set the cyclic trajectory, false otherwise.
   */
  void setCyclicTrj(const bool is_cyclic);
  
protected:

  /**
   * \brief Send a finished trajectory command to the robot
   */
  void trajectoryStop();

  boost::thread* streaming_thread_;
  ros::NodeHandle node_;
  ros::ServiceServer srv_reload_trj_;  // handle for start_motion service
  
  int current_point_;
  std::vector<JointTrajPtComauMessage> current_traj_;
  TransferState state_;
  TransferState previous_state_;
  ros::Time streaming_start_;
  int min_buffer_size_;
  
  bool is_busy;
  bool force_thread_stop_;
  
  bool trajectory_dwnl_complete_;
  bool cyclic_traj_;
  
};

} //joint_trajectory_streamer
} //comau

#endif /* COMAU_JOINT_TRAJECTORY_STREAMER_H */

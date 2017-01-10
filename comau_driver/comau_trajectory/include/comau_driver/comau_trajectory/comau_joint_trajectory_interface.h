
/*********************************************************************
 *
 * Provides the Message handler that relays joint trajectories to the robot controller
 * "class JointTrajectoryInterface"
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

#ifndef COMAU_JOINT_TRAJECTORY_INTERFACE_H
#define COMAU_JOINT_TRAJECTORY_INTERFACE_H


#include <map>
#include <vector>
#include <string>
#include <fstream>
#include <boost/thread.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>

#include <ros/ros.h>
#include <industrial_msgs/StartMotion.h>
#include <industrial_msgs/StopMotion.h>
#include <sensor_msgs/JointState.h>

#include <simple_message/smpl_msg_connection.h>
#include <simple_message/socket/tcp_client.h>
#include <simple_message/socket/tcp_socket.h>
#include <simple_message/socket/simple_socket.h>
#include <simple_message/shared_types.h>

#include <comau_driver/comau_trajectory/joint_traj_pt_comau_message.h>
#include <comau_driver/comau_trajectory/joint_traj_pt_comau.h>
#include <comau_msgs/comau_simple_message.h>
#include <comau_msgs/JointTrajComau.h>
#include <comau_msgs/JointTrajPointComau.h>
#include <comau_msgs/CmdJointTrjComau.h>


namespace comau
{
  
   /** @ns joint_trajectory_interface */
namespace joint_trajectory_interface
{

  using industrial::smpl_msg_connection::SmplMsgConnection;
  using industrial::tcp_client::TcpClient;
  using comau::joint_traj_pt_comau_message::JointTrajPtComauMessage;
  using comau::joint_traj_pt_comau::JointTrajPtComau;

/**
 * \brief Message handler that relays joint trajectories to the robot controller
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class JointTrajectoryInterface
{

public:

  /**
    * \brief Default constructor.
    */
  JointTrajectoryInterface() :  default_joint_pos_(0.0), 
                                default_vel_ratio_(0.1), 
                                default_duration_(10.0),
                                named_mtx_(boost::interprocess::open_or_create, "jnt_trj_shared_mtx")
                                { };
  
  /**
  * \brief Default destructor.
  */
  virtual ~JointTrajectoryInterface();

  /**
    * \brief Initialize robot connection using default method.
    *
    * \param default_ip default IP address to use for robot connection [OPTIONAL]
    *                    - this value will be used if ROS param "robot_ip_address" cannot be read
    * \param default_port default port to use for robot connection [OPTIONAL]
    *                    - this value will be used if ROS param "~port" cannot be read
    *
    * \return true on success, false otherwise
    */
  virtual bool init(std::string default_ip = "", int default_port = comau::ComauControllerPorts::ComauControllerPort::TRJ_MOTION);


  /**
    * \brief Initialize robot connection using specified method.
    *
    * \param connection new robot-connection instance (ALREADY INITIALIZED).
    *
    * \return true on success, false otherwise
    */
  virtual bool init(SmplMsgConnection* connection);

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
   * \brief Begin processing messages and publishing topics.
   */
  virtual void run() { ros::spin(); }

protected:

  /**
   * \brief Send a finished trajectory command to the robot
   */
  virtual void trajectoryStop();

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
   * \brief Transform joint positions before publishing.
   * Can be overridden to implement, e.g. robot-specific joint coupling.
   *
   * \param[in] pt_in trajectory-point, in same order as expected for robot-connection.
   * \param[out] pt_out transformed trajectory-point (in same order/count as input positions)
   *
   * \return true on success, false otherwise
   */
  virtual bool transform(const comau_msgs::JointTrajPointComau& pt_in, comau_msgs::JointTrajPointComau* pt_out)
  {
    *pt_out = pt_in;  // by default, no transform is applied
    return true;
  }

  /**
   * \brief Select specific joints for sending to the robot
   *
   * \param[in] ros_joint_names joint names from ROS command
   * \param[in] ros_pt target pos/vel from ROS command
   * \param[in] rbt_joint_names joint names, in order/count expected by robot connection
   * \param[out] rbt_pt target pos/vel, matching rbt_joint_names
   *
   * \return true on success, false otherwise
   */
  virtual bool select(const std::vector<std::string>& ros_joint_names, const comau_msgs::JointTrajPointComau& ros_pt,
                      const std::vector<std::string>& rbt_joint_names, comau_msgs::JointTrajPointComau* rbt_pt);
  
  /**
   * \brief Send trajectory to robot, using this node's robot-connection.
   *   Specific method must be implemented in a derived class (e.g. streaming, download, etc.)
   *
   * \param messages List of SimpleMessage JointTrajPtComauMessage to send to robot.
   *
   * \return true on success, false otherwise
   */
  virtual bool send_to_robot(const std::vector<JointTrajPtComauMessage>& messages)=0;
  
  /**
   * \brief Function to pause the streming thread (it has to be deifined in the streming class).
   *
   * \param time_sec Optional time (in seconds) before the thread is restored.
   */
  virtual void streamingPause(const double time_sec=0.0)=0;
  
  /**
   * \brief Function to unpause the streming thread (it has to be deifined in the streming class).
   */
  virtual void streamingResume()=0;

  /**
   * \brief Callback function registered to ROS topic-subscribe.
   *   Transform message into SimpleMessage objects and send commands to robot.
   *
   * \param msg JointTrajComau message from ROS trajectory-planner
   */
  virtual bool jointTrajectoryCB(const comau_msgs::JointTrajComauConstPtr &msg);
  
  /**
   * \brief Callback function registered to ROS startMotion service
   *   Sends start-motion command to robot.
   *
   * \param req StartMotion request from service call
   * \param res StartMotion response to service call
   * \return true always.  Look at res.code.val to see if call actually succeeded.
   */
  virtual bool startMotionCB( industrial_msgs::StartMotion::Request &req,
                              industrial_msgs::StartMotion::Response &res);

  /**
   * \brief Callback function registered to ROS stopMotion service
   *   Sends stop-motion command to robot.
   *
   * \param req StopMotion request from service call
   * \param res StopMotion response to service call
   * \return true always.  Look at res.code.val to see if call actually succeeded.
   */
  virtual bool stopMotionCB(industrial_msgs::StopMotion::Request &req,
                            industrial_msgs::StopMotion::Response &res);
  
  /**
   * \brief Callback function registered to ROS cancelMotion service
   *   Sends cancel-motion command to robot.
   *
   * \param req StopMotion request from service call
   * \param res StopMotion response to service call
   * \return true always.  Look at res.code.val to see if call actually succeeded.
   */
  virtual bool cancelMotionCB(industrial_msgs::StopMotion::Request &req,
                              industrial_msgs::StopMotion::Response &res);

  /**
   * \brief Validate that trajectory command meets minimum requirements
   *
   * \param traj incoming trajectory
   * \return true if trajectory is valid, false otherwise
   */
  virtual bool is_valid(const comau_msgs::JointTrajComau &traj);
  
  /**
   * \brief Callback for JointState topic
   *
   * \param msg JointState message
   */
  virtual void jointStateCB(const sensor_msgs::JointStateConstPtr &msg);
  
  TcpClient default_tcp_connection_;

  ros::NodeHandle node_;
  SmplMsgConnection* connection_;
  ros::Subscriber sub_cur_pos_;  // handle for joint-state topic subscription
  ros::Subscriber sub_joint_trajectory_; // handle for joint-trajectory topic subscription
  ros::ServiceServer srv_joint_trajectory_;  // handle for joint-trajectory service
  ros::ServiceServer srv_stop_motion_;   // handle for stop_motion service
  ros::ServiceServer srv_start_motion_;  // handle for start_motion service
  ros::ServiceServer srv_cancel_motion_;  // handle for start_motion service
  std::vector<std::string> all_joint_names_;
  double default_joint_pos_;  // default position to use for "dummy joints", if none specified
  double default_vel_ratio_;  // default velocity ratio to use for joint commands, if no velocity or max_vel specified
  double default_duration_;   // default duration to use for joint commands, if no
  std::map<std::string, double> joint_vel_limits_;  // cache of max joint velocities from URDF
  sensor_msgs::JointState cur_joint_pos_;  // cache of last received joint state
  
  boost::interprocess::named_mutex named_mtx_;

  bool cancel_motion_;

private:
  /**
   * \brief maximum number of robot arms that can be held.
   */
  #ifdef DUAL_ARM
    static const industrial::shared_types::shared_int MAX_NUM_ARMS = 2;
  #else
    static const industrial::shared_types::shared_int MAX_NUM_ARMS = 1;
  #endif
  
  static JointTrajPtComauMessage create_message(industrial::shared_types::shared_int arm_number, industrial::shared_types::shared_int seq, 
						industrial::shared_types::shared_real fly_tol, std::vector<industrial::shared_types::shared_real> velocity, 
						std::vector<industrial::shared_types::shared_real> joint_pos, industrial::shared_types::shared_int digital_out);

  /**
   * \brief Callback function registered to ROS CmdJointTrjComau service
   *   Duplicates message-topic functionality, but in service form.
   *
   * \param req CmdJointTrjComau request from service call
   * \param res CmdJointTrjComau response to service call
   * \return true always.  Look at res.code.val to see if call actually succeeded
   */
  bool jointTrajectoryCB(comau_msgs::CmdJointTrjComau::Request &req,
                         comau_msgs::CmdJointTrjComau::Response &res);
};

} //joint_trajectory_interface
} //comau

#endif /* COMAU_JOINT_TRAJECTORY_INTERFACE_H */

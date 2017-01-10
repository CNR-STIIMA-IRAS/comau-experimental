

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

#include <algorithm>
#include <industrial_utils/param_utils.h>

#include <comau_msgs/comau_simple_message.h>
#include <comau_driver/comau_trajectory/comau_joint_trajectory_interface.h>

using namespace industrial::shared_types;
using namespace industrial_utils::param;
using industrial::simple_message::SimpleMessage;
namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;
typedef comau::joint_traj_pt_comau::JointTrajPtComau rbt_JointTrajPtComau;
typedef comau_msgs::JointTrajPointComau ros_JointTrajPtComau;

namespace comau
{
namespace joint_trajectory_interface
{

#define ROS_ERROR_RETURN(rtn,...) do {ROS_ERROR(__VA_ARGS__); return(rtn);} while(0)

bool JointTrajectoryInterface::init(std::string default_ip, int default_port)
{
  std::string ip;
  int port;
  this->cancel_motion_ = false;

  // override IP/port with ROS params, if available
  ros::param::param<std::string>("robot_ip_address", ip, default_ip);
  ros::param::param<int>("~trj_port", port, default_port);

  // check for valid parameter values
  if (ip.empty())
  {
    ROS_ERROR("No valid robot IP address found.  Please set ROS 'robot_ip_address' param");
    return false;
  }
  if (port <= 0)
  {
    ROS_ERROR("No valid robot IP port found.  Please set ROS '~trj_port' param");
    return false;
  }

  char* ip_addr = strdup(ip.c_str());  // connection.init() requires "char*", not "const char*"
  ROS_INFO("Joint Trajectory Interface connecting to IP address: '%s:%d'", ip_addr, port);
  default_tcp_connection_.init(ip_addr, port);  
  free(ip_addr);

  return init(&default_tcp_connection_);
}

bool JointTrajectoryInterface::init(SmplMsgConnection* connection)
{
  this->cancel_motion_ = false;
  
  std::vector<std::string> joint_names;
  if (!getJointNames("controller_joint_names", "robot_description", joint_names))
  {
    ROS_ERROR("Failed to initialize joint_names.  Aborting");
    return false;
  }

  return init(connection, joint_names);
}

bool JointTrajectoryInterface::init(SmplMsgConnection* connection, const std::vector<std::string> &joint_names,
                                    const std::map<std::string, double> &velocity_limits)
{
  
  this->connection_ = connection;
  this->all_joint_names_ = joint_names;
  this->joint_vel_limits_ = velocity_limits;
  this->cancel_motion_ = false;
  
  bool rtn = false;
  int max_try_num = 5;
  
  for (int i=0;i<max_try_num;i++)
  {
    if (!connection_->makeConnect())    
      ROS_WARN("Connection try %d failed!", i+1);
    else
    {
      rtn = true;
      break;
    }
    ros::Duration(1.0).sleep();
  }
  if(rtn)
    ROS_INFO("Connection done!");
  else
  {
    ROS_ERROR("Connection to the robot failed!");
    throw("Call to makeConnect() failed!");
    return false;
  }

  // try to read velocity limits from URDF, if none specified
  if (joint_vel_limits_.empty() && !industrial_utils::param::getJointVelocityLimits("robot_description", joint_vel_limits_))
    ROS_WARN("Unable to read velocity limits from 'robot_description' param.  Velocity validation disabled.");

  this->srv_start_motion_ = this->node_.advertiseService("start_motion", &JointTrajectoryInterface::startMotionCB, this);
  this->srv_stop_motion_ = this->node_.advertiseService("stop_motion", &JointTrajectoryInterface::stopMotionCB, this);
  this->srv_cancel_motion_ = this->node_.advertiseService("cancel_motion", &JointTrajectoryInterface::cancelMotionCB, this);
  this->srv_joint_trajectory_ = this->node_.advertiseService("comau_joint_path_command", &JointTrajectoryInterface::jointTrajectoryCB, this);

  return true;
}

JointTrajectoryInterface::~JointTrajectoryInterface()
{  
  trajectoryStop();
  named_mtx_.remove("jnt_trj_shared_mtx");
  
  std::ifstream f("/run/shm/sem.jnt_trj_shared_mtx");
  
  if ( f.good() == 0 )
    std::cout << "FILE: /run/shm/sem.jnt_trj_shared_mtx not properly removed! Please remove it manually. " << std::endl;
  
}

bool JointTrajectoryInterface::jointTrajectoryCB(comau_msgs::CmdJointTrjComau::Request &req,
                                                 comau_msgs::CmdJointTrjComau::Response &res)
{                   
  comau_msgs::JointTrajComauPtr traj_ptr(new comau_msgs::JointTrajComau);
  *traj_ptr = req.trajectory;  // copy message data
    
  if ( this->jointTrajectoryCB(traj_ptr) )
    res.code.val = 1;  //industrial_msgs::ServiceReturnCode::SUCCESS;
  else
    res.code.val = -1;
  
  return true;  // always return true.  To distinguish between call-failed and service-unavailable.
}

bool JointTrajectoryInterface::jointTrajectoryCB(const comau_msgs::JointTrajComauConstPtr &msg)
{
  ROS_INFO("Receiving joint trajectory message");

  if (msg->points.empty())
  {
    ROS_INFO("Empty trajectory received, canceling current trajectory");
    trajectoryStop();
    return true;
  }

  // convert trajectory into robot-format
  std::vector<JointTrajPtComauMessage> robot_msgs;
  if (!trajectory_to_msgs(msg, &robot_msgs))
    return false;

  // send command messages to robot
  return send_to_robot(robot_msgs);
      
}

bool JointTrajectoryInterface::trajectory_to_msgs(const comau_msgs::JointTrajComauConstPtr& traj, std::vector<JointTrajPtComauMessage>* msgs)
{
  msgs->clear();

  // check for valid trajectory
  if (!is_valid(*traj))
    return false;
  
  shared_int arm_number, digital_output, sequence;
  shared_real fly_tolerance;
  std::vector<shared_real> velocity, joint_pos;

  for (size_t i=0; i<traj->points.size(); ++i)
  {
    arm_number = traj->points[i].arm_number;
    sequence = traj->points[i].sequence_number;
    fly_tolerance = traj->points[i].fly_tolerance;
    velocity = traj->points[i].linear_velocity;
    joint_pos = traj->points[i].joint_positions;
    digital_output = traj->points[i].digital_output;
    

    JointTrajPtComauMessage msg = create_message( arm_number, sequence, fly_tolerance, velocity, joint_pos, digital_output );
    msgs->push_back(msg);
  }

  return true;
}

bool JointTrajectoryInterface::trajectory_to_msgs(const std::string traj, std::vector<JointTrajPtComauMessage>* msgs)
{
  ROS_INFO("Interface: trajectory_to_msgs");
  msgs->clear();
  
  // check if the trajectory is present in the parameter server
  if(!ros::param::has(traj))
  {
    ROS_ERROR("Trajectory '%s' not found!",traj.c_str());
    return false;
  }
  int nPoints = 0;
  ros::param::param<int>(traj+"/nPoints", nPoints, -1);
  if(nPoints==0)
  {
    ROS_ERROR("The trajectory is empty!");
    return false;
  }
  if(nPoints==-1)
  {
    ROS_ERROR("Impossible to find '/nPoints' in the parameter server!");
    return false;
  }
  std::string tmp_str;
  shared_int arm_number, digital_output, sequence;
  shared_real fly_tolerance;
  std::vector<shared_real> velocity, joint_pos;
  for (int i=0; i<nPoints; ++i)
  {
    tmp_str = traj+"/point_"+std::to_string(i);
    // retrive parameters values
    if(!ros::param::get(tmp_str+"/arm_number", arm_number))
    {
      ROS_WARN("Arm number not defined for point_%d. Default value (1) set.", i);
      arm_number = 1;
    }
    if(!ros::param::get(tmp_str+"/fly_tolerance", fly_tolerance))
    {
      ROS_WARN("Fly tolerance not defined for point_%d. Default value (1.0) set.", i);
      fly_tolerance = 1.0;
    }
    if(!ros::param::get(tmp_str+"/digital_output",digital_output))
    {
      ROS_WARN("Digital output not defined for point_%d. Default value (0) set.", i);
      digital_output = 0;
    }
    if(!ros::param::get(tmp_str+"/velocity",velocity))
    {
      ROS_ERROR("Velocity not defined for point_%d. Trajectory loading aborted!", i);
      return false;
    }
    if(!ros::param::get(tmp_str+"/joint_position",joint_pos))
    {
      ROS_ERROR("Joint position not defined for point_%d. Trajectory loading aborted!", i);
      return false;
    }
    sequence = i+1;
    
    JointTrajPtComauMessage msg = create_message(arm_number, sequence, fly_tolerance, velocity, joint_pos, digital_output);
    msgs->push_back(msg);
  }

  return true;
}

bool JointTrajectoryInterface::select(const std::vector<std::string>& ros_joint_names, const ros_JointTrajPtComau& ros_pt,
                                      const std::vector<std::string>& rbt_joint_names, ros_JointTrajPtComau* rbt_pt)
{
  ROS_ASSERT(ros_joint_names.size() == ros_pt.joint_positions.size());

  // initialize rbt_pt
  *rbt_pt = ros_pt;
  rbt_pt->joint_positions.clear();
  rbt_pt->linear_velocity.clear(); 

  for (size_t rbt_idx=0; rbt_idx < rbt_joint_names.size(); ++rbt_idx)
  {
    bool is_empty = rbt_joint_names[rbt_idx].empty();

    // find matching ROS element
    size_t ros_idx = std::find(ros_joint_names.begin(), ros_joint_names.end(), rbt_joint_names[rbt_idx]) - ros_joint_names.begin();
    bool is_found = ros_idx < ros_joint_names.size();

    // error-chk: required robot joint not found in ROS joint-list
    if (!is_empty && !is_found)
    {
      ROS_ERROR("Expected joint (%s) not found in JointTrajectory.  Aborting command.", rbt_joint_names[rbt_idx].c_str());
      return false;
    }

    if (is_empty) 
    {
      if (!ros_pt.joint_positions.empty()) 
        rbt_pt->joint_positions.push_back(default_joint_pos_);
      if (!ros_pt.linear_velocity.empty()) 
      {
        rbt_pt->linear_velocity[0] = -1;
        rbt_pt->linear_velocity[1] = -1;
      }
    }
    else
    {
      if (!ros_pt.joint_positions.empty()) 
        rbt_pt->joint_positions.push_back(ros_pt.joint_positions[ros_idx]);
      if (!ros_pt.linear_velocity.empty())
      {
        rbt_pt->linear_velocity[0] = ros_pt.linear_velocity[0];
        rbt_pt->linear_velocity[1] = ros_pt.linear_velocity[1];
      }
    }
  }
  return true;
}

JointTrajPtComauMessage JointTrajectoryInterface::create_message(shared_int arm_number, shared_int seq, shared_real fly_tol, std::vector<shared_real> velocity, 
                                                                 std::vector<shared_real> joint_pos, shared_int digital_out)
{
  std::vector<industrial::joint_data::JointData> pos(MAX_NUM_ARMS);
  for(int i=0;i<MAX_NUM_ARMS;i++)
    pos.at(i).init();
  std::vector<shared_real> vel(MAX_NUM_ARMS,0.0);

  if(joint_pos.size()>10 && MAX_NUM_ARMS==2)
  {
    for (size_t i=0; i<(joint_pos.size()/2); ++i)
      pos.at(0).setJoint(i, joint_pos[i]);
    for (size_t i=0; i<(joint_pos.size()/2); ++i)
      pos.at(1).setJoint(i, joint_pos[i+(joint_pos.size()/2)]);
  }
  else
  {
    if(MAX_NUM_ARMS==2 && arm_number==2)
    {
      for (size_t i=0; i<joint_pos.size(); ++i)
        pos.at(1).setJoint(i, joint_pos[i]);
    }
    else
    {
      for (size_t i=0; i<joint_pos.size(); ++i)
        pos.at(0).setJoint(i, joint_pos[i]);
    }
  }
  if(velocity.size()>1 && MAX_NUM_ARMS==2)
    vel = velocity;
  else
  {
    if(MAX_NUM_ARMS==2 && arm_number==2)
    {
      vel.at(1) = velocity.at(0);
    }
    else
    {
      vel.at(0) = velocity.at(0);
    }
  }      

  rbt_JointTrajPtComau pt;
  pt.init(arm_number, comau::simple_message::ComauQueueStatus::INVALID, seq, fly_tol, vel, pos, digital_out);

  JointTrajPtComauMessage msg;
  msg.init(pt);

  return msg;
}

void JointTrajectoryInterface::trajectoryStop()
{
  JointTrajPtComauMessage jMsg;
  SimpleMessage msg, reply;

  ROS_INFO("Joint trajectory handler: entering stopping trajectory state");
  jMsg.init(comau::simple_message::ComauMsgTypes::TRJ_FINISHED);
  jMsg.toRequest(msg);
  ROS_DEBUG("Sending stop command");
  streamingPause(); 
  ROS_DEBUG("trajectoryStop mutex lock");
  this->named_mtx_.lock();
  std::cout << "sending trj stop message\n";
  this->connection_->sendAndReceiveMsg(msg, reply);
  this->named_mtx_.unlock();
  ROS_DEBUG("trajectoryStop mutex unlocked");
  streamingResume();
  std::cout << "Interface stop reply: " << reply.getMsgLength() << std::endl;
}

bool JointTrajectoryInterface::startMotionCB( industrial_msgs::StartMotion::Request &req,
                                              industrial_msgs::StartMotion::Response &res)
{
  JointTrajPtComauMessage jMsg;
  SimpleMessage msg, reply;

  ROS_INFO("Joint trajectory interface: entering starting motion state");
  jMsg.init(comau::simple_message::ComauMsgTypes::START_MOTION);
  jMsg.toRequest(msg);
  ROS_DEBUG("Sending start command");
  streamingPause();
  
  ROS_DEBUG("startMotionCB mutex lock");
  this->named_mtx_.lock();
  this->cancel_motion_ = false;
  ros::Duration(0.5).sleep();
  this->connection_->sendAndReceiveMsg(msg, reply);
  this->named_mtx_.unlock();
  ROS_DEBUG("startMotionCB mutex unlock");
  streamingResume();
  
  if(reply.getReplyCode()==1 && reply.getMessageType()==comau::simple_message::ComauMsgTypes::START_MOTION && reply.getCommType()==3)
    res.code.val = industrial_msgs::ServiceReturnCode::SUCCESS;
  else
    res.code.val = industrial_msgs::ServiceReturnCode::FAILURE;

  return true;  // always return true.  To distinguish between call-failed and service-unavailable.
}

bool JointTrajectoryInterface::stopMotionCB(industrial_msgs::StopMotion::Request &req,
                                            industrial_msgs::StopMotion::Response &res)
{
  JointTrajPtComauMessage jMsg;
  SimpleMessage msg, reply;

  ROS_INFO("Joint trajectory interface: entering stopping motion state");

  jMsg.init(comau::simple_message::ComauMsgTypes::STOP_MOTION);
  jMsg.toRequest(msg);
  ROS_DEBUG("Sending stop command");
  streamingPause();
  this->named_mtx_.lock();
  std::cout << "Sending motion stop message\n";
  this->connection_->sendAndReceiveMsg(msg, reply);
  std::cout << "stop motion end reply: " << reply.getMsgLength() << std::endl;
  this->named_mtx_.unlock();
  streamingResume();
  
  if(reply.getReplyCode()==1 && reply.getMessageType()==comau::simple_message::ComauMsgTypes::STOP_MOTION && reply.getCommType()==3)
    res.code.val = industrial_msgs::ServiceReturnCode::SUCCESS;
  else
    res.code.val = industrial_msgs::ServiceReturnCode::FAILURE;
  
  return true;  // always return true.  To distinguish between call-failed and service-unavailable.
}


bool JointTrajectoryInterface::cancelMotionCB(industrial_msgs::StopMotion::Request &req,
                                              industrial_msgs::StopMotion::Response &res)
{
  JointTrajPtComauMessage jMsg;
  SimpleMessage msg, reply;

  ROS_INFO("Joint trajectory interface: entering cancel motion state");

  jMsg.init(comau::simple_message::ComauMsgTypes::CANCEL_MOTION);
  jMsg.toRequest(msg);
  ROS_DEBUG("Sending cancel motion command");
  streamingPause();
  std::cout << "Sending cancel motion message\n";
  this->named_mtx_.lock();
  this->connection_->sendAndReceiveMsg(msg, reply);
  this->cancel_motion_ = true;
  this->named_mtx_.unlock();
  std::cout << "cancel motion reply: " << reply.getMsgLength() << std::endl;
//   streamingResume();
  
  if(reply.getReplyCode()==1 && reply.getMessageType()==comau::simple_message::ComauMsgTypes::CANCEL_MOTION && reply.getCommType()==3)
    res.code.val = industrial_msgs::ServiceReturnCode::SUCCESS;
  else
    res.code.val = industrial_msgs::ServiceReturnCode::FAILURE;
  
//   streamingPause();

  return true;  // always return true.  To distinguish between call-failed and service-unavailable.
}


bool JointTrajectoryInterface::is_valid(const comau_msgs::JointTrajComau &traj)
{
  for (int i=0; i<traj.points.size(); ++i)
  {
    const comau_msgs::JointTrajPointComau &pt = traj.points[i];
    
    if (pt.arm_number == 0)
      ROS_ERROR_RETURN(false, "Validation failed: Invalid arm number for trajectory pt %d", i);
    
    // check for non-empty positions
    if (pt.joint_positions.empty())
      ROS_ERROR_RETURN(false, "Validation failed: Missing position data for trajectory pt %d", i);
        
    if (pt.linear_velocity.empty())
      ROS_ERROR_RETURN(false, "Validation failed: Missing velocity data for trajectory pt %d", i);
  }

  return true;
}

void JointTrajectoryInterface::jointStateCB(const sensor_msgs::JointStateConstPtr &msg)
{
  this->cur_joint_pos_ = *msg;
}

} //joint_trajectory_interface
} //comau


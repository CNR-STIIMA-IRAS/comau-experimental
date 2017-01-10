
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
#include <ros/ros.h>
#include <boost/exception_ptr.hpp>
#include <simple_message/simple_message.h>
#include <simple_message/byte_array.h>
#include <simple_message/shared_types.h>
#include <simple_message/joint_data.h>

#include <comau_msgs/comau_simple_message.h>
#include <comau_driver/comau_trajectory/joint_traj_pt_comau.h>
#include <comau_driver/comau_joint_streamer/comau_joint_trajectory_streamer.h>

using industrial::simple_message::SimpleMessage;
using industrial::byte_array::ByteArray;
using namespace industrial::shared_types;


namespace comau
{
namespace joint_trajectory_streamer
{

bool ComauJointTrajectoryStreamer::init(SmplMsgConnection* connection, const std::vector<std::string> &joint_names,
                                        const std::map<std::string, double> &velocity_limits)
{
  bool rtn = true;

  ROS_INFO("ComauJointTrajectoryStreamer: init");
  this->is_busy = false;
  this->trajectory_dwnl_complete_ = true;
  this->cyclic_traj_ = false;

  rtn &= JointTrajectoryInterface::init(connection, joint_names, velocity_limits);
  this->srv_reload_trj_ = this->node_.advertiseService("reload_trj", &ComauJointTrajectoryStreamer::reloadTrjCB, this);
    
  this->current_point_ = 0;
  this->state_ = TransferStates::IDLE;

  try
  {
    this->streaming_thread_ = new boost::thread(boost::bind(&ComauJointTrajectoryStreamer::streamingThread, this));
  } 
  catch (...)
  {
    ROS_ERROR("Thread creation exception!");
    return false;
  }

  ROS_INFO("ComauJointTrajectoryStreamer: initialized");

  return rtn;
}

ComauJointTrajectoryStreamer::~ComauJointTrajectoryStreamer()
{
  force_thread_stop_ = true;
  streaming_thread_->join();
  delete this->streaming_thread_;
}

bool ComauJointTrajectoryStreamer::jointTrajectoryCB(const comau_msgs::JointTrajComauConstPtr &msg)
{
  ROS_INFO("Receiving joint trajectory message");

  // read current state value (should be atomic)
  int state = this->state_;

  ROS_DEBUG("Current state is: %d", state);
  if (TransferStates::IDLE != state)
  {
    if (msg->points.empty())
      ROS_INFO("Empty trajectory received, canceling current trajectory");
    else
      ROS_ERROR("Trajectory splicing not yet implemented, stopping current motion.");

    trajectoryStop();
    
    return true;
  }

  if (msg->points.empty())
  {
    ROS_INFO("Empty trajectory received while in IDLE state, nothing is done");
    return true;
  }

  // calc new trajectory
  std::vector<JointTrajPtComauMessage> new_traj_msgs;
  if (!trajectory_to_msgs(msg, &new_traj_msgs))
    return false;

  // send command messages to robot
  return send_to_robot(new_traj_msgs); 
}

bool ComauJointTrajectoryStreamer::reloadTrjCB( industrial_msgs::StartMotion::Request &req,
                                                industrial_msgs::StartMotion::Response &res )
{
  ROS_INFO("Reloading trajectory!");
  
  if ( this->named_mtx_.try_lock() )
  {
    for (int i=0;i<this->current_traj_.size();i++)
    {
      this->current_traj_.at(i).setSequence(i+1);
    }
    {
      this->cyclic_traj_ = false;
      this->current_point_ = 0;
      this->is_busy = false;
      this->state_ = TransferStates::STREAMING;
      this->streaming_start_ = ros::Time::now();
    }
    this->named_mtx_.unlock();
    res.code.val = industrial_msgs::ServiceReturnCode::SUCCESS;
  }
  else
  {
    res.code.val = industrial_msgs::ServiceReturnCode::SUCCESS;
  }
  return true;
}

bool ComauJointTrajectoryStreamer::loadTrjFromParam(const std::string &trj_name) 
{
  ROS_INFO("Reading joint trajectory from parameter server");

  if(!ros::param::has(trj_name))   
  {
    ROS_INFO("Trajectory is not present in the parameter server");
    return false;
  }
  
  // read current state value (should be atomic)
  int state = this->state_;

  ROS_DEBUG("Current state is: %d", state);

  if (TransferStates::IDLE != state)
  {
    //add control on trajectory size
  }
  
  std::vector<JointTrajPtComauMessage> new_traj_msgs;
  if (!trajectory_to_msgs(trj_name, &new_traj_msgs))
    return false;
  
  return send_to_robot(new_traj_msgs);
}

bool ComauJointTrajectoryStreamer::send_to_robot(const std::vector<JointTrajPtComauMessage>& messages)
{
  ROS_INFO("Loading trajectory, setting state to streaming");
  
  size_t n_trial = 0;
  while ( n_trial<5 ) 
  {
    if ( this->named_mtx_.try_lock() )
    {
      {
        ROS_INFO("Executing trajectory of size: %d\n", (int)messages.size());
        this->current_traj_ = messages;
        this->current_point_ = 0;
	this->is_busy = false;
        this->state_ = TransferStates::STREAMING;
        this->streaming_start_ = ros::Time::now();
      }
      this->named_mtx_.unlock();

      return true;
    }
    else
    {
      n_trial++;
      ROS_WARN("Cannot update data in function send_to_robot(), trial: %zu", n_trial); 
    }
    
    ros::Duration(0.05).sleep();
    ros::spinOnce();
    
  }
  
  ROS_ERROR("IMPOSSIBLE to update data in function send_to_robot()");
  return false;
}

bool ComauJointTrajectoryStreamer::trajectory_to_msgs(const comau_msgs::JointTrajComauConstPtr &traj, std::vector<JointTrajPtComauMessage>* msgs)
{
  // use base function to transform points
  if (!JointTrajectoryInterface::trajectory_to_msgs(traj, msgs))
    return false;

  // pad trajectory as required for minimum streaming buffer size
  if (!msgs->empty() && (msgs->size() < (size_t)min_buffer_size_))
  {
    ROS_DEBUG("Padding trajectory: current(%d) => minimum(%d)", (int)msgs->size(), min_buffer_size_);
    while (msgs->size() < (size_t)min_buffer_size_)
      msgs->push_back(msgs->back());
  }

  return true;
}

bool ComauJointTrajectoryStreamer::trajectory_to_msgs(const std::string traj, std::vector<JointTrajPtComauMessage>* msgs)
{
  ROS_INFO("Streamer: trajectory_to_msgs");
  // use base function to transform points
  if (!JointTrajectoryInterface::trajectory_to_msgs(traj, msgs))
    return false;

  // pad trajectory as required for minimum streaming buffer size
  if (!msgs->empty() && (msgs->size() < (size_t)min_buffer_size_))                     
  {
    ROS_DEBUG("Padding trajectory: current(%d) => minimum(%d)", (int)msgs->size(), min_buffer_size_);
    while (msgs->size() < (size_t)min_buffer_size_)
      msgs->push_back(msgs->back());
  }

  return true;
}


void ComauJointTrajectoryStreamer::streamingThread()
{
  JointTrajPtComauMessage jtpMsg;
  int connectRetryCount = 1;

  ROS_INFO("Starting joint trajectory streamer thread");
  while ( ros::ok() && !(force_thread_stop_) )
  {
    ros::Duration(0.005).sleep();

    // automatically re-establish connection, if required
    if (connectRetryCount-- > 0)
    {
      ROS_INFO("Connecting to robot server");
      this->connection_->makeConnect();
      ros::Duration(0.250).sleep();  // wait for connection

      if (this->connection_->isConnected())
        connectRetryCount = 0;
      else if (connectRetryCount <= 0)
      {
        ROS_ERROR("Timeout connecting to robot controller.  Send new motion command to retry.");
        this->state_ = TransferStates::IDLE;
      }
      continue;
    }
    
    while ( this->is_busy )
    {
      if(this->state_ == TransferStates::IDLE)
      {
        ros::Duration(0.50).sleep();
      }
      else
      {
        SimpleMessage msg, reply;
        ROS_WARN("Busy queue!");
        
        jtpMsg.init(comau::simple_message::ComauMsgTypes::PING);
        jtpMsg.toRequest(msg);
        
        this->named_mtx_.lock();
        if(!this->connection_->sendAndReceiveMsg(msg, reply, false))
          ROS_ERROR("Sending message error!");
        this->named_mtx_.unlock();
        
        JointTrajPtComauMessage pt;        
        pt.init(reply);

        switch (pt.point_.getQueueStatus())
        {
          case comau::simple_message::ComauQueueStatus::ERROR:
            ROS_ERROR("Controller reply: ERROR. ");
            return;
            break;
          case comau::simple_message::ComauQueueStatus::INVALID:
            ROS_ERROR("Invalid controller reply. ");
            return;
            break;
          case comau::simple_message::ComauQueueStatus::READY:
            this->is_busy = false;
            break;
          default:
            this->is_busy = true;
            break;
        }
        ros::Duration(0.5).sleep();
      }
    }
    
    
    SimpleMessage msg, reply;    
    bool point_sended_ = false;
     
    switch (this->state_)
    {
      case TransferStates::IDLE:
        ros::Duration(0.05).sleep();  //  slower loop while waiting for new trajectory
        break;

      case TransferStates::STREAMING:
        this->named_mtx_.lock();
        this->trajectory_dwnl_complete_ = false;
        this->named_mtx_.unlock();
        
        if (this->current_point_ >= (int)this->current_traj_.size())
        {
	  if(this->cyclic_traj_ == true)
	  {
	    int seq = this->current_traj_.back().point_.getSequence();
	    for(int i=0;i<this->current_traj_.size();i++)
	    {
	      this->current_traj_.at(i).setSequence(seq+1+i);
	    }
	    this->current_point_ = 0;
	  }
	  else
	  {
	    ROS_INFO("Trajectory streaming complete, setting state to IDLE");
	    this->state_ = TransferStates::IDLE;
	    this->trajectoryStop();
	    this->trajectory_dwnl_complete_ = true;
	    break;
	  }
        }

        if (!this->connection_->isConnected())
        {
          ROS_DEBUG("Robot disconnected.  Attempting reconnect...");
          connectRetryCount = 5;
          break;
        }

        ROS_INFO("Sending point %d.", this->current_point_+1);
        jtpMsg = this->current_traj_[this->current_point_];
        jtpMsg.toRequest(msg);
       
        ROS_DEBUG("Sending joint trajectory point");
        
        point_sended_ = false;
        while ( !point_sended_ )
        {
          ROS_DEBUG("named_mtx_.try_lock()");
          if ( this->named_mtx_.try_lock() )
          {
            ROS_DEBUG("Before sendAndReceiveMsg() ");
            if (this->connection_->sendAndReceiveMsg(msg, reply, false))
            {
              ROS_INFO("Point[%d of %d] sent to controller. \n", this->current_point_+1, (int)this->current_traj_.size());
              this->current_point_++;

              JointTrajPtComauMessage pt;        
              pt.init(reply);         
              
              if ( pt.point_.getQueueStatus() == comau::simple_message::ComauQueueStatus::BUSY ) 
                this->is_busy = true;
            }
            else
              ROS_INFO("Failed sent joint point, will try again");
            
            this->named_mtx_.unlock();
            ROS_DEBUG("named_mtx_.unlock()");
            
            point_sended_ = true;
          }
          
          ros::Duration(0.001).sleep();  
        }
        
        break;
      
      default:
        ROS_ERROR("Joint trajectory streamer: unknown state");
        this->state_ = TransferStates::IDLE;
        break;
    }
  }
  
  ROS_WARN("Exiting trajectory streamer thread");
}

void ComauJointTrajectoryStreamer::trajectoryStop()
{
//   streamingPause();
  this->is_busy = false;
  
  JointTrajectoryInterface::trajectoryStop();
  
  ROS_DEBUG("Stop command sent, entering idle mode");
}


void ComauJointTrajectoryStreamer::streamingPause(const double time_sec)
{
  std::cout << "State before pause: " << this->state_ << std::endl;
  this->previous_state_ = this->state_;
  this->state_ = TransferStates::IDLE;
  if (time_sec>0)
  {
    ros::Duration(time_sec).sleep();
    this->state_ = this->previous_state_;
  }
  std::cout << "State after pause: " << this->state_ << std::endl;
}

void ComauJointTrajectoryStreamer::streamingResume()
{
  std::cout << "State before resume: " << this->state_ << std::endl;
  this->state_ = this->previous_state_;
  std::cout << "State after resume: " << this->state_ << std::endl;
}

int ComauJointTrajectoryStreamer::getTrajectorySts()
{
  // sts_ = 3 --> Cancel Motion request && Trajectory Downloading Completed
  // sts_ = 2 --> Trajectory Downloading complete without any Cancel Motion request
  // sts_ = 1 --> Cancel Motion request && Trajectory Downloading NOT Completed
  // sts_ = 0 --> Trajectory is in downloading
  
  int sts_;
  if ( this->trajectory_dwnl_complete_ && this->cancel_motion_ )
  {
//     std::cout << "Trajectory STATE 3" << std::endl;
    sts_ = 3;
  } 
  else if ( this->trajectory_dwnl_complete_ && !this->cancel_motion_ ) 
  {
    sts_ = 2; 
  }
  else if ( !this->trajectory_dwnl_complete_ && this->cancel_motion_ )
  {
//     std::cout << "Trajectory STATE 1" << std::endl;
    sts_ = 1;
  }
  else
  {
    sts_ = 0;
  }
  return sts_;
}

void ComauJointTrajectoryStreamer::setCyclicTrj(const bool is_cyclic)
{
  this->cyclic_traj_ = is_cyclic;
}



} //joint_trajectory_streamer
} //comau


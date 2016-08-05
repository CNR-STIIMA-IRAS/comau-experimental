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
#include <std_msgs/Int32.h>
#include <industrial_utils/param_utils.h>
#include <comau_driver/comau_joint_streamer/comau_joint_trajectory_streamer.h>


//#define LINUXSOCKETS

using namespace comau::joint_trajectory_streamer;

int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "trajectory_streamer");
  ros::NodeHandle nh;
  ros::Rate loop_rate( 1000 );
  
  std_msgs::Int32 trj_sts_;
  // advertise the trajectory status topic
  ros::Publisher pub_trj_sts  = nh.advertise<std_msgs::Int32>("/comau_trajectory_sts", 1, false );   
  

  // launch the default ComauJointTrajectoryStreamer connection/handlers
  ComauJointTrajectoryStreamer trajectoryStreamer;
  
  // initialize the streamer
  trajectoryStreamer.init();        // load ip_address from param
  
  // set the initial value for the trajectory status
  trj_sts_.data = 0;  
  while (ros::ok())
  {
    // retrive the trajectory status with the specific method:
    // 3 --> Cancel Motion request && Trajectory Downloading Completed
    // 2 --> Trajectory Downloading complete without any Cancel Motion request
    // 1 --> Cancel Motion request && Trajectory Downloading NOT Completed
    // 0 --> Trajectory is in downloading
    trj_sts_.data = trajectoryStreamer.getTrajectorySts(); 
    
    // publish the status on the topic
    pub_trj_sts.publish( trj_sts_ );
        
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}

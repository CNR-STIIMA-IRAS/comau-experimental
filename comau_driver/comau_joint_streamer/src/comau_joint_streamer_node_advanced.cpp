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


using namespace comau::joint_trajectory_streamer;

// Using methods of the comau_driver 

// Before running this code, load the example trajectory (or your own) in the parameter server with the command
// rosparam load example_trj.yaml
// from the linux terminal (be careful that the example trajectory is made for a Comanu NS16-Hand robot!!!).
// The example_trj.yaml file is placed in the test folder of the comau_driver.

int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "trajectory_streamer_adv");
  ros::NodeHandle nh;  

  // create an object of type ComauJointTrajectoryStreamer for using its methods
  ComauJointTrajectoryStreamer trajectoryStreamer;
  
  // initialize the new object
  trajectoryStreamer.init("192.168.254.240",1212); // initialize the new object, connectiong to a specify ip_address and port
                                                   // the ip_address is the one of the robot controller, the 1212 port is the 
                                                   // default one of this driver, if you change it, it has to be modified
                                                   // also in the PDL code
  trajectoryStreamer.loadTrjFromParam("/test_trj"); // load a specific trajectory from the parameter server,
                                                    // the name is the same of the trajectory namespace (loaded with the yaml
                                                    // file before)
  trajectoryStreamer.run();   // enter in an infinite loop 

  return 0;
}
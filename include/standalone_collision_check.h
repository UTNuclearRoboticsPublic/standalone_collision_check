
// Copyright (c) 2016, The University of Texas at Austin
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
// OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
// OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
// ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef STANDALONE_COLLISION_CHECK_H
#define STANDALONE_COLLISION_CHECK_H

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <string> 

// MoveIt!
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model/joint_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <eigen_conversions/eigen_msg.h>


namespace standalone_collision_check
{
  // Update robot joints
  void jointCallback(sensor_msgs::JointStateConstPtr msg);

  // Spawn a cube for testing
  void spawn_collision_cube(ros::NodeHandle& nh);

  // Read launch file arguments
  void read_launch_args(ros::NodeHandle& nh);


  std::vector<std::string> g_joint_names;

  std_msgs::String g_urscript_string;

  char g_ur_cmd [400];
  double g_deceleration = 3.;  // How fast to stop

  sensor_msgs::JointState g_my_joint_info;

  std::string g_group_name;  // Read from launch file
  bool g_test_with_cube = false;  // Read from launch file
  bool g_test_with_random_joints = false;  // Read from launch file
  std::string g_ur_topic_name;  // Read from launch file
  std::string g_kill_cmd;  // Read from launch file
}
using namespace standalone_collision_check;

#endif

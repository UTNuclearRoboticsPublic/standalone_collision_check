
#ifndef STANDALONE_COLLISION_CHECK_H
#define STANDALONE_COLLISION_CHECK_H

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <string> 

// MoveIt!
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene/planning_scene.h>
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

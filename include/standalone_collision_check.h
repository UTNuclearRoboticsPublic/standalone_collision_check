
#ifndef STANDALONE_COLLISION_CHECK_H
#define STANDALONE_COLLISION_CHECK_H

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <string> 

// MoveIt!
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <eigen_conversions/eigen_msg.h>


// Update robot joints
void jointCallback(const sensor_msgs::JointState &msg);

// Spawn a cube for testing
void spawn_collision_cube(ros::NodeHandle& nh);

// Read launch file arguments
void read_launch_args(ros::NodeHandle& nh);

namespace standalone_collision_check
{
  int g_num_joints = 6;  // Read from launch file
  std::vector<double> g_current_joints;

  std::string g_group_name;  // Read from launch file

  bool g_test_with_cube = false;  // Read from launch file
  bool g_test_with_random_joints = false;  // Read from launch file

  std_msgs::String g_urscript_string;

  char g_ur_cmd [400];
  double g_deceleration = 3.;  // How fast to stop
  std::string g_ur_topic_name;  // Name of the URscript topic
  std::string g_node_to_kill;  // Name of the ROS node to kill, e.g. the joystick node
}
using namespace standalone_collision_check;

#endif

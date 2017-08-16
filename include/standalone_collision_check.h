
#ifndef STANDALONE_COLLISION_CHECK_H
#define STANDALONE_COLLISION_CHECK_H

#include <ros/ros.h>
#include "std_msgs/Bool.h"
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

  bool g_test_with_cube = false;  // Read from launch file
  bool g_test_with_random_joints = false;  // Read from launch file
}
using namespace standalone_collision_check;

#endif


// Refer to http://docs.ros.org/indigo/api/moveit_tutorials/html/doc/pr2_tutorials/planning/src/doc/planning_scene_tutorial.html

#include <standalone_collision_check.h>

int main(int argc, char** argv) {

  ros::init (argc, argv, "collision_check");
  ros::NodeHandle nh;

  // Read string arguments from launch file, then convert to proper data types
  ros::NodeHandle pn("~");
  read_launch_args(pn);

  // Send URscript commands
  ros::Publisher vel_pub = nh.advertise<std_msgs::String>( g_ur_topic_name, 1);
  while (vel_pub.getNumSubscribers() == 0)
  {
    ROS_INFO_STREAM("Waiting for URScript publisher creation");
    ros::Duration(0.1).sleep();
  }

  ros::Subscriber sub = nh.subscribe("joint_states", 5, jointCallback);

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);
  collision_detection::CollisionRequest collision_request;
  collision_request.group_name = g_group_name;
  collision_detection::CollisionResult collision_result;
  robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();

  // Spawn a virtual collision object? (for testing)
  if (g_test_with_cube)
    spawn_collision_cube(nh);

  std::vector<double> joint_values;
  const robot_model::JointModelGroup* joint_model_group = current_state.getJointModelGroup(g_group_name);

  /////////////////////////////////////////////////
  // Spin while checking collisions
  /////////////////////////////////////////////////
  while ( ros::ok() )
  {

    // For testing: overwrite actual joint values with randoms
    if (g_test_with_random_joints)
      current_state.setToRandomPositions();
    else
    {
      current_state.copyJointGroupPositions(joint_model_group, joint_values);
      ROS_INFO_STREAM( joint_values.at(0) );
      current_state.setJointGroupPositions(joint_model_group, joint_values);
    }


    // Are the joints being updated?
    const double* left_ur5_shoulder_pan_joint = current_state.getJointPositions("left_ur5_shoulder_pan_joint");
    ROS_INFO_STREAM( *left_ur5_shoulder_pan_joint );


    collision_result.clear();
    planning_scene.checkCollision(collision_request, collision_result);


    // Bring the robot to a halt and kill the joystick node
    if ( collision_result.collision )
    {
      ROS_WARN("[standalone_collision_check] Halting!");
/*
      std::string s = "rosnode kill " + g_node_to_kill;
      system(s.c_str());

      // This is specific to Universal Robots
      sprintf(g_ur_cmd, "Stop_l(%f)", g_deceleration);
      g_urscript_string.data = g_ur_cmd;
      vel_pub.publish(g_urscript_string);

      return 0;
*/
    }

    ros::spinOnce();
    ros::Duration(.01).sleep();
  }

  ros::shutdown();
  return 0;
}


void jointCallback(const sensor_msgs::JointState &msg)
{
  for (int i=0; i<g_num_joints; i++)
    g_current_joints.at(i) = msg.position[i];
  
  return;
}


// Spawn a collision cube for test purposes
void spawn_collision_cube(ros::NodeHandle& nh)
{
  ROS_INFO_STREAM("Spawning a collision cube...");

  ros::Publisher collision_object_publisher = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
  while(collision_object_publisher.getNumSubscribers() < 1)
  {
    ROS_INFO_STREAM("Waiting for collision_object publisher creation");
    ros::Duration(0.1).sleep();
  }

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "base_link";
  collision_object.id = "box";

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = .2;
  primitive.dimensions[1] = .2;
  primitive.dimensions[2] = .2;
  collision_object.primitives.resize(1);
  collision_object.primitives[0] = primitive;

  geometry_msgs::Pose pose;
  pose.position.x = -0.4;
  pose.orientation.w = 1.0;
  collision_object.primitive_poses.resize(1);
  collision_object.primitive_poses[0] = pose;

  collision_object.operation = collision_object.ADD;
  collision_object_publisher.publish( collision_object );

  return;
}


// Read string arguments from launch file, then convert to proper data types
void read_launch_args(ros::NodeHandle& nh)
{
  while (!nh.hasParam("test_with_cube"))
  {
    ros::Duration(0.1).sleep();
  }
  nh.param<bool>("test_with_cube", g_test_with_cube, false);
  while (!nh.hasParam("test_with_random_joints"))
  {
    ros::Duration(0.1).sleep();
  }
  nh.param<bool>("test_with_random_joints", g_test_with_random_joints, false);
  while (!nh.hasParam("num_joints"))
  {
    ros::Duration(0.1).sleep();
  }
  nh.param<int>("num_joints", g_num_joints, 6);
  while (!nh.hasParam("group_name"))
  {
    ros::Duration(0.1).sleep();
  }
  nh.param<std::string>("group_name", g_group_name, "manipulator");
  while (!nh.hasParam("ur_topic_name"))
  {
    ros::Duration(0.1).sleep();
  }
  nh.param<std::string>("ur_topic_name", g_ur_topic_name, "/ur_driver/URScript");
  while (!nh.hasParam("node_to_kill"))
  {
    ros::Duration(0.1).sleep();
  }
  nh.param<std::string>("node_to_kill", g_node_to_kill, "/joy_teleop/joy_node");


  ROS_INFO_STREAM("----------------------------");
  ROS_INFO_STREAM("----------------------------");
  ROS_INFO_STREAM("Configuration from launch file: ");
  ROS_INFO_STREAM("Number of joints: " << g_num_joints);
  ROS_INFO_STREAM("MoveIt group to check: " << g_group_name);
  ROS_INFO_STREAM("URScript topic: " << g_ur_topic_name);
  //ROS_INFO( "%d", g_test_with_cube );
  //ROS_INFO( "%d", g_test_with_random_joints );
  ROS_INFO_STREAM( (g_test_with_cube ? "test with virtual cube" : "no virtual collision cube will be added") );
  ROS_INFO_STREAM( (g_test_with_random_joints ? "test with random joints" : "use actual robot joints") );
  ROS_INFO_STREAM("----------------------------");
  ROS_INFO_STREAM("----------------------------");

  g_current_joints.resize(g_num_joints);

  return;
}

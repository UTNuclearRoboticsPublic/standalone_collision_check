
#include <standalone_collision_check.h>

int main(int argc, char** argv) {

  ros::init (argc, argv, "collision_check");
  ros::NodeHandle nh;

  // Read string arguments from launch file, then convert to proper data types
  ros::NodeHandle pn("~");
  read_launch_args(pn);

  // Send URscript commands
  ros::Publisher vel_pub = nh.advertise<std_msgs::String>( g_ur_topic_name, 1);
  /*while (vel_pub.getNumSubscribers() == 0)
  {
    ROS_INFO_STREAM("[standalone collision check] Waiting for URScript publisher creation");
    ros::Duration(0.1).sleep();
  }*/

  ros::Subscriber sub = nh.subscribe("joint_states", 1, jointCallback);

  moveit::planning_interface::MoveGroup* move_group_ptr = new moveit::planning_interface::MoveGroup(g_group_name);
  g_my_joint_info.name = move_group_ptr -> getJointNames();
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);
  collision_detection::CollisionRequest collision_request;
  collision_request.group_name = g_group_name;
  collision_detection::CollisionResult collision_result;
  robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  // Initialize the joint position vector
  for (int i=0; i<g_my_joint_info.name.size(); i++)
    g_my_joint_info.position.push_back(0.);

  // Spawn a virtual collision object? (for testing)
  if (g_test_with_cube)
    spawn_collision_cube(nh);

  // Wait for a /joint_state msg to initialize g_my_joint_info
  while ( g_my_joint_info.position.size() < 1 )
  {
    ROS_INFO_STREAM("[standalone_collision_check] Waiting for initial joint_state msg");
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  ROS_INFO_STREAM("[standalone collision check] Received the initial joint_state msg.");

  /////////////////////////////////////////////////
  // Spin while checking collisions
  /////////////////////////////////////////////////
  while ( ros::ok() )
  {

    // Get joint angles
    if (g_test_with_random_joints)
      current_state.setToRandomPositions();
    else
    {
      for (int i=0; i<g_my_joint_info.position.size(); i++)
        current_state.setJointPositions( g_my_joint_info.name[i], &g_my_joint_info.position[i] );
    }

    //process collision objects in scene
    std::map<std::string, moveit_msgs::CollisionObject> c_objects_map = planning_scene_interface_.getObjects();
    for(auto& kv : c_objects_map){
      planning_scene.processCollisionObjectMsg(kv.second);
    }

    collision_result.clear();
    planning_scene.checkCollision(collision_request, collision_result);


    // Bring the robot to a halt and optionally kill a process, e.g. joystick input
    if ( collision_result.collision )
    {
      if ( !g_kill_cmd.empty() )
        system(g_kill_cmd.c_str());

      // This is specific to Universal Robots
      sprintf(g_ur_cmd, "Stop_l(%f)", g_deceleration);
      g_urscript_string.data = g_ur_cmd;
      vel_pub.publish(g_urscript_string);

      // Give time for another process to read the changed environment variable
      ROS_WARN("[standalone_collision_check] Halting!");
      ros::Duration(5).sleep();

      return 0;
    }

    ros::spinOnce();
    ros::Duration(.01).sleep();
  }

  ros::shutdown();
  return 0;
}


void standalone_collision_check::jointCallback(sensor_msgs::JointStateConstPtr msg)
{
  // Store joints in a global variable
  for (int m=0; m<msg->name.size(); m++)
  {
    for (int c=0; c<g_my_joint_info.name.size(); c++)
    {
      if ( msg->name[m] == g_my_joint_info.name[c] )
      {
        g_my_joint_info.position[c] = msg->position[m];
        goto NEXT_JOINT;
      }
    }
NEXT_JOINT:
    ;
  }
  
  return;
}


// Spawn a collision cube for test purposes
void standalone_collision_check::spawn_collision_cube(ros::NodeHandle& nh)
{
  ROS_INFO_STREAM("[standalone collision check] Spawning a collision cube...");

  ros::Publisher collision_object_publisher = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 1, true);  // latch it
  while(collision_object_publisher.getNumSubscribers() < 1)
  {
    ROS_INFO_STREAM("[standalone collision check] Waiting for collision_object connections");
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
  pose.position.x = 0.2;
  pose.position.z = 0.3;
  pose.orientation.w = 1.0;
  collision_object.primitive_poses.resize(1);
  collision_object.primitive_poses[0] = pose;

  collision_object.operation = collision_object.ADD;
  collision_object_publisher.publish( collision_object );
  ros::Duration(0.005).sleep();

  return;
}


// Read string arguments from launch file, then convert to proper data types
void standalone_collision_check::read_launch_args(ros::NodeHandle& nh)
{
  while (!nh.hasParam("test_with_cube"))
  {
    ros::Duration(0.5).sleep();
    ROS_INFO_STREAM("Cannot read the 'test_with_cube' param from the launch file.");
  }
  nh.param<bool>("test_with_cube", g_test_with_cube, false);
  while (!nh.hasParam("test_with_random_joints"))
  {
    ros::Duration(0.5).sleep();
    ROS_INFO_STREAM("Cannot read the 'test_with_random_joints' param from the launch file.");
  }
  nh.param<bool>("test_with_random_joints", g_test_with_random_joints, false);
  while (!nh.hasParam("group_name"))
  {
    ros::Duration(0.5).sleep();
    ROS_INFO_STREAM("Cannot read the 'group_name' param from the launch file.");
  }
  nh.param<std::string>("group_name", g_group_name, "manipulator");
  while (!nh.hasParam("ur_topic_name"))
  {
    ros::Duration(0.5).sleep();
    ROS_INFO_STREAM("Cannot read the 'ur_topic_name' param from the launch file.");
  }
  nh.param<std::string>("ur_topic_name", g_ur_topic_name, "/ur_driver/URScript");
  while (!nh.hasParam("kill_cmd"))
  {
    ros::Duration(0.5).sleep();
    ROS_INFO_STREAM("Cannot read the 'kill_cmd' param from the launch file. Leave it as a blank string if it is not necessary.");
  }
  nh.param<std::string>("kill_cmd", g_kill_cmd, "/joy_teleop/joy_node");


  ROS_INFO_STREAM("----------------------------");
  ROS_INFO_STREAM("----------------------------");
  ROS_INFO_STREAM("Configuration from launch file: ");
  ROS_INFO_STREAM("MoveIt group to check: " << g_group_name);
  ROS_INFO_STREAM("URScript topic: " << g_ur_topic_name);
  //ROS_INFO( "%d", g_test_with_cube );
  //ROS_INFO( "%d", g_test_with_random_joints );
  ROS_INFO_STREAM( (g_test_with_cube ? "test with virtual cube" : "no virtual collision cube will be added") );
  ROS_INFO_STREAM( (g_test_with_random_joints ? "test with random joints" : "use actual robot joints") );
  ROS_INFO_STREAM("----------------------------");
  ROS_INFO_STREAM("----------------------------");

  return;
}

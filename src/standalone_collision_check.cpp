
// Refer to http://docs.ros.org/indigo/api/moveit_tutorials/html/doc/pr2_tutorials/planning/src/doc/planning_scene_tutorial.html

#include <standalone_collision_check.h>

int main(int argc, char** argv) {

  ros::init (argc, argv, "collision_check");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("joint_states", 5, jointCallback);

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;

  robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();

  // Send URscript commands
  ros::Publisher vel_pub = nh.advertise<std_msgs::String>("/ur_driver/URScript", 1);

  // Read string arguments from launch file, then convert to proper data types
  ros::NodeHandle pn("~");
  read_launch_args(pn);

  // Spawn a virtual collision object (for testing)
  if (g_test_with_cube)
    spawn_collision_cube(nh);

  // Spin while checking minimum collision distance
  while ( ros::ok() )
  {
    //ROS_INFO_STREAM( g_current_joints.at(0) <<"  " << g_current_joints.at(1) );
    current_state = planning_scene.getCurrentStateNonConst();

    // For testing: overwrite actual joint values with randoms
    if (g_test_with_random_joints)
      current_state.setToRandomPositions();
    else
      current_state.setVariablePositions(g_current_joints);

    collision_result.clear();

    planning_scene.checkCollision(collision_request, collision_result);
    /*ROS_INFO_STREAM("Current state is "
                  << (collision_result.collision ? "in" : "not in")
                  << " collision"); */

    // Bring the robot to a halt
    // This is UR-specific
    if ( collision_result.collision )
    {
      ROS_WARN("[standalone_collision_check] Halting!");

      sprintf(g_ur_cmd, "Stop_l(%f)", g_deceleration);
      g_urscript_string.data = g_ur_cmd;

      // Quickly pump out 'stop' commands.
      // Essentially, overwhelm any other URScript commands going to the robot.
      // This doesn't work for pendant commands, but yes for XBox cmds via URx.
      while (ros::ok())
      {
        vel_pub.publish(g_urscript_string);
        ros::Duration(0.005).sleep();
      }
    }

    ros::spinOnce();
    ros::Duration(.01).sleep();
  }

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


  ROS_INFO_STREAM("Configuration from launch file: ");
  ROS_INFO_STREAM("Number of joints: " << g_num_joints);
  //ROS_INFO( "%d", g_test_with_cube );
  //ROS_INFO( "%d", g_test_with_random_joints );
  ROS_INFO_STREAM( (g_test_with_cube ? "test with virtual cube" : "no virtual collision cube will be added") );
  ROS_INFO_STREAM( (g_test_with_random_joints ? "test with random joints" : "use actual robot joints") );

  g_current_joints.resize(g_num_joints);

  return;
}


// Refer to https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/pr2_tutorials/interactivity/src/collision_contact_tutorial.cpp

#include <standalone_collision_check.h>

int main(int argc, char** argv) {

  ros::init (argc, argv, "collision_check");
  ros::NodeHandle nh;

  ros::Duration(1.0).sleep();

  // Set up the MoveIt scene
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  robot_state::RobotState robot_state = planning_scene.getCurrentState();
  collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();

  ros::Duration(10).sleep();
  // Spawn a virtual collision object (for testing)
  spawn_collision_cube(nh);

  // Spin while checking minimum collision distance
  while ( ros::ok() )
  {
    collision_result.clear();
    robot_state = planning_scene.getCurrentState();
    //planning_scene.checkCollision(collision_request, collision_result, robot_state, acm);
    //ROS_INFO_STREAM("Current state is " << (collision_result.collision ? "in" : "not in") << " collision");
    ROS_INFO_STREAM("Minimum distance is " << planning_scene.distanceToCollision( robot_state ) << ".");

    ros::Duration(1.).sleep();
  }

  return 0;
}


void spawn_collision_cube(ros::NodeHandle& nh)
{
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
  primitive.dimensions[0] = 0.05;
  primitive.dimensions[1] = 0.05;
  primitive.dimensions[2] = 0.05;
  collision_object.primitives.resize(1);
  collision_object.primitives[0] = primitive;

  geometry_msgs::Pose pose;
  pose.position.x = 0.8;
  pose.orientation.w = 1.0;
  collision_object.primitive_poses.resize(1);
  collision_object.primitive_poses[0] = pose;

  collision_object.operation = collision_object.ADD;
  collision_object_publisher.publish( collision_object );

  return;
}

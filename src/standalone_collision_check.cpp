
// Refer to https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/pr2_tutorials/interactivity/src/collision_contact_tutorial.cpp

#include <standalone_collision_check.h>


int main(int argc, char** argv) {

  ros::init (argc, argv, "collision_check");
  ros::NodeHandle nh;

  // Set up the MoveIt scene
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  robot_state::RobotState robot_state = planning_scene.getCurrentState();
  collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();


  // Spawn a virtual collision object (for testing)
  ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  while(planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    ros::Duration(0.1).sleep();
  }

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "base_link";
  collision_object.id = "box";
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.5;
  primitive.dimensions[1] = 0.5;
  primitive.dimensions[2] = 0.5;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(pose);

  planning_scene.world.collision_objects.push_back(collision_object.object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);
  ros::Duration(0.1).sleep();


  // Spin while checking minimum collision distance
  while ( ros::ok() )
  {
    collision_result.clear();
    robot_state = planning_scene.getCurrentState();
    planning_scene.checkCollision(collision_request, collision_result, robot_state, acm);
    ROS_INFO_STREAM("Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

    ros::Duration(0.05).sleep();
  }

  return 0;
}

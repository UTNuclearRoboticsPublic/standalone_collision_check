
// Refer to https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/pr2_tutorials/interactivity/src/collision_contact_tutorial.cpp

#include <standalone_collision_check.h>

int main(int argc, char** argv) {

  ros::init (argc, argv, "collision_check");
  ros::NodeHandle nh;

  // Set up the MoveIt scene
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);
  robot_state::RobotState robot_state = planning_scene.getCurrentState();
  collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();

  // Spawn a virtual collision object (for testing)
  //spawn_collision_cube(nh);

  // Spin while checking minimum collision distance
  while ( ros::ok() )
  {
    //process collision objects in scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    std::map<std::string, moveit_msgs::CollisionObject> c_objects_map = planning_scene_interface_.getObjects();
    for(auto& kv : c_objects_map){
      planning_scene.processCollisionObjectMsg(kv.second);
    }

  ROS_INFO("");
  ROS_INFO_STREAM("Distance to Collision: " << planning_scene.distanceToCollision(robot_state));
    ROS_INFO_STREAM("Is state colliding: " << planning_scene.isStateColliding());
  ROS_INFO("");

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
  primitive.dimensions[0] = .2;
  primitive.dimensions[1] = .2;
  primitive.dimensions[2] = .2;
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

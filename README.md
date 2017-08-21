# standalone_collision_check
Use MoveIt! to check the collision state of a given robot model.

# Usage
The robot driver must be running first (e.g. roslaunch ur_bringup ur5_bringup.launch robot_ip:=IP_OF_THE_ROBOT )
roslaunch standalone_collision_check standalone_collision_check.launch

Several parameters can be configured from the launch file:

kill_cmd: When a collision is detected, this command will be executed by a system() call. It can kill a process ("pkill -f -9 my_process") or a ros node ("rosnode kill my_node").

group_name: The MoveIt! group to monitor. Often "manipulator"

ur_topic_name: If using a Universal Robot, publish on this topic to halt robot motion.

test_with_cube: If true, spawn a virtual collision object for testing. Turn on the "Planning scene" plugin to see this cube in RViz.

test_with_random_joints: Continuously check the collision state with random joint values. This almost ensures a collision state will occur quickly, so it's a good way to test the collision logic.

# Tips
You can add "padding" around the robot by increasing the scale of the stl files in the URDF, then rerunning moveit_setup_assistant.

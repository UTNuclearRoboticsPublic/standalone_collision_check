# standalone_collision_check
Use MoveIt! to check the collision state of a given robot model.

# Usage
roslaunch standalone_collision_check standalone_collision_check.launch

The robot driver must be running first (e.g. roslaunch ur_bringup ur5_bringup.launch robot_ip:=IP_OF_THE_ROBOT )

Several parameters can be configured from the launch file:

kill_cmd: When a collision is detected, this command will be executed by a system() call. It can kill a process ("pkill -f -9 my_process") or a ros node ("rosnode kill my_node").

group_name: The MoveIt! group to monitor. Often "manipulator"

ur_topic_name: If using a Universal Robot, publish on this topic to halt robot motion.

test_with_cube: If true, spawn a virtual collision object for testing. Turn on the "Planning scene" plugin to see this cube in RViz.

test_with_random_joints: Continuously check the collision state with random joint values. This almost ensures a collision state will occur quickly, so it's a good way to test the collision logic.

# Usage with non-ROS processes
An environment variable is set when a collision is detected. Non-ROS processes can use as this as a flag. It is stored in /etc/environment. COLLISION_DETECTED=1  This new environment variable is held for 5s, then deleted to restore the environment variables to their original state.

Make sure you give the system permission to access this file/folder:
sudo chown username.users /etc/environment
sudo chown username.users /etc/

# Tips
You can add "padding" around the robot by increasing the scale of the stl files in the URDF, then rerunning moveit_setup_assistant.

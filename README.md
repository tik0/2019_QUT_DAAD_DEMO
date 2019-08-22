# 2019_QUT_DAAD_DEMO
First sampling of Rubik's Cube viewpoints in Gazebosim

Slides: https://docs.google.com/presentation/d/1RRx7ej7wjt436i5TSMu7O3fbJ1yzu5_S09al2V4BUIo/edit?usp=sharing
Video in Gazebo: https://youtu.be/LmE_yJIhcHM

![Panda with D435](/media/panda_d435.png)

## Sample Data with Free-Floating Camera

- Run tha data sampling without Panda robot (just a camera sweep in Gazebo) with `./catkin_ws/src/move_camera_trajectory/scripts//run.py` which produces bags, pngs, a lookup list for angles (`lookup_pitch_yaw.csv`), gif, and movie of the trajectory
 - NOTE: Gazebo restarts every yaw-sweep because of non-closed sockets
 - Images can be viewd via `rosrun rqt_image_view rqt_image_view`
 - Alter the cube pose by some PI rotation with `rosed rubiks_description cube.launch`
- Run the cube simulation standalone with `roslaunch rubiks_description cube.launch`

## Panda Setup

### Install and Run

- Precondition
 - Ubuntu 16.04.x with ROS-Kinteic
 - `apt-get install ros-kinetic-libfranka install ros-kinetic-moveit ros-kinetic-moveit-visual-tools`
 - Run `run.sh` in `patches`
- Use packages from https://erdalpekel.de/?p=55 which are already included as a submodule in this repository
 - https://github.com/erdalpekel/panda_simulation
 - https://github.com/erdalpekel/franka_ros
 - https://github.com/erdalpekel/panda_moveit_config/tree/melodic-devel (even if we use Kinteic)
- Run a simple example: `roslaunch panda_simulation simulation.launch gui_rviz:=True gui_gz:=False`
 - One might switch the gui for Gazebo and rviz on/off explitily using the example above
 - Hints for control in rviz
   - Click on `Reset` on the left-bottom to let the MoveIT! gui appear
   - move the endeffector to the desired position
   - Click on `Plan and Execute`

### Tutorial - Script a Pose or Trajectory

- Use the python tutorials http://docs.ros.org/indigo/api/moveit_tutorials/html/doc/pr2_tutorials/planning/src/doc/motion_planning_api_tutorial.html
- Follow the scripts in the tutorials: https://github.com/ros-planning/moveit_tutorials/tree/kinetic-devel/doc/move_group_python_interface
- I've forked the package in `move_group_python_interface` which is executable as follows:
 - Run the simulation: `roslaunch move_camera_trajectory simulation.launch gui_True:=False gui_gz:=False config_rviz:=$(rospack find move_camera_trajectory)/launch/panda_only.rviz`
 - Run the tutorial: `roslaunch move_group_python_interface move_group_python_interface_tutorial.launch`

### Use the camera sampling script

- Planning happens originally in the `panda_link8` frame wrt. the `world` which can be evaluated by
 - Bash: `bash> rosrun tf tf_echo world panda_link8` and see that the desired values match
 - Python and the Tutorial example from before: `move_group.get_planning_frame()` (http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html)
 - HOWEVER, we want to plan the `camera_color_optical_frame_ctrl` frame wrt. the reference frame `base_link_cube` to define a trajectory around Rubik's cube
- Collision objects will be loaded from `~/.panda_simulation` via `panda_simulation robot_control_node`
- Run simulator: `roslaunch move_camera_trajectory simulation_with_cube.launch gui_True:=False gui_gz:=False config_rviz:=$(rospack find move_camera_trajectory)/launch/panda_only.rviz`
- Run the trajectory planner: `roslaunch move_camera_trajectory move_camera_trajectory.launch`


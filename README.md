# 2019_QUT_DAAD_DEMO
First sampling of Rubik's Cube viewpoints in Gazebosim

- Run the cube simulation with `roslaunch rubiks_description cube.launch`
- Alter the cube pose with `rosed rubiks_description cube.launch`

## Panda Setup

### Install and Run

- Precondition
 - Ubuntu 18.04.x with ROS-Kinteic
 - `apt-get install ros-kinetic-libfranka install ros-kinetic-moveit ros-kinetic-moveit-visual-tools`
 - Run `run.sh` in `patches`
- Use packages from `https://erdalpekel.de/?p=55` which are already included as a submodule in this repository
 - `https://github.com/erdalpekel/panda_simulation`
 - `https://github.com/erdalpekel/franka_ros`
 - `https://github.com/erdalpekel/panda_moveit_config/tree/melodic-devel` (even if we use Kinteic)
- Run a simple example: `roslaunch panda_simulation simulation.launch gui_rviz:=True gui_gz:=False`
 - One might switch the gui for Gazebo and rviz on/off explitily using the example above
 - Hints for control in rviz
   - Click on `Reset` on the left-bottom to let the MoveIT! gui appear
   - move the endeffector to the desired position
   - Click on `Plan and Execute`

### Tutorial - Script a Pose or Trajectory

- Use the python tutorials `http://docs.ros.org/indigo/api/moveit_tutorials/html/doc/pr2_tutorials/planning/src/doc/motion_planning_api_tutorial.html`
- Follow the scripts in the tutorials: `https://github.com/ros-planning/moveit_tutorials/tree/kinetic-devel/doc/move_group_python_interface`
- I've forked the package in `move_group_python_interface` which is executable as follows:
 - Run the simulation: `roslaunch panda_simulation simulation.launch gui_rviz:=True gui_gz:=False`
 - Run the tutorial: `roslaunch move_group_python_interface move_group_python_interface_tutorial.launch`

### Use the camera sampling script

- Todo
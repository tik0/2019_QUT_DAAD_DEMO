cmake_minimum_required(VERSION 2.8.3)
project(move_camera_trajectory)

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
)

catkin_package()

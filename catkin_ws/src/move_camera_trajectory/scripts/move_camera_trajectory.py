#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import numpy as np
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from scipy.spatial.transform import Rotation as R

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveCameraTrajectory(object):
  """MoveCameraTrajectory"""
  def __init__(self, planning_frame = "", end_effector_link = ""):
    super(MoveCameraTrajectory, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)



    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    if planning_frame != "": # otherwise use the default one
        move_group.set_pose_reference_frame (planning_frame)
    planning_frame = move_group.get_pose_reference_frame()
    print "============ Planning frame: %s" % planning_frame

    # Set and get the planner (doesn't work: TBD)
    #move_group.set_planner_id("")
    #print "============ Planner Id: %s" % move_group.set_planner_id("")

    # We can also print the name of the end-effector link for this group:
    if end_effector_link != "":
        move_group.set_end_effector_link(end_effector_link)
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def go_to_pose_goal(self, pose_goal):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector
    move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

def main():
  try:
    print ""
    print "----------------------------------------------------------"
    print "Welcome to the camera trajectory executer"
    print "----------------------------------------------------------"
    print "Press Ctrl-D to exit at any time"
    print ""
    print "============ Begin the movements ..."
    mct = MoveCameraTrajectory("base_link_cube", "camera_color_optical_frame_ctrl")

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.orientation.x = 0.0
    pose_goal.orientation.y = 0.0
    pose_goal.orientation.z = 0.0
    # pose_goal.position.x = -0.5
    # pose_goal.position.y = 0.0
    # pose_goal.position.z = 0.4
    # mct.go_to_pose_goal(pose_goal)

    # Configure the parameters of the trajectory
    yaw_offset = np.pi
    z_offset = 0.0 # we assume the origin of the cube is centered in the cube
    #pitch_steps = 90 # in 1 deg steps
    #yaw_steps = 360 # in 1 deg steps
    #pitch_steps = 45 # in 2 deg steps
    #yaw_steps = 180 # in 2 deg steps
    pitch_steps = 9 # in 10 deg steps
    yaw_steps = 36 # in 10 deg steps
    # Radius of sphere on which the camera trajectory is defined
    radius = 0.25

    def pose(_pitch, _yaw):
        x = -np.cos(_pitch) * np.cos(_yaw) * radius
        y = np.cos(_pitch) * np.sin(_yaw) * radius
        z = z_offset + np.sin(_pitch) * radius
        roll = 0.0
        pitch = _pitch
        yaw = _yaw
        return x, y, z, roll, pitch, yaw

    # Tryout
    # r = R.from_euler('y', 90, degrees=True)
    # r *= R.from_euler('x', -10, degrees=True) # yaw
    # r *= R.from_euler('y', -10, degrees=True) # pitch
    # q = r.as_quat()
    # pose_goal.orientation.x = q[0]
    # pose_goal.orientation.y = q[1]
    # pose_goal.orientation.z = q[2]
    # pose_goal.orientation.w = q[3]
    # pose_goal.position.x = 0.
    # pose_goal.position.y = 0.
    # pose_goal.position.z = 0.
    # mct.go_to_pose_goal(pose_goal)
    # return


    for _pitch, _pitch_iter in zip(np.linspace(0.0, np.pi / 2.0, num=pitch_steps+1, endpoint=True), range(0, pitch_steps+1)):
        for _yaw, _yaw_iter in zip(np.linspace(np.pi / 2.0, -np.pi / 2.0, num=yaw_steps, endpoint=False), range(0, yaw_steps)):
            x, y, z, roll, pitch, yaw = pose(_pitch, _yaw)
            r = R.from_euler('y', 90, degrees=True) # aligne camera view axis on the link to the cube coordinate system
            r *= R.from_euler('x', yaw, degrees=False) # yaw
            r *= R.from_euler('y', pitch, degrees=False) # pitch
            q = r.as_quat()
            pose_goal.orientation.x = q[0]
            pose_goal.orientation.y = q[1]
            pose_goal.orientation.z = q[2]
            pose_goal.orientation.w = q[3]
            pose_goal.position.x = x
            pose_goal.position.y = y
            pose_goal.position.z = z
            mct.go_to_pose_goal(pose_goal)

            print('-------------------- pitch ' + str(_pitch_iter).zfill(3)  + ' yaw ' + str(_yaw_iter).zfill(3))


    #radius = .4
    # for angle_rad in np.linspace(0.0, np.pi, 10):
    #     print angle_rad
    #     pose_goal.position.y = radius * np.cos(angle_rad)
    #     pose_goal.position.x = radius * np.sin(angle_rad)
    #     mct.go_to_pose_goal(pose_goal)


    # print "============ Press `Enter` to plan and display a Cartesian path ..."
    # raw_input()
    # cartesian_plan, fraction = mct.plan_cartesian_path()

    # print "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
    # raw_input()
    # mct.display_trajectory(cartesian_plan)

    # print "============ Press `Enter` to execute a saved path ..."
    # raw_input()
    # mct.execute_plan(cartesian_plan)

    print "============ Complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

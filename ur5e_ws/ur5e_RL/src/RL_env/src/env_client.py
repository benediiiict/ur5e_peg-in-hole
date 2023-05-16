#!/usr/bin/env python3

import actionlib
import sys
import copy
import rospy
import tf
import numpy as np
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Vector3, Wrench, WrenchStamped
from geometry_msgs.msg import Point, Pose
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import String
from math import pi
from moveit_commander.conversions import pose_to_list
from cv_bridge import CvBridge

# from PIH_action_env import PIHAssembleEnv
# from action_move.srv import posx
# from action_move.msg import ArmPosition

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


class UrMoveClient(object):
  def __init__(self):
    super(UrMoveClient, self).__init__()
    # self.client = actionlib.SimpleActionClient('move_arm', Point)

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    # rospy.init_node('ur_moveEx', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    ## END_SUB_TUTORIAL
    ## BEGIN_SUB_TUTORIAL basic_info
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    # print("============ Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    # print ("============ End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    # print ("============ Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    # print ("============ Printing robot state")
    # print (move_group.get_current_pose().pose)
    # print ("")
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    # self.ft_sensor_array = []
    self.max_force = 0.0
    self.max_torque = 0.0

    # rospy.Subscriber('/ft_sensor_topic', WrenchStamped, self.force_return)


  # def force_return(self,data):
  #       # rospy.loginfo(data)
  #       self.adjust = np.array([data.wrench.force.x,\
  #                               data.wrench.force.y,\
  #                               data.wrench.force.z,\
  #                               data.wrench.torque.x,\
  #                               data.wrench.torque.y,\
  #                               data.wrench.torque.z])
  #       # self.adjust[2] -= 11.898
  #       self.ft_sensor_array = self.adjust  
  #       # print (self.ft_sensor_array) 
  
  # def get_force_data(self, force_array):
  #   force_x = force_array[0]
  #   force_y = force_array[1]
  #   force_z = force_array[2]
  #   self.force_feedback = np.array([force_x, force_y, force_z])
  #   return self.force_feedback

  
  def Cal_distance_to_target_and_Force_feedback(self, target):
    move_group = self.move_group
    current_pose = move_group.get_current_pose().pose
    # target = res from vision
    # target = np.array([0.5590, 0.1595, 0.2550])         # Actual Target = np.array([0.5490, 0.1595, 0.2550])
    # force = UrMoveClient.get_force_data(force_array = UrMoveClient().ft_sensor_array)
    
    DisToTarget_Force = np.array([target[0] - current_pose.position.x, 
                                  target[1] - current_pose.position.y, 
                                  target[2] - current_pose.position.z])
                                  # force[0], force[1], force[2] ])
    return DisToTarget_Force
  
  
  def catch_current_pose(self):
    move_group = self.move_group
    pose_now = move_group.get_current_pose().pose
    return pose_now


  def go_to_joint_state(self):
    move_group = self.move_group
    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0.0480
    joint_goal[1] = -1.3621
    joint_goal[2] = 1.67432
    joint_goal[3] = -1.881
    joint_goal[4] = -pi/2
    joint_goal[5] = -pi/2

    move_group.go(joint_goal, wait=True)
    move_group.stop()

    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def plan_cartesian_path(self, posx, posy, posz, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    
    # self.client.wait_for_server()

    move_group = self.move_group
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    waypoints = []
    wpose = move_group.get_current_pose().pose

    wpose.position.x += posx 
    wpose.position.y += posy   # Third move sideways (y)
    wpose.position.z += posz 

    wpose.orientation.x =0.999987923071728100
    wpose.orientation.y =9.314867899602954e-0500
    wpose.orientation.z = 0.00491375724784805400
    wpose.orientation.w =4.8707576392202635e-06

    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # self.client.send_goal(goal)
    # self.client.wait_for_result()

    move_group.execute(plan, wait=True)

    # print ("============ Printing robot state")
    # print (move_group.get_current_pose().pose)
    # print ("")

  def cancel_move(self):
    self.client.cancel_all_goals()


  def move_arm(self, posx, posy, posz):
    # print ("============ Start moveing UR5e by P2P ...")
    # UrMoveClient.go_to_pose_goal()

    # print ("========================================= Press `Enter` to plan and display a Cartesian path ...")
    UrMoveClient().plan_cartesian_path(posx, posy, posz)
    # UrMoveClient().get_force_data(force_array = UrMoveClient().ft_sensor_array)

    # cartesian_plan = UrMoveClient().plan_cartesian_path(posx, posy, posz)
    # UrMoveClient().execute_plan(cartesian_plan)

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    # return plan



  # def go_to_pose_goal(self, obs_pose):
  #   # Copy class variables to local variables to make the web tutorials more clear.
  #   # In practice, you should use the class variables directly unless you have a good
  #   # reason not to.
  #   move_group = self.move_group

  #   ## BEGIN_SUB_TUTORIAL plan_to_pose
  #   ##
  #   ## Planning to a Pose Goal
  #   ## ^^^^^^^^^^^^^^^^^^^^^^^
  #   ## We can plan a motion for this group to a desired pose for the
  #   ## end-effector:
  #   pose_goal = geometry_msgs.msg.Pose()
  #   # obs_pose = Point()

  #   pose_goal.orientation.w = 1.0
  #   pose_goal.position.x = obs_pose.x
  #   pose_goal.position.y = obs_pose.y
  #   pose_goal.position.z = obs_pose.z

  #   move_group.set_pose_target(pose_goal)

  #   ## Now, we call the planner to compute the plan and execute it.
  #   plan = move_group.go(wait=True)
  #   # Calling `stop()` ensures that there is no residual movement
  #   move_group.stop()
  #   # It is always good to clear your targets after planning with poses.
  #   # Note: there is no equivalent function for clear_joint_value_targets()
  #   move_group.clear_pose_targets()

  #   ## END_SUB_TUTORIAL

  #   # For testing:
  #   # Note that since this section of code will not be included in the tutorials
  #   # we use the class variable rather than the copied state variable
  #   current_pose = self.move_group.get_current_pose().pose
  #   return all_close(pose_goal, current_pose, 0.01)
    
    


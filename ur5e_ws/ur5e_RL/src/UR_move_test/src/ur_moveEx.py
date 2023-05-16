#!/usr/bin/python3

import sys
import copy
import rospy
import numpy as np
from numpy import random
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg 
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Wrench, WrenchStamped
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import String
from math import pi
from moveit_commander.conversions import pose_to_list



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


def talker():
  pub = rospy.Publisher('joint_states', JointState, queue_size=10)
  rospy.init_node('joint_state', anonymous=True)
  print("Start")
  rate = rospy.Rate(10) # 10hz
  hello_str = JointState()
  hello_str.header = Header()
  hello_str.header.stamp = rospy.Time.now()
  hello_str.header.frame_id = ""
  hello_str.name = ["elbow_joint", "finger_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
  # hello_str.name = ['- elbow_joint  - finger_joint  - shoulder_lift_joint  - shoulder_pan_joint  - wrist_1_joint  - wrist_2_joint  - wrist_3_joint']
  hello_str.position = [1.6822342591438515, -0.01273020127900315, -1.3596906174439463, 0.04793161043986327, -1.8796019214940056, -1.570848232977486, -1.5701671469420075]
  hello_str.velocity = [0.04883706950570791, 0.036701998636350375, -0.07785393911099522, -0.018331844076873563, -0.05917739380140098, -0.13214819583632498, 1.2310764145268183]
  hello_str.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  
  pub.publish(hello_str)
  print(hello_str)
  
  while not rospy.is_shutdown():
    hello_str.header.stamp = rospy.Time.now()
    pub.publish(hello_str)
    rate.sleep()


def Gauss():
  x = random.normal(loc=0, scale=0.05, size=(1, 3))
  print(x)
  return 0


class UrMoveEx(object):
  def __init__(self):
    super(UrMoveEx, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur_moveEx', anonymous=True)

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
    group_name = "manipulator"       # ur5e_arm
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
    print("============ Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print ("============ End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print ("============ Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print ("============ Printing robot state")
    # print (move_group.get_current_pose().pose)
    print ("")
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
    self.ft_sensor_array = []
    self.max_force = 0.0
    self.max_torque = 0.0

    rospy.Subscriber('/ft_sensor_topic', WrenchStamped, self.force_return)

  def go_to_joint_state(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group
    print (move_group.get_current_pose().pose)

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = move_group.get_current_joint_values()
    # print (joint_goal)
    joint_goal[0] = 0
    joint_goal[1] = -pi/2
    joint_goal[2] = pi/2
    joint_goal[3] = -pi/2
    joint_goal[4] = -pi/2
    joint_goal[5] = -pi/2

    # joint_goal[1] = 0
    # joint_goal[2] = 0
    # joint_goal[3] = -pi/2
    # joint_goal[4] = 0
    # joint_goal[5] = 0

    # joint_goal[1] = -pi/3
    # joint_goal[2] = pi/1.7
    # joint_goal[3] = -pi/1.33
    # joint_goal[4] = -pi/2
    # joint_goal[5] = -pi/2

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    current_joints = move_group.get_current_joint_values()
    # print (move_group.get_current_pose().pose)
    return all_close(joint_goal, current_joints, 0.01)
  
  def go_to_joint_state_ready(self):
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
    print (move_group.get_current_pose().pose)
    print(current_joints)
    return all_close(joint_goal, current_joints, 0.01)
  
  def go_to_joint_state_take_picture(self):
    move_group = self.move_group
    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = -0.5
    joint_goal[1] = -pi/2
    joint_goal[2] = pi/2
    joint_goal[3] = -pi/2
    joint_goal[4] = -pi/2
    joint_goal[5] = -pi/2

    move_group.go(joint_goal, wait=True)
    move_group.stop()

    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_pose_goal(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 7.20
    pose_goal.position.x = 0.5545  #get curren = 0.5520         #0.5523
    pose_goal.position.y = 0.1595           #0.1600
    pose_goal.position.z = 0.2729           #0.3529
    pose_goal.orientation.x =0.999987923071728100
    pose_goal.orientation.y =9.314867899602954e-0500
    pose_goal.orientation.z = 0.00491375724784805400
    pose_goal.orientation.w =4.8707576392202635e-06

    move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    print (move_group.get_current_pose().pose)
    return all_close(pose_goal, current_pose, 0.01)

  def plan_cartesian_path(self, posx, posy, posz, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through. If executing  interactively in a
    ## Python shell, set scale = 1.0.
    waypoints = []
    wpose = move_group.get_current_pose().pose

    # wpose.position.z -= scale * 0.1  # First move up (z)
    # wpose.position.y += scale * 0.2  # and sideways (y)
    # waypoints.append(copy.deepcopy(wpose))
    # wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    # waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += posx 
    wpose.position.y += posy 
    wpose.position.z += posz 

    wpose.orientation.x =0.999987923071728100
    wpose.orientation.y =9.314867899602954e-0500
    wpose.orientation.z = 0.00491375724784805400
    wpose.orientation.w =4.8707576392202635e-06

    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
    
    move_group.execute(plan, wait=True)
    # print (move_group.get_current_pose().pose)

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    # return plan

  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    move_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail

  def get_force_data(self, force_array):
    force_x = force_array[0]
    force_y = force_array[1]
    force_z = force_array[2]
    self.force_feedback = np.array([force_x, force_y, force_z])
    print("force x, y, z ===", self.force_feedback)


  def force_return(self,data):
    # rospy.loginfo(data)
    self.adjust = np.array([data.wrench.force.x,\
                            data.wrench.force.y,\
                            data.wrench.force.z,\
                            data.wrench.torque.x,\
                            data.wrench.torque.y,\
                            data.wrench.torque.z])
    # self.adjust[2] -= 11.898
    self.ft_sensor_array = self.adjust  
    # print (self.ft_sensor_array)
    # return self.ft_sensor_array
         

def main():
  try:
    # talker()
    # Gauss()

    print ("============ Press `Enter` to begin the tutorial by setting up the moveit_commander ...")
    tutorial = UrMoveEx()
    
    print ("============ Press `Enter` to execute a movement using a joint state goal ...")
    tutorial.go_to_joint_state()
    tutorial.go_to_joint_state_ready()
    # tutorial.go_to_joint_state_take_picture()
    # tutorial.go_to_pose_goal()


    # for i in range (8): # 8  # 10 collision
    #   print ("------------- Press `Enter` to plan and display a Cartesian path...")
    #   tutorial.plan_cartesian_path(posx = 0.00308, posy = 0.00001, posz = -0.00106)
      # tutorial.plan_cartesian_path(posx = 0.00208, posy = 0.00001, posz = -0.00206)            # insert hole

    #   tutorial.get_force_data(force_array = tutorial.ft_sensor_array)
   
    # for i in range (9): # 8
    #   print ("============ Press `Enter` to plan and display a Cartesian path...")
    #   tutorial.plan_cartesian_path(posx = 0.0, posy = 0.0, posz = -0.000003006)
      
    
    # tutorial.execute_plan(cartesian_plan)

    # print ("============ Press `Enter` to execute a movement using a pose goal ...")
    # tutorial.go_to_pose_goal()

    # print ("============ Press `Enter` to plan and display a Cartesian path2...")
    # cartesian_plan = tutorial.plan_cartesian_path(posx = -0.25008, posy = -0.18024, posz = 0.15106)

    # print ("============ Press `Enter` to execute a saved path2...")
    # tutorial.execute_plan(cartesian_plan)	
    print("============ Python tutorial demo complete!")
    
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return


if __name__ == '__main__':
  main()


##########################################################################################################
# def generate_points(self, roll_detla, pitch_delta, yaw_delta, x_distance, y_distance, z_distance):

# 		print(x_distance)
# 		# input()

# 		try:
# 			(tool_trans_base, tool_rot_base) = self.tf_listener.lookupTransform(self.tip_link, self.base_link, rospy.Time(0))
# 		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
# 			rospy.logwarn('lookupTransform for robot failed!, ' + self.tip_link + ', ' + self.base_link)
# 			return

# 		# tool_rotation = list(euler_from_quaternion([tool_rot_base[0],tool_rot_base[1],tool_rot_base[2],tool_rot_base[3]]))

# 		# quaternion =  quaternion_from_euler(tool_rotation[0]+np.radians(roll_detla),
# 		# 									tool_rotation[1]+np.radians(pitch_delta), 
# 		# 									tool_rotation[2]+np.radians(yaw_delta))  	
# 		tool_base_m = self.tf_listener.fromTranslationRotation((tool_trans_base[0]+x_distance,
# 																tool_trans_base[1]+y_distance,
# 																tool_trans_base[2]+z_distance,)
# 																,(tool_rot_base[0],
# 																tool_rot_base[1],
# 																tool_rot_base[2],
# 																tool_rot_base[3]))

# 		base_move_delta = np.linalg.inv(tool_base_m)

# 		delta_pos = np.array(base_move_delta[0:3, 3]).reshape(-1)


# 		delta_pos[0] = (self.wpose.position.x - delta_pos[0])
# 		delta_pos[1] = (self.wpose.position.y - delta_pos[1])
# 		delta_pos[2] = (self.wpose.position.z - delta_pos[2])

# 		self.wpose.position.x += delta_pos[0]
# 		self.wpose.position.y += delta_pos[1]
# 		self.wpose.position.z += delta_pos[2]

# 		# print("delta_pos",delta_pos)
		
# 		# return delta_pos
# 		return


  # def display_trajectory(self, plan):
  #   # Copy class variables to local variables to make the web tutorials more clear.
  #   # In practice, you should use the class variables directly unless you have a good
  #   # reason not to.
  #   robot = self.robot
  #   display_trajectory_publisher = self.display_trajectory_publisher

  #   ## Displaying a Trajectory
  #   ## ^^^^^^^^^^^^^^^^^^^^^^^
  #   ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
  #   ## group.plan() method does this automatically so this is not that useful
  #   ## here (it just displays the same trajectory again):
  #   ##
  #   ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
  #   ## We populate the trajectory_start with our current robot state to copy over
  #   ## any AttachedCollisionObjects and add our plan to the trajectory.
  #   display_trajectory = moveit_msgs.msg.DisplayTrajectory()
  #   display_trajectory.trajectory_start = robot.get_current_state()
  #   display_trajectory.trajectory.append(plan)
  #   # Publish
  #   display_trajectory_publisher.publish(display_trajectory)


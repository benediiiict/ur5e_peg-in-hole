#!/usr/bin/env python3

import numpy as np
import gym
import os
import rospy
import RL_env
import math
# import actionlib
# import torch
# from stable_baselines3.common.policies import MlpPolicy
# from stable_baselines3.common import make_vec_env

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Point, Pose
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import String
from math import pi
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Vector3, Wrench, WrenchStamped
from gym import spaces
from stable_baselines3 import PPO, SAC, TD3
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import DummyVecEnv

from env_client import UrMoveClient

class PIHAssembleEnv(gym.Env):
  """
  Custom Environment that follows gym interface.
  This is a simple env where the agent must learn to go always left. 
  """
  metadata = {'render.modes': ['human']}
  # Define constants for clearer code
  CURRENT = np.array([0.4915, 0.13316, 0.48323])
  TARGET = np.array([0.5822, 0.13316, 0.24460])

  def __init__(self):
    super(PIHAssembleEnv, self).__init__()

    self.robot = UrMoveClient()
    self.action_dim = 3
    self.obs_dim = 9
    self.count = 0
    self.target = np.array([0.5525, 0.1595, 0.2550]) 
    self.pose_last = []
    self.distance = []
    self.ft_sensor_array = []
    self.force = []
    self.force_again = []
    self.hole = []
    self.max_force = 0.0
    self.max_torque = 0.0

    high = np.ones([self.action_dim])
    self.action_space = spaces.Box(np.float32(-high), np.float32(high))
    # self.action_space = spaces.Box(-high, high, dtype=np.float32)
    high = np.inf*np.ones([self.obs_dim])
    self.observation_space = spaces.Box(np.float32(-high), np.float32(high))
    # self.observation_space = spaces.Box(-high, high, dtype=np.float32)

    rospy.Subscriber('/ft_sensor_topic', WrenchStamped, self.force_return)    # force_feedback

########################################### User-Defined Function ################################################

  def action_normalization(self, action):
    action[0] = action[0] * 0.003 + 0.0008
    action[1] = action[1] * 0.00001
    action[2] = action[2] * 0.001
    return action
  
  def force_feedback(self,data):
        # rospy.loginfo(data)
        self.adjust = np.array([data.wrench.force.x,\
                                data.wrench.force.y,\
                                data.wrench.force.z,\
                                data.wrench.torque.x,\
                                data.wrench.torque.y,\
                                data.wrench.torque.z])
        # self.adjust[2] -= 11.898
        self.ft_sensor_array = self.adjust  

  def force_return(self,data):
    self.force_data = []
    self.force_data.append(data.wrench.force.x)
    self.force_data.append(data.wrench.force.y)
    self.force_data.append(data.wrench.force.z)
    self.ft_sensor_array = self.force_data
    # print(self.force_data)
    # return self.ft_sensor_array
  
  def get_force_data(self, force_array):
    force_x = force_array[0]
    force_y = force_array[1]
    force_z = force_array[2]
    self.forceFB = np.array([force_x, force_y, force_z])
    return self.forceFB
  
  def Dis_plus_ForceFeedback(self, dis, pose_now, ft_x, ft_y, ft_z):
    self.obs = np.array([dis[0], dis[1], dis[2], pose_now.position.x, pose_now.position.y, pose_now.position.z, ft_x, ft_y, ft_z])      # self.ft_sensor_array[0], self.ft_sensor_array[1], self.ft_sensor_array[2]
    return self.obs
  
  def cal_reward_dis(self, dis):
    distance_r = math.sqrt(pow(dis.position.x - self.target[0], 2) + pow(dis.position.y - self.target[1], 2) + pow(dis.position.z - self.target[2], 2))
    return distance_r
  

########################################### Stablebaseline3 ##################################################

  def reset(self):
    """
    Important: the observation must be a numpy array
    :return: (np.array) 
    """
    # Initialize the agent
    # initPoint = Point()
    # self.obs = initPoint
    # self.robot.plan_cartesian_path = self.obs
    self.robot.go_to_joint_state()   
    self.robot.go_to_joint_state()
    self.distance = self.robot.Cal_distance_to_target_and_Force_feedback(self.target)
    self.pose_last = self.robot.catch_current_pose()
    self.observation = self.Dis_plus_ForceFeedback(self.distance, self.pose_last, self.ft_sensor_array[0], self.ft_sensor_array[1], self.ft_sensor_array[2])
    # print(self.target)

    # here we convert to float32 to make it more general (in case we want to use continuous actions)
    # return np.array([self.obs]).astype(np.float32)
    return np.array([self.observation]).astype(np.float32)
  

  def step(self, action):
    # err_msg = f"{action!r} ({type(action)}) invalid"
    # assert self.action_space.contains(action), err_msg
    print("=============================one step================================")
    done = False
    reward = 0
    force_flag = 0
    self.count += 1
    self.action = self.action_normalization(action)
    print(self.action)
    print("============================= action ================================ ")

    self.robot.move_arm(self.action[0], self.action[1], self.action[2])
    self.pose = self.robot.catch_current_pose()
    self.force = self.ft_sensor_array

    # self.reward_distance = self.cal_reward_dis(self.pose)
    # self.reward_distance_last = self.cal_reward_dis(self.pose_last)

    # self.force_again = self.ft_sensor_array
    # print("--------------------------- pose_last ======================== \n", self.pose_last)
    # print("--------------------------- pose_now  ======================= \n", self.pose)
    # print("--------------------------- force_now ========================= \n", self.force)
    # print("--------------------------- force_again_now ========================= \n", self.force_again)

    # random point             target = [0.5525, 0.1595, 0.2550]             Actual Target = [0.5490, 0.1595, 0.2550]
    if abs(self.pose.position.x - self.target[0]) < abs(self.pose_last.position.x - self.target[0]) :
      reward += abs(self.pose.position.x - self.pose_last.position.x) * 10
    elif abs(self.pose.position.x - self.target[0]) > abs(self.pose_last.position.x - self.target[0]) :
      reward -= abs(self.pose.position.x - self.pose_last.position.x) * 10
    elif self.pose.position.x > (self.target[0] + 0.012):
      reward -= 3
      done = True
    elif self.pose.position.x < (self.target[0] - 0.010):
      reward -= 3
      done = True

    # if abs(self.pose.position.y - self.target[1]) < abs(self.pose_last.position.y - self.target[1]) :
    #   reward += 0.5
    # elif abs(self.pose.position.y - self.target[1]) > abs(self.pose_last.position.y - self.target[1]) :
    #   reward -= abs(self.pose.position.y - self.target[1])
    # elif self.pose.position.y < (self.target[1] - 0.0005) or self.pose.position.y > (self.target[1] + 0.0015):
    #   reward -= 2
    if self.pose.position.y < (self.target[1] - 0.0005) or self.pose.position.y > (self.target[1] + 0.0015):
      reward -= 2

    if abs(self.pose.position.z - self.target[2]) < abs(self.pose_last.position.z - self.target[2]) :
      reward += abs(self.pose.position.z - self.pose_last.position.z) * 10
    elif abs(self.pose.position.z - self.target[2]) > abs(self.pose_last.position.z - self.target[2]) :
      reward -= abs(self.pose.position.z - self.pose_last.position.z) * 10
    elif self.pose.position.z < (self.target[2] + 0.020):
      reward -= 3
      done = True
    elif self.pose.position.z > (self.target[2] + 0.125):
      reward -= 2
    



    # if self.pose.position.x > (self.target[0] + 0.001):
    #   reward += 1.0
    #   if self.pose.position.x > (self.target[0] + 0.006):
    #     reward -= 3
    #     done = True
    # elif (self.target[0] + 0.001) > self.pose.position.x > (self.target[0] - 0.002):
    #   reward += 0.8
    # elif (self.target[0] - 0.002) > self.pose.position.x > (self.target[0] - 0.004):
    #   reward += 0.5
    # elif (self.target[0] - 0.004) > self.pose.position.x > (self.target[0] - 0.009):
    #   reward += 0.2
    # elif self.pose.position.x < (self.target[0] - 0.014):
    #   reward -= 1
    # elif self.pose.position.x < (self.target[0] - 0.016):
    #   reward -= 3
    #   done = True

    # if (self.target[1]) < self.pose.position.y < (self.target[1] + 0.0005):
    #   reward += 0.5
    #   if self.pose.position.y < (self.target[1] - 0.0005):
    #     reward -= 2
    # elif (self.target[1] + 0.0005) < self.pose.position.y < (self.target[1] + 0.001):
    #   reward += 0.3
    # elif (self.target[1] + 0.0010) < self.pose.position.y < (self.target[1] + 0.0015):
    #   reward += 0.1
    # elif self.pose.position.y > (self.target[1] + 0.0015):
    #   reward -= 2

    # if self.pose.position.z < (self.target[2] + 0.025):
    #   reward += 1.0
    #   if self.pose.position.z < (self.target[2] + 0.020):
    #     reward -= 2
    #     done = True
    # elif (self.target[2] + 0.025) < self.pose.position.z < (self.target[2] + 0.045):
    #   reward += 0.8
    # elif (self.target[2] + 0.045) < self.pose.position.z < (self.target[2] + 0.065):
    #   reward += 0.6
    # elif (self.target[2] + 0.065) < self.pose.position.z < (self.target[2] + 0.085):
    #   reward += 0.4
    # elif (self.target[2] + 0.085) < self.pose.position.z < (self.target[2] + 0.105):
    #   reward += 0.2
    # elif self.pose.position.z > (self.target[2] + 0.125):
    #   reward -= 2



    # if  self.reward_distance < self.reward_distance_last :
    #   reward += (self.reward_distance_last - self.reward_distance) * 2
    # elif self.reward_distance > self.reward_distance_last :
    #   reward -= (self.reward_distance - self.reward_distance_last)
    # elif self.pose.position.x > (self.target[0] + 0.006):
    #   reward -= 3
    #   done = True
    # elif self.pose.position.x < (self.target[0] - 0.016):
    #   reward -= 3
    #   done = True
    # elif self.pose.position.y < (self.target[1] - 0.0005):
    #   reward -= 2
    # elif self.pose.position.y > (self.target[1] + 0.0015):
    #   reward -= 2
    # elif self.pose.position.z < (self.target[2] + 0.018):
    #   reward -= 2
    #   done = True
    # elif self.pose.position.z > (self.target[2] + 0.125):
    #   reward -= 2



    if self.force[0] > 15 or self.force[0] < -10 :
      print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! break !!!!!!!!!!!!!!!!!!!!!!!!!!!!")
      reward -= 3
      done = True
    if self.force[1] > 20 or self.force[1] < -20 :
      reward -= 3
      done = True
    if self.force[2] > 15 or self.force[2] < -15 :
      reward -= 3
      done = True

    # if self.force_again[0] > 10 or self.force_again[0] < -10 :
    #   print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! break !!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    #   reward -= 3
    #   force_flag = 1
    #   self.reset()
    # if self.force_again[1] > 20 or self.force_again[1] < -20 :
    #   reward -= 3
    #   force_flag = 1
    #   done = True
    # if self.force_again[2] > 10 or self.force_again[2] < -10 :
    #   reward -= 3
    #   force_flag = 1
    #   done = True
    
    if 0.554 > self.pose.position.x > 0.550 and 0.1600 > self.pose.position.y > 0.1590 and 0.2780 > self.pose.position.z > 0.2700:
      reward += 3
      done = True
      print("doooooooooooooooooooooooooooooooooooooooone!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
      

    # if reward < -1:
    #   done = True

    # else:
      # raise ValueError("Received invalid action={} which is not part of the action space".format(action))
    
    self.distance = self.robot.Cal_distance_to_target_and_Force_feedback(self.target)
    # if force_flag == 0:
    self.observation = self.Dis_plus_ForceFeedback(self.distance, self.pose, self.force[0], self.force[1], self.force[2])
    # elif force_flag == 1:
    #   self.observation = self.Dis_plus_ForceFeedback(self.distance, self.force_again[0], self.force_again[1], self.force_again[2])
    # print(self.observation)
    
    self.pose_last = self.pose
    # Optionally we can pass additional info, we are not using that for now
    info = {}
    print("epsoide",self.count,"   Reward:" + str(reward))
    # return np.array([self.agent_pos]).astype(np.float32), reward, done, info
    return np.array([self.observation]).astype(np.float32),  reward, done, info


  def render(self, mode='human'):
    if mode != 'human':
      raise NotImplementedError()
    pass

  def close(self):
    pass


# rospy.init_node("train_ppo2", anonymous=True, log_level=rospy.INFO)
# env_id = "ur5e-v0"

# log_dir = "/home/ben/work/ur5e_RL/src/RL_env/results/"+env_id+"/"
# os.makedirs(log_dir, exist_ok=True)

# env = gym.make(env_id)
# # env = make_vec_env(env_id, n_envs=4)
# env = Monitor(env, filename=log_dir, allow_early_resets=True)
# # env = PIHAssembleEnv()
# env = DummyVecEnv([lambda: env])

# ############ Train ############
# env.reset()
# # model = PPO("MlpPolicy", env, n_steps=1024, verbose=1)
# # model.learn(total_timesteps=20000)
# model = PPO("MlpPolicy", env, n_steps=2048, verbose=1)
# model.learn(total_timesteps=60000)
# model.save(log_dir+"ppo_05_")

# ########### Predict ############
# # model = PPO.load("ppo2_UR5e_done")

# # obs = env.reset()
# # for i in range(25000):
# #     action, _states = model.predict(obs)
# #     obs, rewards, dones, info = env.step(action)

# env.close()



    # if self.pose.position.x > 0.56:
    #   reward += 1.0
    #   if self.pose.position.x > 0.565:
    #     reward -= 3
    #     done = True
    # elif 0.560 > self.pose.position.x > 0.557:
    #   reward += 0.8
    # elif 0.557 > self.pose.position.x > 0.555:
    #   reward += 0.5
    # elif 0.555 > self.pose.position.x > 0.55:
    #   reward += 0.2
    # elif self.pose.position.x < 0.545:
    #   reward -= 1
    # elif self.pose.position.x < 0.543:
    #   reward -= 3
    #   done = True
    
    # if 0.1595 < self.pose.position.y < 0.1600:
    #   reward += 0.5
    #   if self.pose.position.y < 0.1590:
    #     reward -= 2
    # elif 0.1600 < self.pose.position.y < 0.1605:
    #   reward += 0.3
    # elif 0.1605 < self.pose.position.y < 0.1610:
    #   reward += 0.1
    # elif self.pose.position.y > 0.1610:
    #   reward -= 2

    # if self.pose.position.z < 0.2800:
    #   reward += 1.0
    #   if self.pose.position.z < 0.2700:
    #     reward -= 2
    #     done = True
    # elif 0.2800 < self.pose.position.z < 0.300:
    #   reward += 0.8
    # elif 0.3000 < self.pose.position.z < 0.320:
    #   reward += 0.6
    # elif 0.320 < self.pose.position.z < 0.340:
    #   reward += 0.4
    # elif 0.340 < self.pose.position.z < 0.360:
    #   reward += 0.2
    # elif self.pose.position.z > 0.380:
    #   reward -= 2
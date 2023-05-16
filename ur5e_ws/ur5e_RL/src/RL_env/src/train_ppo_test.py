#!/usr/bin/env python3

import numpy as np
import gym
import rospy
import RL_env

import numpy as np
import gym
import os
import rospy
import RL_env
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


rospy.init_node("train_ppo", anonymous=True, log_level=rospy.INFO)
env_id = "ur5e-v0"

log_dir = "/home/ben/work/ur5e_RL/src/RL_env/results/"+env_id+"/"
os.makedirs(log_dir, exist_ok=True)

env = gym.make(env_id)
# env = make_vec_env(env_id, n_envs=4)
env = Monitor(env, filename=log_dir, allow_early_resets=True)
# env = PIHAssembleEnv()
env = DummyVecEnv([lambda: env])

############ Train ############
env.reset()
model = PPO("MlpPolicy", env, n_steps=1024, verbose=1)
model.learn(total_timesteps=20000)
# model = PPO("MlpPolicy", env, n_steps=2048, verbose=1)
# model.learn(total_timesteps=150000)
model.save(log_dir+"ppo_05_")

########### Predict ############
# model = PPO.load("ppo2_UR5e_done")

# obs = env.reset()
# for i in range(25000):
#     action, _states = model.predict(obs)
#     obs, rewards, dones, info = env.step(action)

env.close()


# TD3 train
# The noise objects for TD3
# n_actions = env.action_space.shape[-1]
# action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))
# model = TD3("MlpPolicy", env, action_noise=action_noise, verbose=1)
# model.learn(total_timesteps=10000, log_interval=10)


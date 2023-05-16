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
from stable_baselines3.common.evaluation import evaluate_policy

rospy.init_node("predict_ppo2", anonymous=True, log_level=rospy.INFO)

env_id = "ur5e-v0"
log_dir = "/home/ben/work/ur5e_RL/src/RL_env/results/"+env_id+"/"
os.makedirs(log_dir, exist_ok=True)

env = gym.make(env_id)
env = Monitor(env, filename=log_dir, allow_early_resets=True)
env = DummyVecEnv([lambda: env])

model = PPO.load("/home/ben/work/ur5e_RL/src/RL_env/results/ur5e-v0/ppo_0510_150000_holem_goood.zip")

obs = env.reset()

# 譽為的 "只跑一次 碰到就結束"
# mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=1)

while True:
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)



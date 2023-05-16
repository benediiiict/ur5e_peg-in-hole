from gym.envs.registration import register


# environment to move the real arm
register(
    id="ur5e-v0",
    entry_point="RL_env.src.PIH_action_env:PIHAssembleEnv",
    max_episode_steps=200
)


#register(
#    id="ur5eTest-v0",
#    entry_point="RL_env.src.train_ppo_test:TestPIHEnv",
#    max_episode_steps=200
#)



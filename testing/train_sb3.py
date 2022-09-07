from stable_baselines3 import PPO, DQN
from simulation1 import CartPoleEnv

from stable_baselines3.common.env_checker import check_env

# Instantiate the env
env = CartPoleEnv()
# Define and Train the agent
model = PPO("MlpPolicy", env, verbose=1)

# model = PPO("MlpPolicy", env, learning_rate=0.0003, n_steps=1024, batch_size=64, 
# n_epochs=10, gamma=0.99, gae_lambda=0.95, clip_range=0.2, clip_range_vf=None, 
# normalize_advantage=True, ent_coef=0.0, vf_coef=0.5, max_grad_norm=0.5, 
# use_sde=False, sde_sample_freq=-1, target_kl=None, tensorboard_log=None, 
# create_eval_env=False, policy_kwargs=None, verbose=1, seed=None, 
# device='auto', _init_setup_model=True)

# model= DQN("MlpPolicy", env, learning_rate=0.001, buffer_size=50000, 
# learning_starts=10000, batch_size=64, tau=1, gamma=0.99, 
# train_freq=10, gradient_steps=1, optimize_memory_usage=False, 
# target_update_interval=10000, exploration_fraction=0.5, 
# exploration_initial_eps=1, exploration_final_eps=0.15, verbose=1)
model.learn(total_timesteps=30000)

obs = env.reset()
for _ in range(10000):
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    if dones:
       env.reset()
    env.render()

# import gym
# import numpy as np

# from stable_baselines3 import PPO
# from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
# from stable_baselines3.common.env_util import make_vec_env
# from stable_baselines3.common.utils import set_random_seed

# def make_env(env_id, rank, seed=0):
#     """
#     Utility function for multiprocessed env.

#     :param env_id: (str) the environment ID
#     :param num_env: (int) the number of environments you wish to have in subprocesses
#     :param seed: (int) the inital seed for RNG
#     :param rank: (int) index of the subprocess
#     """
#     def _init():
#         env = gym.make(env_id)
#         env.seed(seed + rank)
#         return env
#     set_random_seed(seed)
#     return _init

# if __name__ == '__main__':
#     env_id = "CartPole-v1"
#     num_cpu = 4  # Number of processes to use
#     # Create the vectorized environment
#     env = SubprocVecEnv([make_env(env_id, i) for i in range(num_cpu)])

#     # Stable Baselines provides you with make_vec_env() helper
#     # which does exactly the previous steps for you.
#     # You can choose between `DummyVecEnv` (usually faster) and `SubprocVecEnv`
#     # env = make_vec_env(env_id, n_envs=num_cpu, seed=0, vec_env_cls=SubprocVecEnv)

#     model = PPO('MlpPolicy', env, verbose=1)
#     model.learn(total_timesteps=25_000)

#     obs = env.reset()
#     for _ in range(1000):
#         action, _states = model.predict(obs)
#         obs, rewards, dones, info = env.step(action)
#         env.render()

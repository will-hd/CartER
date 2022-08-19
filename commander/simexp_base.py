import logging
from enum import Enum, unique
from typing import Any, Type, Union

import numpy as np
import stable_baselines3
from stable_baselines3 import PPO, DQN

import click
import matplotlib.pyplot as plt
import stable_baselines3
from ml.environment import ExperimentalCartpoleEnv
from log import setup_logging

@unique
class SimulationExperimentCommand(str, Enum):
    EXPERIMENT = "experiment"

setup_logging(debug=False, file=True)


env = ExperimentalCartpoleEnv()


# from stable_baselines3.common.env_checker import check_env

# # It will check your custom environment and output additional warnings if needed
# check_env(ExperimentalCartpoleEnv())

# model = PPO("MlpPolicy", env, learning_rate=0.00003, n_steps=1024, batch_size=64, 
# n_epochs=10, gamma=0.99, gae_lambda=0.95, clip_range=0.2, normalize_advantage=True, 
# ent_coef=0.0, vf_coef=0.5, max_grad_norm=0.5, use_sde=False, sde_sample_freq=-1,
# verbose=3, seed=None, 
# device='auto', _init_setup_model=True)
model = DQN("MlpPolicy", env, learning_rate= 0.0001, buffer_size=1000000, learning_starts=10000, batch_size=32,verbose=2)
model.learn(total_timesteps=100000)
from time import time

import numpy as np

from env import RealPendulumEnv
from wrappers import HistoryWrapper

env = RealPendulumEnv(port="/dev/ttyUSB0", baudrate=19200)
env = HistoryWrapper(env, steps=3)

# test policy
obs = env.reset()
for _ in range(100_000):
    start_time = time()
    action = [np.random.uniform(-1, 1)]
    obs, reward, done, truncated, info = env.step(action)
    print(f"Obs: {obs}, Reward: {reward}, Done: {done}, Truncated: {truncated}, Info: {info}")
    if done or truncated:
        obs = env.reset()
    print(f"Step took {time() - start_time:.4f} seconds")
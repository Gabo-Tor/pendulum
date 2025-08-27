from typing import Optional

import gymnasium as gym
import numpy as np
from gymnasium.spaces import Box


class HistoryWrapper(gym.Wrapper):
    """
    Track history of actions and observations for a given number of steps.

    - Initial actions are zeros.
    - Initial observations repeat the first observed state.
    """

    def __init__(self, env: gym.Env, steps: int):
        super().__init__(env)
        assert steps > 0, "steps must be > 0"
        self.steps = steps

        # Ensure obs/action are 1D arrays
        obs_low = np.array(self.observation_space.low, dtype=np.float32).ravel()
        obs_high = np.array(self.observation_space.high, dtype=np.float32).ravel()
        action_low = np.array(self.action_space.low, dtype=np.float32).ravel()
        action_high = np.array(self.action_space.high, dtype=np.float32).ravel()

        # History = obs_history + action_history
        obs_low_hist = np.tile(obs_low, (self.steps,))
        obs_high_hist = np.tile(obs_high, (self.steps,))
        action_low_hist = np.tile(action_low, (self.steps,))
        action_high_hist = np.tile(action_high, (self.steps,))

        self.observation_space = Box(
            low=np.concatenate([obs_low_hist, action_low_hist]),
            high=np.concatenate([obs_high_hist, action_high_hist]),
            dtype=np.float32,
        )

        self.obs_history = None
        self.act_history = None

    def _make_action_history(self):
        """Initialize with zero actions."""
        return [np.zeros(self.action_space.shape, dtype=np.float32) for _ in range(self.steps)]

    def _make_obs_history(self, first_obs):
        """Initialize by repeating the first observation."""
        return [np.array(first_obs, dtype=np.float32).ravel() for _ in range(self.steps)]

    def step(self, action):
        # Force action to 1D array
        action = np.array(action, dtype=np.float32).ravel()

        obs, reward, terminated, truncated, info = self.env.step(action)
        obs = np.array(obs, dtype=np.float32).ravel()

        # update histories
        self.obs_history.pop(0)
        self.obs_history.append(obs)

        self.act_history.pop(0)
        self.act_history.append(action)

        # concat into a single flat vector
        obs_out = np.concatenate(self.obs_history + self.act_history).astype(np.float32)
        return obs_out, reward, terminated, truncated, info

    def reset(self, seed: Optional[int] = None, options: Optional[dict] = None):
        obs, info = self.env.reset(seed=seed, options=options)
        obs = np.array(obs, dtype=np.float32).ravel()

        self.obs_history = self._make_obs_history(obs)
        self.act_history = self._make_action_history()

        obs_out = np.concatenate(self.obs_history + self.act_history).astype(np.float32)
        return obs_out, info

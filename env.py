import time

import gymnasium as gym
import numpy as np
import serial
from gymnasium import spaces


class RealPendulumEnv(gym.Env):
    """
    Gymnasium environment for a real pendulum controlled by ESP32 + AS5600.
    Fixed at 10 Hz control loop.
    Communication protocol:
      - Send: motor command as integer string + '\n'  (range -255..255)
      - Receive: angle in degrees as float string
    """
    metadata = {"render.modes": []}

    def __init__(self, port="/dev/ttyUSB0", baudrate=19200, max_episode_steps=200):
        super().__init__()

        # --- Serial connection ---
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2.0)  # give ESP32 time to reset

        # --- Spaces ---
        self.action_space = spaces.Box(low=0.0, high=1.0, shape=(1,), dtype=np.float32)
        self.observation_space = spaces.Box(
            low=np.array([0.0], dtype=np.float32),
            high=np.array([np.pi], dtype=np.float32),
            dtype=np.float32,
        )

        # --- State ---
        self.max_episode_steps = max_episode_steps
        self.step_count = 0
        self.last_obs_deg = np.array([0.0], dtype=np.float32)
        self.last_action = 0.0
        self.last_step_time = None
        self.period = 0.1  # 10 Hz

    def _send_action(self, action: float):
        """Send action to ESP32 and get angle back."""
        pwm = int(np.clip(action, -1, 1) * 255)
        cmd = f"{pwm}\n".encode("utf-8")
        self.ser.write(cmd)

        line = self.ser.readline().decode("utf-8").strip()
        try:
            angle_deg = float(line)
        except ValueError:
            angle_deg = float(self.last_obs_deg[0])  # fallback to last obs

        angle_deg = np.clip(angle_deg, 0.0, 180.0)
        return np.array([angle_deg], dtype=np.float32)

    def step(self, action):
        self.step_count += 1

        # --- Send command immediately ---
        self.last_obs_deg = self._send_action(action[0])

        # --- Enforce 10 Hz timing ---
        now = time.time()
        if self.last_step_time is None:
            self.last_step_time = now

        elapsed = now - self.last_step_time
        remaining = self.period - elapsed

        if remaining > 0:
            time.sleep(remaining)
        else:
            print(f"[WARN] Step overran by {-remaining:.3f}s")

        self.last_step_time = time.time()

        angle_deg = float(self.last_obs_deg[0])
        angle_rad = np.deg2rad(angle_deg)

        # --- Reward ---
        angle_rew = np.sin(angle_rad) ** 2
        action_cost = (action[0] - 0.5) ** 2
        continuity_cost = (self.last_action - action[0]) ** 2

        reward = angle_rew - 0.5 * action_cost - 0.0 * continuity_cost

        terminated = False
        truncated = self.step_count >= self.max_episode_steps

        if truncated:
            self._send_action(0.0)

        self.last_action = action[0]

        return np.deg2rad(self.last_obs_deg), float(reward), terminated, truncated, {}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.step_count = 0

        # Reset motor
        self._send_action(0.0)
        time.sleep(1.0)

        # First obs
        self.last_obs_deg = self._send_action(0.0)
        self.last_step_time = time.time()
        return np.deg2rad(self.last_obs_deg), {}

    def close(self):
        if self.ser.is_open:
            self._send_action(0.0)
            self.ser.close()

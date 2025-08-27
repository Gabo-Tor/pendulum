import stable_baselines3 as sb3

from env import RealPendulumEnv
from wrappers import HistoryWrapper

# Create environment
env = RealPendulumEnv(port="/dev/ttyUSB0", baudrate=19200)
env = HistoryWrapper(env, steps=3)

# Define logging directory
log_dir = "./logs/"
experiment_name = "SAC_gamma0.85_lr6e-4_grad6"

model = sb3.SAC(
    "MlpPolicy",
    env,
    verbose=1,
    use_sde=False,
    tensorboard_log=log_dir,
    gamma=0.85,
    learning_rate=6e-4,
    gradient_steps=6,
)

# Train model
model.learn(total_timesteps=30_000, tb_log_name=experiment_name, log_interval=1)
model.save(f"models/{experiment_name}")

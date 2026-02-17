from stable_baselines3 import PPO
from env import HydrofoilEnv

env = HydrofoilEnv()

model = PPO(
    "MlpPolicy",
    env,
    verbose=1,
    learning_rate=3e-4,
    tensorboard_log="./ppo_hydrofoil_tensorboard/"
)

model.learn(total_timesteps=100_000)

model.save("ppo_hydrofoil")

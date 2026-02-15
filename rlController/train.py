from stable_baselines3 import PPO
from env import HydrofoilEnv

env = HydrofoilEnv()

model = PPO(
    "MlpPolicy",
    env,
    verbose=1,
    learning_rate=3e-4,
)

model.learn(total_timesteps=500_000)

model.save("ppo_hydrofoil")

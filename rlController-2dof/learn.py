from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize, VecMonitor
from env import HydrofoilEnv
from stable_baselines3.common.env_checker import check_env
from stateLoggingCallback import stateLoggingCallback

model = PPO.load("ppo_hydrofoil")
env = DummyVecEnv([lambda: HydrofoilEnv()])
env = VecMonitor(env, "./logs/hydrofoil_monitor")
env = VecNormalize(env, norm_obs=True, norm_reward=False)
model.set_env(env)
callback = stateLoggingCallback()
model.learn(total_timesteps=10_000, callback=callback)

model.save("ppo_hydrofoil4dof")
env.save("vecnormalize_stats.pkl")

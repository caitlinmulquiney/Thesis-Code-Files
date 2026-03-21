from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize, VecMonitor
from env import HydrofoilEnv
from stable_baselines3.common.env_checker import check_env
from stateLoggingCallback import stateLoggingCallback

model = PPO.load("ppo_hydrofoil_3dof_1")
env = DummyVecEnv([lambda: HydrofoilEnv()])
env = VecMonitor(env, "./logs/hydrofoil_monitor")
env = VecNormalize(env, norm_obs=True, norm_reward=False)
model.set_env(env)
callback = stateLoggingCallback()
model.learn(total_timesteps=50_000, callback=callback)

model.save("ppo_hydrofoil_4dof_2")
env.save("vecnormalize_stats.pkl")

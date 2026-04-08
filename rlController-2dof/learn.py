from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize, VecMonitor
from delayEnv import HydrofoilEnv
from stable_baselines3.common.env_checker import check_env
from stateLoggingCallback import stateLoggingCallback

model = PPO.load("ppo_hydrofoil_4dof_delay_prob_1")
env = DummyVecEnv([lambda: HydrofoilEnv()])
env = VecMonitor(env, "./logs/hydrofoil_monitor")
env = VecNormalize.load("ppo_hydrofoil_4dof_delay_prob_1.pkl", env)
env.training = True
env.norm_reward = True
model.set_env(env)
callback = stateLoggingCallback()
model.learn(total_timesteps=50_000, callback=callback)

model.save("ppo_hydrofoil_4dof_delay_prob_2")
env.save("ppo_hydrofoil_4dof_delay_prob_2.pkl")

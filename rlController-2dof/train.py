from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize, VecMonitor
from delayEnv import HydrofoilEnv
from stable_baselines3.common.env_checker import check_env
from stateLoggingCallback import stateLoggingCallback
from stable_baselines3 import SAC
import numpy as np


env = DummyVecEnv([lambda: HydrofoilEnv()])
env = VecMonitor(env, "./logs/hydrofoil_monitor")
# 2. Wrap with VecNormalize
env = VecNormalize(env, norm_obs=True, norm_reward=False)
# model = SAC(
#     "MlpPolicy",
#     env,
#     verbose=1,
#     learning_rate=3e-4,
#     buffer_size=50_000,
#     learning_starts=1000,
#     batch_size=256,
#     policy_kwargs=dict(net_arch=[256, 256]),
#     tensorboard_log="./sac_hydrofoil_tensorboard/"
# )

model = PPO(
    "MlpPolicy",
    env,
    verbose=1,
    learning_rate=3e-4,
    n_steps=1024,
    tensorboard_log="./ppo_hydrofoil_4dof_tensorboard/"
)

callback = stateLoggingCallback()
model.learn(total_timesteps=150_000, callback=callback)
model.save("ppo_hydrofoil_4dof_actuate_1")
env.save("vecnormalize_stats_4dof_actuate_1.pkl")

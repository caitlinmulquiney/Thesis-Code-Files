from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize, VecMonitor
from env import HydrofoilEnv
from stable_baselines3.common.env_checker import check_env

env = DummyVecEnv([lambda: HydrofoilEnv()])
env = VecMonitor(env)
# 2. Wrap with VecNormalize
env = VecNormalize(env, norm_obs=True, norm_reward=False)
model = PPO(
    "MlpPolicy",
    env,
    verbose=1,
    learning_rate=3e-4,
    n_steps=256,
    tensorboard_log="./ppo_hydrofoil_tensorboard/"
)

model.learn(total_timesteps=50_000)

model.save("ppo_hydrofoil")
env.save("vecnormalize_stats.pkl")

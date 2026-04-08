from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv, VecMonitor
from delayEnv import HydrofoilEnv
import numpy as np

raw_env = DummyVecEnv([lambda: HydrofoilEnv()])
raw_monitor = VecMonitor(raw_env)  # logs raw rewards before normalisation
env = VecNormalize(
    raw_monitor,
    norm_obs=True,
    norm_reward=True,
    clip_obs=10.0,
    clip_reward=20.0,
    gamma=0.99
)

obs = raw_env.reset()
raw_rewards = []
episode_reward = 0

for _ in range(2000):
    action = raw_env.action_space.sample()  # random actions just to see reward scale
    obs, reward, done, info = raw_env.step([action])
    episode_reward += reward[0]
    if done[0]:
        raw_rewards.append(episode_reward)
        episode_reward = 0
        obs = raw_env.reset()

print(f"Raw reward mean: {np.mean(raw_rewards):.2f}")
print(f"Raw reward std:  {np.std(raw_rewards):.2f}")
print(f"Raw reward min:  {np.min(raw_rewards):.2f}")
print(f"Raw reward max:  {np.max(raw_rewards):.2f}")
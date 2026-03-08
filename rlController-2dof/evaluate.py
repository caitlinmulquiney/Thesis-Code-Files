from stable_baselines3 import PPO
from env import HydrofoilEnv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv

# Load
eval_env = DummyVecEnv([lambda: HydrofoilEnv()])
eval_env = VecNormalize.load("vecnormalize_stats.pkl", eval_env)
eval_env.training = False
eval_env.norm_reward = False

model = PPO.load("ppo_hydrofoil")

# Run evaluation episodes
num_episodes = 5
for episode in range(num_episodes):
    obs = eval_env.reset()
    done = False
    total_reward = 0
    steps = 0

    print(f"\n--- Episode {episode + 1} ---")
    
    fig, ax = plt.subplots()
    # Initialize an empty line object that will be updated
    line, = ax.plot([], [])
    # Set axis limits if known, or enable autoscale view
    ax.set_ylim(-3, 3)
    ax.set_xlim(0, 100) 

    while not done:
        # Use the trained model to predict action
        action, _states = model.predict(obs, deterministic=True)
        
        # Take action in environment
        obs, reward, terminated, truncated = eval_env.step(action)
        
        total_reward += float(reward[0])
        steps += 1
        
        raw_obs = eval_env.get_original_obs()
        height = raw_obs[0][0]
        pitch = np.rad2deg(raw_obs[0][2])

        print(f"Step {steps}: Height={height:.3f}, Pitch={pitch:.2f}°, Roll={np.rad2deg(raw_obs[0][1]):.3f} Actions={action}")
        done = terminated[0]
    
    print(f"Episode {episode + 1} finished - Steps: {steps}, Total Reward: {total_reward:.4f}")

eval_env.close()
print("\nEvaluation complete!")

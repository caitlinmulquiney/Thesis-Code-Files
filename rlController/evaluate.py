from stable_baselines3 import PPO
from env import HydrofoilEnv
import numpy as np

# Load the trained model
model = PPO.load("ppo_hydrofoil")

# Create environment
env = HydrofoilEnv()

# Run evaluation episodes
num_episodes = 5
for episode in range(num_episodes):
    obs, info = env.reset()
    done = False
    total_reward = 0
    steps = 0
    
    print(f"\n--- Episode {episode + 1} ---")
    
    while not done:
        # Use the trained model to predict action
        action, _states = model.predict(obs, deterministic=True)
        
        # Take action in environment
        obs, reward, terminated, truncated, info = env.step(action)
        done = terminated or truncated
        
        total_reward += reward
        steps += 1
        
        # Print state periodically
        if steps % 10 == 0:
            height = obs[2]
            pitch = obs[4] * 180 / np.pi
            print(f"Step {steps}: Height={height:.3f}, Pitch={pitch:.2f}°, Actions={action}")
    
    print(f"Episode {episode + 1} finished - Steps: {steps}, Total Reward: {total_reward:.4f}")

env.close()
print("\nEvaluation complete!")

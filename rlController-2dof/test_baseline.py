from env import HydrofoilEnv
import numpy as np

env = HydrofoilEnv()
obs, _ = env.reset()

print("Testing stability with zero action changes (constant baseline foil angles)...")
for step in range(200):
    action = np.array([0.0, 0.0])
    obs, reward, terminated, truncated, info = env.step(action)
    
    height = obs[2]
    pitch = obs[4] * 180 / np.pi
    u_speed = obs[6]
    print(f"Step {step}: height={height:.3f}, pitch={pitch:.2f}°, u={u_speed:.2f}")
    
    if terminated or truncated:
        print(f"Episode terminated at step {step}")
        break
else:
    print("Completed 200 steps successfully!")

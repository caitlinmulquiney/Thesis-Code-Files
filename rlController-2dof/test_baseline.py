from env import HydrofoilEnv
import numpy as np

env = HydrofoilEnv()
obs, _ = env.reset()

print("Testing stability with zero action changes (constant baseline foil angles)...")
print(obs)
for step in range(50):
    action = np.array([-1.0, -1.0, -1.0, 0.0, 0.0, 0.0, 0.0])
    obs, reward, terminated, truncated, info = env.step(action)
    
    height = obs[0]
    pitch = obs[2] * 180 / np.pi
    u_speed = obs[6]
    print(f"Step {step}: height={height:.3f}, pitch={pitch:.2f}°, u={u_speed:.2f}")
    
    if terminated or truncated:
        print(f"Episode terminated at step {step}")
        break
else:
    print("Completed 200 steps successfully!")

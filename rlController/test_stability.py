from env import HydrofoilEnv
import numpy as np

env = HydrofoilEnv()
obs, _ = env.reset()

print("Testing stability with zero actions...")
for step in range(100):
    action = np.array([0.0, 0.0])
    obs, reward, terminated, truncated, info = env.step(action)
    
    if step % 10 == 0:
        print(f"Step {step}: height={obs[2]:.3f}, pitch={obs[4]:.3f}, u={obs[6]:.3f}")
    
    if terminated or truncated:
        print(f"Episode terminated at step {step}")
        break

print("Test complete!")

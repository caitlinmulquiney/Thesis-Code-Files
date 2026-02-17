import gymnasium as gym
from gymnasium import spaces
import numpy as np
from simulator import HydrofoilSimulator


class HydrofoilEnv(gym.Env):

    def __init__(self):
        super().__init__()

        self.sim = HydrofoilSimulator(dt=0.05)

        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(12,),
            dtype=np.float32
        )

        self.action_space = spaces.Box(
            low=np.array([-2.0, -2.0]),  # [-2, 2] on 0.5 increments
            high=np.array([2.0, 2.0]),
            dtype=np.float32
        )

        self.target_height = -1.3

    def reset(self, seed=None, options=None):
        state = self.sim.reset()
        return state.astype(np.float32), {}

    def step(self, action):
        state = self.sim.step(action)

        # Check for NaN or inf in state
        if not np.all(np.isfinite(state)):
            print("WARNING: State contains NaN or inf values!")
            print(f"State: {state}")
            # Reset to safe state
            state = self.sim.reset()
            return state.astype(np.float32), -1.0, True, False, {}

        # Reward function - stabilized to prevent NaN
        height = state[2]  # z-position (negative when submerged)
        pitch = state[4]

        # Target: height around -1.3 (from reset), pitch near 0
        target_height = -1.3
        
        # Height reward: penalize deviation from target height
        height_error = height - target_height
        #height_reward = 1.8-0.04*(height_error ** 2)-0.05*height_error  # Stronger penalty for height deviation
        height_reward = np.exp(-1.0*abs(height_error))  # Gaussian-shaped reward around target height
        # Pitch reward: penalize large pitch angles
        pitch_reward = 0 #-0.2 * (pitch ** 2)
        
        # Action penalty: encourage smaller actions
        action_penalty = 0 #-0.005 * np.sum(action ** 2)
        
        # Stability bonus: minimal reward each step (discourages doing nothing)
        stability_bonus = 0 #0.01
        
        reward = height_reward + pitch_reward + action_penalty + stability_bonus
        
        # Clip reward to prevent extreme values
        reward = np.clip(reward, -10, 10)

        terminated = False
        truncated = False

        # Terminate if pitch angle gets too large
        if abs(pitch) > np.deg2rad(80):
            terminated = True

        return state.astype(np.float32), float(reward), terminated, truncated, {}

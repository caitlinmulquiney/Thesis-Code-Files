import gym
from gym import spaces
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
            low=np.array([-0.3, -0.3]),  # example limits
            high=np.array([0.3, 0.3]),
            dtype=np.float32
        )

        self.target_height = -1.3

    def reset(self, seed=None, options=None):
        state = self.sim.reset()
        return state.astype(np.float32), {}

    def step(self, action):
        state = self.sim.step(action)

        # Reward example
        height = state[2]
        pitch = state[4]

        reward = (
            - 20 * (height - self.target_height) ** 2
            - 5 * pitch ** 2
            - 0.1 * np.sum(action ** 2)
        )

        terminated = False
        truncated = False

        if abs(pitch) > np.deg2rad(30):
            terminated = True

        return state.astype(np.float32), reward, terminated, truncated, {}

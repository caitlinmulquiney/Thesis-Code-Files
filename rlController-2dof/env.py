import gymnasium as gym
from gymnasium import spaces
import numpy as np
from Rbn import Rbn
from simulator import HydrofoilSimulator


class HydrofoilEnv(gym.Env):

    def __init__(self):
        super().__init__()

        self.sim = HydrofoilSimulator(dt=0.05,  wind_file="wind170528.txt")

        self.observation_space = spaces.Box(
            low=np.array([
                # eta - position and orientation
                -2.5,                # z height (m)
                -np.deg2rad(30),     # roll (rad)
                -np.deg2rad(30),     # pitch (rad)
                -np.pi,              # yaw (rad)
                # nu - velocities
                -30.0,               # surge velocity (m/s)
                -30.0,               # sway velocity (m/s)
                -30.0,                # heave velocity (m/s)
                -30,                 # roll rate (rad/s)
                -30,              # pitch rate (rad/s)
                -30,              # yaw rate (rad/s)
            ], dtype=np.float32),
            high=np.array([
                0.5,                 # can't go above water
                np.deg2rad(30),
                np.deg2rad(30),
                np.pi,
                30.0,
                30.0,
                30.0,
                30,                 # roll rate (rad/s)
                30,              # pitch rate (rad/s)
                30,              # yaw rate (rad/s)
            ], dtype=np.float32),
            dtype=np.float32
        )

        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0]),
            high=np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]),
            dtype=np.float32
        )

        self.last_action = np.zeros(7)

    def reset(self, seed=None, options=None):
        self.last_action = np.zeros(7)
        state = self.sim.reset()
        return self._get_obs(state), {}

    def step(self, action):
        state, sim_failed, wind = self.sim.step(action)  # unpack both values

        if sim_failed:
            return self._get_obs(state), -10.0, True, False, {}

        # Check for NaN or inf in state
        if not np.all(np.isfinite(state)):
            print("WARNING: State contains NaN or inf values!")
            print(f"State: {self._get_obs(state)}")
            return self._get_obs(state), -10.0, True, False, {}

        if np.any(np.abs(self._get_obs(state)) > 1e6):
            print(f"OVERFLOW WARNING - state values: {state}")
            print(f"Max value at index: {np.argmax(np.abs(self._get_obs(state)))}")
            return self._get_obs(state), -10.0, True, False, {}

        height = state[2]
        pitch = state[4]
        roll = state[3]
        yaw = state[5]
        x_velocity = state[6]
        y_velocity = state[7]
        roll_rate = state[9]
        pitch_rate = state[10]
        drift = np.arctan2(y_velocity, x_velocity)

        #vel_body = np.array([x_velocity, y_velocity, 0])
        #vel_ned = Rbn(state[0:6]) @ vel_body
        #boat_speed = np.sqrt(vel_ned[0]**2 + vel_ned[1]**2)*1.944
        boat_speed_knots = np.sqrt(x_velocity**2 + y_velocity**2) * 1.944
        windVelocityInN = np.array([
            wind["speedInN"] * np.cos(wind["direction"]),
            wind["speedInN"] * np.sin(wind["direction"]),
            0.0
        ])
        flowLinearVelocityInB = Rbn(state[0:6]).T @ windVelocityInN
        twa = np.arctan2(flowLinearVelocityInB[1], flowLinearVelocityInB[0])
        twa = - np.pi + wind["direction"]
        vmg_knots = abs(boat_speed_knots * np.cos(twa))
        target_vmg = -0.00000272*np.rad2deg(abs(twa))**4+0.001025*np.rad2deg(abs(twa))**3-0.12778*np.rad2deg(abs(twa))**2+6.012*np.rad2deg(abs(twa))-74
        # print(boat_speed)
        # print(wind["direction"])  
        # print(twa)
        # print(vmg_knots)
        # print(target_vmg)
        target_height = -1.3 # permissable range from 0.1 to -2.5
        target_pitch = 0.5 * np.pi / 180 # permissable range -5/10 to 5/10 deg
        target_roll = -1 * np.pi / 180
        target_drift = np.deg2rad(1)
    
        drift_error = drift - target_drift
        height_error = height - target_height 
        roll_error = roll - target_roll
        pitch_error = pitch - target_pitch
        roll_rate_error = roll_rate

        height_reward = 2-40.0 * height_error**2
        pitch_reward = 1-50 * pitch_error**2
        roll_reward = 1-50 * roll_error**2
        drift_reward = 1-50*drift_error**2
        yaw_reward = 1-50*yaw**2
        roll_rate_reward = 1-50*roll_rate_error**2
        saturation_penalty = -0.1 * np.sum(np.maximum(0, np.abs(action) - 0.8)**2)

        reward = height_reward + pitch_reward + roll_reward + roll_rate_reward + drift_reward + yaw_reward
        #reward = height_reward + pitch_reward + roll_reward + roll_rate_reward + drift_reward + speed_reward -0.1 * (height_error**2 + pitch_error**2 + roll_error**2) + saturation_penalty

        terminated = False
        truncated = False

        self.last_action = action.copy()

        if abs(pitch) > np.deg2rad(10):
            print(f"Pitch exceeds maximum value: {state[4]}")
            return self._get_obs(state), -50.0, True, False, {}
        
        if abs(roll) > np.deg2rad(10):
            print(f"Roll exceeds maximum value: {state[3]}")
            return self._get_obs(state), -50.0, True, False, {}

        if height < -2 or height > 0.1:
            print(f"Height exceeds maximum value: {state[2]}")
            return self._get_obs(state), -50.0, True, False, {}

        return self._get_obs(state), float(reward), terminated, truncated, {}

    def _get_obs(self, state):
        # Drop x (index 0) and y (index 1), return remaining 10 values
        return state[2:].astype(np.float32)
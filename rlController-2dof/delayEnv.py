import gymnasium as gym
from gymnasium import spaces
import numpy as np
from collections import deque
from Rbn import Rbn
from simulator import HydrofoilSimulator

class MNetwork:
    """
    Lightweight numpy-based M network that runs alongside the RL policy.
    Learns online: call update() every step with (state, true_actuator_pos).
    Call predict() to get the estimated actuator position for a given state.
 
    For use with a PyTorch-based RL trainer, see MNetworkTorch below.
    """
 
    def __init__(self, state_dim: int = 10, hidden_dim: int = 32,
                 actuator_dim: int = 7, lr: float = 1e-3):
        self.lr = lr
        # Xavier init
        self.W1 = np.random.randn(state_dim, hidden_dim) * np.sqrt(2.0 / state_dim)
        self.b1 = np.zeros(hidden_dim)
        self.W2 = np.random.randn(hidden_dim, actuator_dim) * np.sqrt(2.0 / hidden_dim)
        self.b2 = np.zeros(actuator_dim)
 
    def _forward(self, x: np.ndarray):
        h = np.tanh(x @ self.W1 + self.b1)          # (hidden,)
        out = np.tanh(h @ self.W2 + self.b2)         # (actuator,)  range [-1,1]
        return h, out
 
    def predict(self, state: np.ndarray) -> np.ndarray:
        """Return estimated current_actuator_pos given plant state."""
        _, out = self._forward(state)
        return out
 
    def update(self, state: np.ndarray, true_actuator_pos: np.ndarray) -> float:
        """
        One gradient-descent step minimising  0.5 * ||M(x) - u_actual||^2
        (paper eq. 12-15).  Returns scalar loss for logging.
        """
        h, out = self._forward(state)
 
        # Error (paper's e_M)
        error = out - true_actuator_pos                # (actuator,)
        loss = 0.5 * np.sum(error ** 2)
 
        # Output layer grad
        d_out = error * (1.0 - out ** 2)              # tanh deriv
        dW2 = np.outer(h, d_out)
        db2 = d_out
 
        # Hidden layer grad
        d_h = (d_out @ self.W2.T) * (1.0 - h ** 2)
        dW1 = np.outer(state, d_h)
        db1 = d_h
 
        self.W1 -= self.lr * dW1
        self.b1 -= self.lr * db1
        self.W2 -= self.lr * dW2
        self.b2 -= self.lr * db2
 
        return float(loss)
 

class HydrofoilEnv(gym.Env):
    def __init__(self, use_m_network: bool = True):
        super().__init__()
        self.last_action = np.zeros(7)
        self.delay_steps = 5
        self.action_buffer = deque(maxlen=self.delay_steps)
        self.current_actuator_pos = np.zeros(7)
        self.rate_limit = np.array([0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05])
        self.sim = HydrofoilSimulator(dt=0.05,  wind_file=None)

        self.use_m_network = use_m_network
        if use_m_network:
            self.m_net = MNetwork(state_dim=10, hidden_dim=32, actuator_dim=7, lr=3e-4)
        self.m_loss_log = [] 

        self.observation_space = spaces.Box(
            low=np.concatenate([
                np.array([
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
                np.full(7, -1.0, dtype=np.float32),
                np.full(7, -1.0, dtype=np.float32),
                np.full(self.delay_steps * 7, -1.0, dtype=np.float32),
            ]),
            high=np.concatenate([
                np.array([
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
                np.full(7, 1.0, dtype=np.float32),
                np.full(7, 1.0, dtype=np.float32),
                np.full(self.delay_steps * 7, 1.0, dtype=np.float32)
            ]),
            dtype=np.float32
        )

        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0]),
            high=np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]),
            dtype=np.float32
        )

    def reset(self, seed=None, options=None):
        self.last_action = np.zeros(7)
        self.current_actuator_pos = np.zeros(7)
        state = self.sim.reset()
        self.action_buffer.clear()
        for _ in range(self.delay_steps):
            self.action_buffer.append(np.zeros(self.action_space.shape))
        return self._get_obs(state), {}

    def step(self, action):
        self.action_buffer.append(action)
        delayed_action = self.action_buffer[0]
        delta = delayed_action - self.current_actuator_pos
        delta_clipped = np.clip(delta, -self.rate_limit, self.rate_limit)
        self.current_actuator_pos = self.current_actuator_pos + delta_clipped
        actual_action = self.current_actuator_pos.copy()
        
        state, sim_failed, wind = self.sim.step(actual_action)
        
        if self.use_m_network:
            plant = self._plant_state(state)
            m_loss = self.m_net.update(plant, self.current_actuator_pos)
            self.m_loss_log.append(m_loss)

        obs = self._get_obs(state)
        if sim_failed:
            return self._get_obs(state), -10.0, True, False, {}
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

        pitch_health  = max(0.0, 1.0 - abs(pitch_error) / 10)
        roll_health   = max(0.0, 1.0 - abs(roll_error)  / 10)
        height_health = max(0.0, 1.0 - abs(height_error) / 1.5)

        survival_bonus = 2.0 * pitch_health * roll_health * height_health

        height_reward = 2-40.0 * height_error**2
        pitch_reward = 1-50 * pitch_error**2
        roll_reward = 1-50 * roll_error**2
        drift_reward = 1-50*drift_error**2
        roll_rate_reward = 1-50*roll_rate_error**2
        yaw_reward = 1-50*yaw**2
        gap_reward = -0.5 * np.sum(delta**2)
        action_reward = -0.1 * np.sum(action**2)

        m_residual_penalty = 0.0
        if self.use_m_network and len(self.m_loss_log) > 0:
            m_residual_penalty = -0.5 * self.m_loss_log[-1] 

        reward = height_reward + pitch_reward + roll_reward + yaw_reward + drift_reward + roll_rate_reward + survival_bonus + m_residual_penalty + gap_reward + action_reward

        terminated = False
        truncated = False

        self.last_action = action.copy()

        if abs(pitch) > np.deg2rad(10):
            print(f"Pitch exceeds maximum value: {state[4]}")
            return self._get_obs(state), -100.0, True, False, {}
        
        if abs(roll) > np.deg2rad(10):
            print(f"Roll exceeds maximum value: {state[3]}")
            return self._get_obs(state), -100.0, True, False, {}

        if height < -2 or height > 0.1:
            print(f"Height exceeds maximum value: {state[2]}")
            return self._get_obs(state), -100.0, True, False, {}

        return self._get_obs(state), float(reward), terminated, truncated, {}

    def _plant_state(self, state: np.ndarray) -> np.ndarray:
        """The 10-dim plant state vector used as M network input."""
        return state[2:].astype(np.float32)
    
    def _get_obs(self, state: np.ndarray) -> np.ndarray:
        plant = self._plant_state(state)
 
        m_pred = (
            self.m_net.predict(plant).astype(np.float32)
            if self.use_m_network
            else np.zeros(7, dtype=np.float32)
        )
 
        pending_actions = np.array(self.action_buffer, dtype=np.float32).flatten()

        return np.concatenate([
            plant,
            self.current_actuator_pos.astype(np.float32),
            m_pred,
            pending_actions,
        ])
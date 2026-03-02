from stable_baselines3.common.callbacks import BaseCallback
import numpy as np

class stateLoggingCallback(BaseCallback):
    def __init__(self, verbose=0):
        super().__init__(verbose)

    def _on_step(self) -> bool:
        # Get current state from the environment
        state = self.training_env.envs[0].sim.state
        
        height = state[2]
        pitch = np.rad2deg(state[4])
        roll = np.rad2deg(state[3])
        
        self.logger.record("state/height", height)
        self.logger.record("state/pitch_deg", pitch)
        self.logger.record("state/roll_deg", roll)
        
        return True
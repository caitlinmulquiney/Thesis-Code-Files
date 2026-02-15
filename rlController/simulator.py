import numpy as np
from System import system_dynamics           # your converted MATLAB System()
from massDistribTrimaran import massDistribTrimaran
from foilLoad import foilLoad
from weightLoad import weightLoad
from aerodynamicLoadSuperstructure import aerodynamicLoadSuperstructure
from coriolisCentripetal import coriolisCentripetal
from Jbn import Jbn
from Tbn import Tbn
from Rbn import Rbn                           # import Rbn from your file

class HydrofoilSimulator:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
    def __init__(self, dt=0.05):
        """
        RL-friendly hydrofoil simulator
        dt : float
            Simulation time step (s)
        """
        self.dt = dt
        self.state = None

    def reset(self):
        """
        Reset the environment to initial equilibrium
        """
        U0 = 16.18  # Boat speed
        beta0 = 1.3 * np.pi / 180  # Boat drift angle

        eta0 = np.array([
            0, 0, -1.3,
            2.6 * np.pi / 180,
            -0.5 * np.pi / 180,
            -0.2 * np.pi / 180
        ])  # boat attitude

        vel_nav = np.array([U0 * np.cos(beta0),
                            U0 * np.sin(beta0),
                            0])
        nu_linear = Rbn(eta0).T @ vel_nav  # boat velocity
        nu0 = np.concatenate((nu_linear, np.zeros(3)))

        self.state = np.concatenate((eta0, nu0))
        return self.state.copy()

    def step(self, action):
        """
        Take one simulation step
        action : np.array
            Control inputs from PPO
        """
        self.state = rk4_step(self.state, action, self.dt)
        return self.state.copy()


# ===============================
# RK4 Integrator
# ===============================
def rk4_step(state, action, dt):
    k1 = system_dynamics(state, action)
    k2 = system_dynamics(state + 0.5 * dt * k1, action)
    k3 = system_dynamics(state + 0.5 * dt * k2, action)
    k4 = system_dynamics(state + dt * k3, action)
    return state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

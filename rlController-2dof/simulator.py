import numpy as np
import copy
from System import system, update_foil_list
from loadFoilDescription import loadFoilDescription
from massDistribTrimaran import massDistribTrimaran
from foilLoad import foilLoad
from weightLoad import weightLoad
from aerodynamicLoadSuperstructure import aerodynamicLoadSuperstructure
from coriolisCentripetal import coriolisCentripetal
from Jbn import Jbn
from Tbn import Tbn
from Rbn import Rbn                           # import Rbn from your file
from model import Boat

class HydrofoilSimulator:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
    def __init__(self, dt=0.05, wind_file = None):
        """
        RL-friendly hydrofoil simulator
        dt : float
            Simulation time step (s)
        """
        self.dt = dt
        self.wind_index = 0
        self.state = None
        self.foil_list = loadFoilDescription()
        self.wind_index = 0
        self.steps = 0
        self.boat_model = Boat()
        
        if wind_file is not None:
            self.wind_speeds = np.loadtxt(wind_file)  # 1D array of speeds
        else:
            self.wind_speeds = None

    def reset(self):
        """
        Reset the environment to initial equilibrium
        """
        # self.steps = 0
        self.wind_index = 0
        wind = self.get_wind()
        U0 = 16.18  # Boat speed
        beta0 = 1.3 * np.pi / 180  # Boat drift angle

        eta0 = np.array([
            0, 0, -1.3,
            2.6 * np.pi / 180,
            -0.5 * np.pi / 180,
            0 * np.pi / 180
        ])  # boat attitude

        vel_nav = np.array([U0 * np.cos(beta0),
                            U0 * np.sin(beta0),
                            0])
        nu_linear = Rbn(eta0).T @ vel_nav  # boat velocity
        nu0 = np.concatenate((nu_linear, np.zeros(3)))

        self.state = np.concatenate((eta0, nu0))
        self.foil_list = loadFoilDescription()  

        self.state[2] += np.random.uniform(-0.2, 0.2)
        self.state[4] += np.random.uniform(-0.02, 0.02)
        self.boat_model.drawBoat(np.array([0,0,self.state[2],self.state[3], self.state[4], 0]), self.foil_list, wind)
        return self.state.copy()

    def get_wind(self):
        if self.wind_speeds is None:
            return {"speedInN": 9.231, "direction": 30 * np.pi / 180}
        
        idx = self.wind_index % len(self.wind_speeds)
        speed = self.wind_speeds[idx]
        self.wind_index += 1
        direction = np.random.uniform(45*np.pi/180, 90*np.pi/180)
        
        return {"speedInN": speed, "direction": direction}

    def step(self, action):
        self.foil_list = update_foil_list(action)
        wind = self.get_wind()
        result = rk4_step(self.state, self.foil_list, self.dt, wind=wind)
        
        if result is None:
            return self.state.copy(), True
        
        self.state = result
        if self.steps % 20 == 0: 
            self.boat_model.updateBoat(np.array([0,0,self.state[2],self.state[3], self.state[4], self.state[5]]), self.foil_list, wind)
        self.steps += 1
        return self.state.copy(), False


# ===============================
# RK4 Integrator
# ===============================
def rk4_step(state, foil_list, dt, wind=None):
    k1 = system(foil_list, state, wind)
    k2 = system(foil_list, state + 0.5*dt*k1, wind)
    k3 = system(foil_list, state + 0.5*dt*k2, wind)
    k4 = system(foil_list, state + dt*k3, wind)
    state = state + dt/6 * (k1 + 2*k2 + 2*k3 + k4)

    if not np.all(np.isfinite(state)):
            return None

    return state

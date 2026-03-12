import numpy as np
import copy

from Jbn import Jbn
from aerodynamicLoadSuperstructure import aerodynamicLoadSuperstructure
from coriolisCentripetal import coriolisCentripetal
from foilLoad import foilLoad
from loadFoilDescription import loadFoilDescription
from massDistribTrimaran import massDistribTrimaran
from weightLoad import weightLoad


def system(foil_list, state, wind=None, wave=None):
    """
    Continuous-time system dynamics.
    
    Parameters
    ----------
    state : ndarray (12,)
        [eta(6), nu(6)]
    action : ndarray
        Control inputs (for PPO, passed to foil model)
    wind : dict
        Wind parameters
    wave : dict or None
        Wave parameters
    
    Returns
    -------
    dstate : ndarray (12,)
        Time derivative of state
    """

    eta = state[0:6]
    nu = state[6:12]

    if wind is None:
        wind = {
            "speedInN": 9.231,
            "direction": 30 * np.pi / 180
        }

    if wave is None:
        wave = None

    total_load = np.zeros(6)
    
    for idx, foil in enumerate(foil_list):
        total_load += foilLoad(eta, nu, foil, wind, wave)
  
    total_load += weightLoad(eta)
    total_load += aerodynamicLoadSuperstructure(eta, nu, wind)

    M = massDistribTrimaran()

    C = coriolisCentripetal(M, nu)

    nud = np.linalg.solve(M, total_load - C @ nu)

    nud[0] = 0.0  # surge
    nud[1] = 0.0  # sway
    nud[5] = 0.0

    eta_dot = Jbn(eta) @ nu

    dstate = np.concatenate((eta_dot, nud))

    return dstate



def update_foil_list(action):
    """
    Update foil list with new control inputs.
    """
    
    # Apply action to foil angles
    # action[0] = pitch control (foils 1, 3, 5)
    # action[1] = yaw control (foils 4, 6)
    if action is None:
        action = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    pitchLfoil = action[0] * np.deg2rad(10.0)
    pitchTfoilP = action[1] * np.deg2rad(10.0)
    pitchTfoilS = action[2] * np.deg2rad(10.0)
    pitchSail = action[3] * np.deg2rad(90)
    betaSail = action[4] * np.deg2rad(15)
    twist = action[5] * np.deg2rad(10)

    foil_list = loadFoilDescription()  # Load the original foil list
    for idx, foil in enumerate(foil_list):
        if idx == 0:
            foil["attitudeInB"][2] += pitchSail
            foil["beta"] += betaSail
            foil["twist"] += twist
        if idx == 1:
            foil["attitudeInB"][1] += pitchLfoil
        
        if idx == 3:
            foil["attitudeInB"][1] += pitchTfoilS
        
        if idx == 5:
            foil["attitudeInB"][1] += pitchTfoilP
    return foil_list

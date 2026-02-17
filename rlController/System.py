import numpy as np

from Jbn import Jbn
from aerodynamicLoadSuperstructure import aerodynamicLoadSuperstructure
from coriolisCentripetal import coriolisCentripetal
from foilLoad import foilLoad
from loadFoilDescription import loadFoilDescription
from massDistribTrimaran import massDistribTrimaran
from weightLoad import weightLoad


def system(state, action=None, wind=None, wave=None):
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

    # -----------------------------
    # State split
    # -----------------------------
    eta = state[0:6]
    nu = state[6:12]

    # -----------------------------
    # Default wind (if not provided)
    # -----------------------------
    if wind is None:
        wind = {
            "speedInN": 9.231,
            "direction": 30 * np.pi / 180
        }

    if wave is None:
        wave = None

    # -----------------------------
    # Hydrodynamic / aerodynamic loads
    # -----------------------------
    foil_list = loadFoilDescription()

    total_load = np.zeros(6)
    
    # Angle constraints: [-15, 15] degrees = [-0.2618, 0.2618] radians
    angle_min = -15 * np.pi / 180
    angle_max = 15 * np.pi / 180
    
    # Apply action to foil angles
    # action[0] = pitch control (foils 1, 3, 5)
    # action[1] = yaw control (foils 4, 6)
    if action is not None:
        for idx, foil in enumerate(foil_list):
            foil_copy = foil.copy()
            foil_copy["attitudeInB"] = foil["attitudeInB"].copy()
            
            # Rudders (indices 4, 6): control is yaw (element 2)
            if idx == 4 or idx == 6:
                foil_copy["attitudeInB"][2] = np.clip(
                    foil["attitudeInB"][2] + action[1],
                    angle_min,
                    angle_max
                )
            # Other foils (indices 1, 3, 5): control is pitch (element 1)
            elif idx == 1 or idx == 3 or idx == 5:
                foil_copy["attitudeInB"][1] = np.clip(
                    foil["attitudeInB"][1] + action[0],
                    angle_min,
                    angle_max
                )
            
            foil_list_modified.append(foil_copy)
    else:
        foil_list_modified = foil_list

    for foil in foil_list_modified:
        total_load += foilLoad(
            eta,
            nu,
            foil,
            wind,
            wave
        )

    total_load += weightLoad(eta)
    total_load += aerodynamicLoadSuperstructure(eta, nu, wind)

    # -----------------------------
    # Rigid-body dynamics
    # -----------------------------
    M = massDistribTrimaran()

    C = coriolisCentripetal(M, nu)

    # Solve M * nud = total_load - C*nu
    nud = np.linalg.solve(M, total_load - C @ nu)

    # Constrain surge acceleration (as in MATLAB)
    nud[0] = 0.0

    # -----------------------------
    # Kinematics
    # -----------------------------
    eta_dot = Jbn(eta) @ nu

    # -----------------------------
    # State derivative
    # -----------------------------
    dstate = np.concatenate((eta_dot, nud))

    return dstate

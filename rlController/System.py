import numpy as np


def system_dynamics(t, state, action=None, wind=None, wave=None):
    """
    Continuous-time system dynamics.
    
    Parameters
    ----------
    t : float
        Simulation time
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
    foil_list = load_foil_description()

    total_load = np.zeros(6)

    for foil in foil_list:
        total_load += foil_load(
            eta,
            nu,
            foil,
            wind,
            wave,
            action  # <- pass PPO control here
        )

    total_load += weight_load(eta)
    total_load += aerodynamic_load_superstructure(eta, nu, wind)

    # -----------------------------
    # Rigid-body dynamics
    # -----------------------------
    M = mass_distrib_trimaran()

    C = coriolis_centripetal(M, nu)

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

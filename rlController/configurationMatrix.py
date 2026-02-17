import numpy as np
from foilLoad import foilLoad

def configurationMatrix(foil, eta, nu, wind, wave):
    """
    Returns the configuration (Jacobian) matrix J
    """

    du = 0.1 * np.pi / 180  # small angle perturbation [rad]

    # MATLAB: foilList = [2 4 5 6 7]
    # Python (0-based):
    foilList = [1, 3, 4, 5, 6]

    J = np.zeros((6, len(foilList)))

    for idx, foil_idx in enumerate(foilList):

        # Copy foil safely
        foil_ = foil[foil_idx].copy()
        foil_["attitudeInB"] = foil[foil_idx]["attitudeInB"].copy()

        # Rudders: control is yaw (f3)
        # MATLAB: foil 5 and 7
        if foil_idx == 4 or foil_idx == 6:
            dFoilAngle = np.array([0.0, 0.0, du])
        else:
            # Other foils: control is pitch (f2)
            dFoilAngle = np.array([0.0, du, 0.0])

        # Positive perturbation
        foil_["attitudeInB"] = foil[foil_idx]["attitudeInB"] + dFoilAngle
        dLoadPlus = foilLoad(eta, nu, foil_, wind, wave, False)

        # Negative perturbation
        foil_["attitudeInB"] = foil[foil_idx]["attitudeInB"] - dFoilAngle
        dLoadMinus = foilLoad(eta, nu, foil_, wind, wave, False)

        # Central difference
        J[:, idx] = (dLoadPlus - dLoadMinus) / (2 * du)

    return J

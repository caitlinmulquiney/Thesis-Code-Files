import numpy as np
from weightLoad import weightLoad
from aerodynamicLoadSuperstructure import aerodynamicLoadSuperstructure
from foilLoad import foilLoad

def computeResidual(x, eta, nu, foil, wind):
    """
    x : optimization variables
    eta, nu : state vectors
    foil : list of foil dicts
    wind : wind dict
    """

    # --------------------------------------------------
    # Copy foil attitudes so we don't modify them in-place
    # --------------------------------------------------
    foil_local = []
    for f in foil:
        f_copy = f.copy()
        f_copy["attitudeInB"] = f["attitudeInB"].copy()
        foil_local.append(f_copy)

    # --------------------------------------------------
    # Adjust L foils (starboard horizontal + vertical)
    # MATLAB: foil{2}, foil{3}
    # Python: foil_local[1], foil_local[2]
    # --------------------------------------------------
    foil_local[1]["attitudeInB"] += np.array([0.0, x[1], 0.0])
    foil_local[2]["attitudeInB"] += np.array([0.0, x[1], 0.0])

    # --------------------------------------------------
    # Adjust starboard T foil and rudder
    # MATLAB: foil{4}, foil{5}
    # Python: foil_local[3], foil_local[4]
    # --------------------------------------------------
    foil_local[3]["attitudeInB"] += np.array([0.0, x[2], x[4]])
    foil_local[4]["attitudeInB"] += np.array([0.0, 0.0, x[4]])

    # --------------------------------------------------
    # Adjust port T foil and rudder
    # MATLAB: foil{6}, foil{7}
    # Python: foil_local[5], foil_local[6]
    # --------------------------------------------------
    foil_local[5]["attitudeInB"] += np.array([0.0, x[3], x[4]])
    foil_local[6]["attitudeInB"] += np.array([0.0, 0.0, x[4]])

    # --------------------------------------------------
    # Sum all loads
    # --------------------------------------------------
    totalLoad = np.zeros(6)

    for f in foil_local:
        totalLoad += foilLoad(eta, nu, f, wind, [], False)

    totalLoad += weightLoad(eta, False)
    totalLoad += aerodynamicLoadSuperstructure(eta, nu, wind, False)

    # --------------------------------------------------
    # Residual
    # --------------------------------------------------
    w1 = 1.0
    w2 = 1.0

    force_residual = np.sqrt(1e-3 * np.sum(totalLoad[0:3]**2))
    moment_residual = np.sqrt(1e-3 * np.sum(totalLoad[3:6]**2))

    residual = w1 * force_residual + w2 * moment_residual

    return residual

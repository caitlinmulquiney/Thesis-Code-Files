import numpy as np
from Rbn import Rbn

def weightLoad(eta, verbose=False):
    """
    Returns the weight load expressed in {b}
    """

    # --------------------------------------------------
    # Mass properties
    # --------------------------------------------------
    boatMass = 2.3e3 + 87.5 * 6   # 2,332–2,432 kg
    COGpositionInB = np.array([5.92896, 0.07904, -3.86752])
    g = 9.81

    # --------------------------------------------------
    # Weight force in {b}
    # --------------------------------------------------
    weightForceInB = Rbn(eta).T @ np.array([0.0, 0.0, boatMass * g])

    weightLoadInB = np.hstack((
        weightForceInB,
        np.cross(COGpositionInB, weightForceInB)
    ))

    return weightLoadInB

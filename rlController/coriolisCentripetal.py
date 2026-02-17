import numpy as np

from skewSym import skewSym

def coriolisCentripetal(M, nu):
    """
    Computes the Coriolis-centripetal matrix C
    """

    # Angular velocity
    nu2 = nu[3:6]

    # Mass
    m = M[0, 0]

    # Center of gravity vector rG
    rG = np.array([
        M[1, 5] / m,
        -M[2, 3] / m,
        M[0, 4] / m
    ])

    # Inertia matrix in body frame
    Ib = M[3:6, 3:6]

    # Skew-symmetric matrices
    S_nu2 = skewSym(nu2)
    S_rG = skewSym(rG)

    # Assemble Coriolis matrix
    C = np.block([
        [ m * S_nu2, -m * S_nu2 @ S_rG ],
        [ m * S_rG @ S_nu2, -skewSym(Ib @ nu2) ]
    ])

    return C

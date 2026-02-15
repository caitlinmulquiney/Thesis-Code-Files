import numpy as np


def Tbn(eta):
    """
    Transformation matrix from body angular velocity
    to Euler angle rates.

    eta = [x, y, z, phi, theta, psi]
    """

    phi = eta[3]
    theta = eta[4]
    if abs(theta) > np.deg2rad(60):
        terminated = True


    cphi = np.cos(phi)
    sphi = np.sin(phi)
    cth = np.cos(theta)
    sth = np.sin(theta)

    tth = sth / cth

    T = np.array([
        [1.0,      sphi * tth,     cphi * tth],
        [0.0,      cphi,          -sphi],
        [0.0,      sphi / cth,     cphi / cth]
    ])

    return T

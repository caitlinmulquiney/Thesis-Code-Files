import numpy as np


def wave_mechanics(t, area, H, T, d, L, z, y):
    """
    Linear wave pressure force.

    Parameters
    ----------
    t : float
        Current simulation time
    area : float
        Wetted area where pressure acts
    H : float
        Wave height
    T : float
        Wave period
    d : float
        Water depth
    L : float
        Wavelength
    z : float
        Vertical position (body-fixed or inertial depending on model)
    y : float
        Horizontal wave propagation coordinate

    Returns
    -------
    wave_force : float
        Resulting wave force
    """

    rho_w = 1025.0
    g = 9.81

    omega = 2 * np.pi / T
    a = H / 2.0
    k = 2 * np.pi / L

    # Linear wave pressure expression
    p = (
        -rho_w * g * z
        + rho_w * g * a
        * np.cosh(k * (d + z))
        * np.cos(k * y - omega * t)
        / np.cosh(k * d)
    )

    wave_force = p * area

    return wave_force

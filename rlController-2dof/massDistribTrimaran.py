import numpy as np


def massDistribTrimaran():
    """
    Rough estimation of the inertia properties of a sailing trimaran.
    Reference point is below mast, at deck level.
    Returns:
        M (6x6 numpy array): Rigid-body mass matrix
    """

    # Total mass
    mtot = (2.3e3 + 87.5 * 6)

    # Center of gravity (COG)
    xG = 0
    yG = 0.07904
    zG = -3.86752

    # Moments of inertia at COG
    Ixx_cog = 129000
    Iyy_cog = 143000
    Izz_cog = 52000

    # Radii of gyration (not used later, but kept for completeness)
    rxx = np.sqrt(Ixx_cog / mtot)
    ryy = np.sqrt(Iyy_cog / mtot)
    rzz = np.sqrt(Izz_cog / mtot)

    # Parallel axis theorem
    Ixx = Ixx_cog + mtot * (yG**2 + zG**2)
    Iyy = Iyy_cog + mtot * (xG**2 + zG**2)
    Izz = Izz_cog + mtot * (xG**2 + yG**2)

    # Initialize mass matrix
    M = np.zeros((6, 6))

    # Diagonal terms
    M[0, 0] = mtot
    M[1, 1] = mtot
    M[2, 2] = mtot
    M[3, 3] = Ixx
    M[4, 4] = Iyy
    M[5, 5] = Izz

    # Express zG in {b}
    zG = -zG

    # Off-diagonal coupling terms
    M[0, 4] = mtot * zG
    M[4, 0] = M[0, 4]

    M[1, 3] = -mtot * zG
    M[3, 1] = M[1, 3]

    M[1, 5] = mtot * xG
    M[5, 1] = M[1, 5]

    M[2, 4] = -mtot * xG
    M[4, 2] = M[2, 4]

    M[0, 5] = -mtot * yG
    M[5, 0] = M[0, 5]

    M[2, 3] = mtot * yG
    M[3, 2] = M[2, 3]

    return M

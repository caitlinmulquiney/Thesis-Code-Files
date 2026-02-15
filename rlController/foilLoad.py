import numpy as np
from Rbn import Rbn

def foilLoad(eta, nu, foil, wind, wave):
    """
    Inputs:
    - eta and nu (Fossen notation)
    - foil: dict from loadFoilDescription()
    - wind, wave: dict-like objects (only wind used for now)
    - verbose: boolean
    """

    # Densities [kg/m^3]
    rho_air = 1.0
    rho_water = 1025.0

    # --------------------------------------------------
    # Check whether foil is in air, water, or partially submerged
    # --------------------------------------------------
    foilEnd1Position = (
        eta[0:3]
        + Rbn(eta)
        @ (
            foil["positionInB"]
            + Rbn(np.hstack((np.zeros(3), foil["attitudeInB"])))
            @ np.array([0, -foil["span"] / 2, 0])
        )
    )

    foilEnd2Position = (
        eta[0:3]
        + Rbn(eta)
        @ (
            foil["positionInB"]
            + Rbn(np.hstack((np.zeros(3), foil["attitudeInB"])))
            @ np.array([0, foil["span"] / 2, 0])
        )
    )

    Dplus = max(foilEnd1Position[2], foilEnd2Position[2])
    Dminus = min(foilEnd1Position[2], foilEnd2Position[2])

    if np.sign(Dplus) != np.sign(Dminus):
        foilStatus = "partiallySubmerged"
        sigma = Dplus / (Dplus - Dminus)
    else:
        if Dminus > 0:
            foilStatus = "inWater"
            sigma = 1.0
        else:
            foilStatus = "inAir"

    # --------------------------------------------------
    # Relative velocity of the foil wrt the fluid (in {b})
    # --------------------------------------------------
    foilLinearVelocityInB = nu[0:3] + np.cross(nu[3:6], foil["positionInB"])

    windVelocityInN = np.array([
        wind["speedInN"] * np.cos(wind["direction"]),
        wind["speedInN"] * np.sin(wind["direction"]),
        0.0
    ])

    waveVelocityInN = np.array([0.0, 0.0, 0.0])

    if foilStatus == "inAir":
        flowLinearVelocityInB = Rbn(eta).T @ windVelocityInN
    else:
        flowLinearVelocityInB = Rbn(eta).T @ waveVelocityInN

    relativeLinearVelocityInB = foilLinearVelocityInB - flowLinearVelocityInB

    # --------------------------------------------------
    # Transport velocity into foil frame {f}
    # --------------------------------------------------
    relativeLinearVelocityInF = (
        Rbn(np.hstack((np.zeros(3), foil["attitudeInB"]))).T
        @ relativeLinearVelocityInB
    )

    foilRelativeSpeed = np.linalg.norm(relativeLinearVelocityInF)

    foilAngleOfAttack = np.arctan2(
        np.dot([0, 0, 1], relativeLinearVelocityInF),
        np.dot([1, 0, 0], relativeLinearVelocityInF),
    )

    # --------------------------------------------------
    # Lift and drag coefficients (paper values)
    # --------------------------------------------------
    aoa_deg = foilAngleOfAttack * 180 / np.pi

    liftCoeff = 0.1083 * aoa_deg - 0.0002
    dragCoeff = 6e-05 * aoa_deg**2 + 5e-07 * aoa_deg + 0.0051

    if foil["type"] == "sail":
        if aoa_deg < 28:
            liftCoeff = -1e-6 * aoa_deg**2 + 0.0927 * aoa_deg + 0.0099
        else:
            liftCoeff = -0.0175 * aoa_deg + 3.0

        dragCoeff = 0.7 * (
            0.00006 * aoa_deg**2 - 0.0014 * aoa_deg + 0.0097
        )

    # --------------------------------------------------
    # Lift and drag forces
    # --------------------------------------------------
    if foilStatus == "inAir":
        lift = 0.5 * rho_air * liftCoeff * foilRelativeSpeed**2 * foil["chord"] * foil["span"]
        drag = 0.5 * rho_air * dragCoeff * foilRelativeSpeed**2 * foil["chord"] * foil["span"]
    else:
        lift = 0.5 * sigma * rho_water * liftCoeff * foilRelativeSpeed**2 * foil["chord"] * foil["span"]
        drag = 0.5 * sigma * rho_water * dragCoeff * foilRelativeSpeed**2 * foil["chord"] * foil["span"]

    # --------------------------------------------------
    # Convert lift/drag into forces and moments in {b}
    # --------------------------------------------------
    foilForceInF = (
        Rbn(np.hstack((np.zeros(3), [0, -foilAngleOfAttack, 0])))
        @ np.array([-drag, 0.0, -lift])
    )

    foilForceInB = (
        Rbn(np.hstack((np.zeros(3), foil["attitudeInB"])))
        @ foilForceInF
    )

    foilLoadInB = np.hstack((
        foilForceInB,
        np.cross(foil["positionInB"], foilForceInB)
    ))

    return foilLoadInB

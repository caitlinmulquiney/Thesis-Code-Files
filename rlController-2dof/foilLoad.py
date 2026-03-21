import numpy as np
from Rbn import Rbn

def foilLoad(eta, nu, foil, wind, wave):
    """
    Inputs:
    - eta and nu (Fossen notation)
    - foil: dict from loadFoilDescription()
    - wind, wave: dict-like objects (only wind used for now)
    """

    # Densities [kg/m**3]
    rho_air = 1.225
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
            sigma = 0.0

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

    aoa_deg = foilAngleOfAttack * 180.0 / np.pi
    # --------------------------------------------------
    # Lift and drag
    # --------------------------------------------------
    if foil["type"] == "sail":
        n_sections = 10
        s = np.linspace(0, 1, n_sections)
        local_aoa_deg = aoa_deg - foil["twist"] * (180/np.pi) * s**foil["twist_exponent"]
        ds = foil["span"] / (n_sections - 1)
        beta_deg = foil["beta"] * (180/np.pi)

        # Lift coeff only correct if aoa and beta are the same !!!
        liftCoeff = 0.1*beta_deg + 1.5*np.tanh(0.09 * local_aoa_deg)
        dragCoeff = (0.0007 + 0.00001 * beta_deg**2) * local_aoa_deg**2

        dL = 0.5 * rho_air * liftCoeff * foilRelativeSpeed**2 * foil["chord"] * ds
        dD = 0.5 * rho_air * dragCoeff * foilRelativeSpeed**2 * foil["chord"] * ds

        lift = np.sum(dL)
        drag = np.sum(dD)

    else:
        liftCoeff = 1.5 * np.tanh(0.09 * aoa_deg)
        dragCoeff = 6e-5 * aoa_deg**2 + 5e-7 * aoa_deg + 0.0051

        if foil["type"] in ("starboard T foil", "port T foil", "starboard L foil horizontal part"):
            center_pos = eta[:3] + Rbn(eta) @ foil["positionInB"]
            height_chord_ratio = center_pos[2] / foil["chord"] # index 2, not 3
            Lift_fs = 1   / (1 + np.exp(-1.5 * height_chord_ratio + 0.8))
            Drag_fs = 1.2 / (1 + np.exp(-1.6 * height_chord_ratio + 0.4))
        else:
            Lift_fs = 1.0
            Drag_fs = 1.0

        lift = 0.5 * sigma * rho_water * liftCoeff * foilRelativeSpeed**2 * foil["chord"] * foil["span"] * Lift_fs
        drag = 0.5 * sigma * rho_water * dragCoeff * foilRelativeSpeed**2 * foil["chord"] * foil["span"] * Drag_fs

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

    if foil["type"] == "sail":
        coe_height = foil["span"] - foil["span"] / foil.get("twist_exponent", 1)
        if foil.get("twist_exponent", 1) < 0:
            coe_height = foil.get("twist_exponent", 1)
        moment_arm = np.array([foil["positionInB"][0], foil["positionInB"][1], -coe_height])
        foilLoadInB = np.hstack((
            foilForceInB,
            np.cross(moment_arm, foilForceInB)
        ))
    else:
        foilLoadInB = np.hstack((
            foilForceInB,
            np.cross(foil["positionInB"], foilForceInB)
            #np.cross(foil["positionInB"] + np.random.uniform(-0.2, 0.2), foilForceInB)
        ))

    return foilLoadInB
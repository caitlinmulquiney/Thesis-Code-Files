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
        local_aoa_deg = aoa_deg - foil.get("twist", 0) * 180.0 / np.pi * s ** foil.get("twist_exponent", 1)
        lift_total = 0.0
        drag_total = 0.0
        ds = foil["span"] / (n_sections - 1)

        for i in range(n_sections):
            aoa_i = local_aoa_deg[i]
            beta_deg = foil.get("beta", 0) * 180.0 / np.pi

            liftCoeff_i = (0.1 * abs(beta_deg) + 0.1 * abs(aoa_i)) / 2.0
            dragCoeff_i = (0.0007 + 0.00012 * abs(beta_deg)) * aoa_i ** 2

            # Use local aoa_i for stall check (fix from MATLAB)
            if abs(aoa_i) > (14 - 0.4 * abs(beta_deg)):
                liftCoeff_i = 5 - 0.3 * abs(aoa_i)

            if aoa_i < 0:
                liftCoeff_i = -liftCoeff_i

            dL = 0.5 * rho_air * liftCoeff_i * foilRelativeSpeed ** 2 * foil["chord"] * ds
            dD = 0.5 * rho_air * dragCoeff_i * foilRelativeSpeed ** 2 * foil["chord"] * ds

            lift_total += dL
            drag_total += dD

        lift = lift_total
        drag = drag_total

    else:
        liftCoeff = 0.09 * aoa_deg - 0.0002
        dragCoeff = 6e-5 * aoa_deg ** 2 + 5e-7 * abs(aoa_deg) + 0.0051

        if abs(aoa_deg) > 15:
            liftCoeff = max(0, 5 - 0.25 * abs(aoa_deg))

        # Free surface effect
        if foil["type"] in ("starboard T foil", "port T foil", "starboard L foil horizontal part"):
            center_pos =  eta[0:3] + Rbn(eta) @ foil["positionInB"]
            height_chord_ratio = center_pos[2]/foil["chord"]
            if (center_pos[2] > 0):
                Lift_fs = np.minimum(1.0,0.5*np.log10(height_chord_ratio+0.1)+0.75)
                Drag_fs = np.minimum(1.0,0.5*np.log10(height_chord_ratio+0.1)+0.9)
            else:
                Lift_fs = 0
                Drag_fs = 0
        else:
            Lift_fs = 1.0
            Drag_fs = 1.0

        if foilStatus == "inAir":
            lift = 0.5 * rho_air * liftCoeff * foilRelativeSpeed ** 2 * foil["chord"] * foil["span"] * Lift_fs
            drag = 0.5 * rho_air * dragCoeff * foilRelativeSpeed ** 2 * foil["chord"] * foil["span"] * Drag_fs
        else:
            lift = 0.5 * sigma * rho_water * liftCoeff * foilRelativeSpeed ** 2 * foil["chord"] * foil["span"] * Lift_fs
            drag = 0.5 * sigma * rho_water * dragCoeff * foilRelativeSpeed ** 2 * foil["chord"] * foil["span"] * Drag_fs

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
        ))

    return foilLoadInB
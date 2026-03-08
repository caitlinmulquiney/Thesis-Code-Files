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

    if foil["type"] == "sail":
        if abs(aoa_deg) < 28:
            liftCoeff = -1e-6 * abs(aoa_deg)**2 + 0.0927 * abs(aoa_deg) + 0.0099
        else:
            liftCoeff = -0.0175 * abs(aoa_deg) + 3.0

        dragCoeff = 0.7 * (
            0.00006 * abs(aoa_deg)**2 - 0.0014 * abs(aoa_deg) + 0.0097
        )
    
        if aoa_deg < 0:
            liftCoeff = -liftCoeff
            dragCoeff = -dragCoeff
    else:
        if (aoa_deg > 15):
            aoa_deg = 15
        
        if (aoa_deg < -15):
            aoa_deg = -15
            
        liftCoeff = 0.1083 * aoa_deg - 0.0002
        dragCoeff = 6e-05 * aoa_deg**2 + 5e-07 * aoa_deg + 0.0051
    # --------------------------------------------------
    # Lift and drag forces
    # --------------------------------------------------
    
    if foilStatus == "inAir":
        if abs(aoa_deg) < 28:
            liftCoeff = -1e-6 * abs(aoa_deg)**2 + 0.0927 * abs(aoa_deg) + 0.0099
        else:
            liftCoeff = -0.0175 * abs(aoa_deg) + 3.0

        dragCoeff = 0.7 * (
            0.00006 * abs(aoa_deg)**2 - 0.0014 * abs(aoa_deg) + 0.0097
        )
        liftCoeff = 0.1*foil["beta"] + 0.1*abs(aoa_deg)
        dragCoeff = (0.0007+0.00012*foil["beta"])*aoa_deg**2
    
        if aoa_deg < 0:
            liftCoeff = -liftCoeff
            dragCoeff = -dragCoeff
        lift = 0.5 * rho_air * liftCoeff * foilRelativeSpeed**2 * foil["chord"] * foil["span"]
        drag = 0.5 * rho_air * dragCoeff * foilRelativeSpeed**2 * foil["chord"] * foil["span"]
    else:
        # Free Surface Effect
        if foil["type"] in ("starboard T foil", "port T foil", "starboard L foil horizontal part"):
            center_pos =  (eta[0:3]
                + Rbn(eta)
                @ (
                    foil["positionInB"]
                )
            )
            height_chord_ratio = center_pos[2]/foil["chord"]
            Lift_fs = np.minimum(1.0,0.5*np.log10(height_chord_ratio+0.1)+0.75)
            Drag_fs = np.minimum(1.0,0.5*np.log10(height_chord_ratio+0.1)+0.9)
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

    foilLoadInB = np.hstack((
        foilForceInB,
        np.cross(foil["positionInB"], foilForceInB)
    ))

    return foilLoadInB

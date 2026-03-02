from vpython import *
import numpy as np
from Rbn import Rbn 
from loadFoilDescription import loadFoilDescription
def initalise():
    scene.width = 1000
    scene.height = 1000
    #scene.background = vector(0.5, 0.7, 1)  # light blue sky
    scene.forward = vector(0, -1, 0)   # Look along negative x
    scene.up = vector(0, 0, -1)        # Make z axis point upward
    scene.center = vector(0, 0, -5)
    scene.range = 30
   
    # Sky: just set the background color (covers z < 0)
    scene.background = vector(0.5, 0.7, 1)  # light blue sky
    water = box(
        pos=vector(0,0,25),
        size=vector(100,50,50),
        color=vector(0,0.3,0.8),
        opacity=0.5
    )
    rate(30)
# -----------------------------
# Main draw function
# -----------------------------
def drawBoat(eta, foils, wind):
    # Parameters
    hullLength = 14.656
    hullSeparation = 3.744 * 2
    COGpositionInB = np.array([0, 0.07904, -3.86752])

    R = Rbn(eta)
    pos = eta[0:3]

    def to_world(pB):
        return pos + R @ pB

    # -----------------------------
    # Draw starboard hull
    # -----------------------------
    p1 = to_world(np.array([hullLength-5.75, hullSeparation/2, 0]))
    p2 = to_world(np.array([-5.75, hullSeparation/2, 0]))

    curve(pos=[vector(*p1), vector(*p2)], color=color.red, radius=0.1)

    # -----------------------------
    # Draw port hull
    # -----------------------------
    p1 = to_world(np.array([hullLength-5.75, -hullSeparation/2, 0]))
    p2 = to_world(np.array([-5.75, -hullSeparation/2, 0]))

    curve(pos=[vector(*p1), vector(*p2)], color=color.red, radius=0.1)

    # -----------------------------
    # Draw foils
    # -----------------------------
    for idx, foil in enumerate(foils):
        foilAtt = foil["attitudeInB"]
        ftype = foil["type"].lower()
        if "l foil" in ftype:
            foilColor = color.cyan
        elif "t foil" in ftype:
            foilColor = color.magenta
        elif "rudder" in ftype:
            foilColor = color.yellow
        else:
            foilColor = color.blue

        if "sail" in ftype:
            foilColor = color.green
            Rfoil = Rbn(np.hstack((np.zeros(3), foilAtt)))
            mastBase = np.array([0, 0, 0])
            sailBase = np.array([-4.5, 0, 0])
            mastBaseW = to_world(np.array([1.3, 0, 0]))
            sailBaseW = to_world(np.array([-4.5, 0, 0]))
            mastTop = to_world(mastBase + np.array([0, 0, -22.5]))
            sailTop = to_world(sailBase + np.array([4.5 - 1.8, 0, -22.5]))
            jibBaseFore = to_world(np.array([hullLength - 2.7 - 5.75, 0, 0]))
            jibBaseAft = to_world(np.array([hullLength - 7.35 - 5.75, 0, 0]))
            sailJibTop = to_world(jibBaseAft + np.array([0, 0, -13.6]))
            v1 = vertex(pos=vector(*mastBaseW), color=foilColor)
            v2 = vertex(pos=vector(*sailBaseW), color=foilColor)
            v3 = vertex(pos=vector(*mastTop), color=foilColor)
            v4 = vertex(pos=vector(*sailTop), color=foilColor)
            quad(vs = [v1, v3, v4, v2])
            v5 = vertex(pos=vector(*jibBaseFore), color=foilColor)
            v6 = vertex(pos=vector(*sailJibTop), color=foilColor)
            v7 = vertex(pos=vector(*jibBaseAft), color=foilColor)
            triangle(vs = [v5, v6, v7])


        else:
            foilPos = foil["positionInB"]
            span = foil["span"]
            Rfoil = Rbn(np.hstack((np.zeros(3), foilAtt)))
            p1 = to_world(foilPos + Rfoil @ np.array([0, -span/2, 0]))
            p2 = to_world(foilPos + Rfoil @ np.array([0,  span/2, 0]))

            curve(pos=[vector(*p1), vector(*p2)],
                color=foilColor,
                radius=0.07)

    # -----------------------------
    # Draw COG
    # -----------------------------
    pCOG = to_world(COGpositionInB)

    sphere(pos=vector(*pCOG),
           radius=0.3,
           color=color.red)
    
    # -----------------------------
    # Draw wind arrow
    # -----------------------------
    windScale = 5
    windDir = wind["direction"]

    windVec = windScale * np.array([
        np.cos(windDir),
        np.sin(windDir),
        0
    ])

    pWind = to_world(np.array([hullLength/2 + 5, 0, -5]))

    arrow(pos=vector(*pWind),
          axis=vector(*windVec),
          shaftwidth=0.3,
          color=color.black)

    label(pos=vector(*(pWind + 0.5*windVec)),
          text=f"Wind {wind['speedInN']:.1f} m/s",
          box=False,
          height=10)

    # -----------------------------
    # Title
    # -----------------------------
    scene.title = (
        f"Heel={np.degrees(eta[3]):.1f}°, "
        f"Pitch={np.degrees(eta[4]):.1f}°, "
        f"Heave={eta[2]:.2f} m"
    )


# -----------------------------
# Example usage loop
# -----------------------------
if __name__ == "__main__":

    scene.width = 1000
    scene.height = 1000
    #scene.background = vector(0.5, 0.7, 1)  # light blue sky
    scene.forward = vector(0, -1, 0)   # Look along negative x
    scene.up = vector(0, 0, -1)        # Make z axis point upward
    scene.center = vector(0, 0, -5)
    scene.range = 30
   
    # Sky: just set the background color (covers z < 0)
    scene.background = vector(0.5, 0.7, 1)  # light blue sky
    water = box(
        pos=vector(0,0,25),
        size=vector(100,50,50),
        color=vector(0,0.3,0.8),
        opacity=0.5
    )


    eta = np.array([0, 0, -1.3, 0.1, 0.05, 0.0])
    nu = np.zeros(6)

    foils = loadFoilDescription()

    wind = {"speedInN": 9.231, "direction": 30 * np.pi / 180}

    while True:
        rate(30)
        drawBoat(eta, nu, foils, wind)

        # Dummy motion example
        #eta[3] += 0.01  # roll oscillation
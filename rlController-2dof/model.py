from vpython import *
import numpy as np
from Rbn import Rbn 
from loadFoilDescription import loadFoilDescription

class Boat: 
    def __init__(self):
        self.objects = {}
        scene.width = 1000
        scene.height = 1000
        #scene.background = vector(0.5, 0.7, 1)  # light blue sky
        scene.forward = vector(0, -1, 0)   # Look along negative x
        scene.up = vector(0, 0, -1)        # Make z axis point upward
        scene.center = vector(0, 0, -5)
        scene.range = 20
    
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
    def drawBoat(self, eta, foils, wind):
        for key, obj in list(self.objects.items()):
            print(f"Clearing key={key}, type={type(obj)}")
            obj.visible = False 
            del obj
        # Parameters
        hullLength = 14.656
        hullSeparation = 3.744 * 2

        R = Rbn(eta)
        pos = eta[0:3]

        def to_world(pB):
            return pos + R @ pB

        # -----------------------------
        # Draw starboard hull
        # -----------------------------
        p1 = to_world(np.array([hullLength-5.75, hullSeparation/2, 0]))
        p2 = to_world(np.array([-5.75, hullSeparation/2, 0]))

        self.objects["star_hull"] = curve(pos=[vector(*p1), vector(*p2)], color=color.red, radius=0.1)

        # -----------------------------
        # Draw port hull
        # -----------------------------
        p1 = to_world(np.array([hullLength-5.75, -hullSeparation/2, 0]))
        p2 = to_world(np.array([-5.75, -hullSeparation/2, 0]))

        self.objects["port_hull"] = curve(pos=[vector(*p1), vector(*p2)], color=color.red, radius=0.1)

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
                mastBase = np.array([1.3, 0, 0])
                mastBaseW = to_world(mastBase)
                sailBase = to_world(mastBase + Rfoil @ np.array([-4.5, 0, 0]))
                mastTop = to_world(mastBase + Rfoil @ np.array([0, 22.5, 0]))
                sailTop = to_world(mastBase + Rfoil @ np.array([-2.7, 22.5, 0]))
                jibBase = np.array([hullLength - 2.7 - 5.75, 0, 0])
                jibBaseFore = to_world(jibBase)
                jibBaseAft = to_world(jibBase + Rfoil @ np.array([-4.65, 0, 0]))
                sailJibTop = to_world(jibBase + Rfoil @ np.array([-4.65, 13.6, 0]))
                self.objects["main_v1"] = vertex(pos=vector(*mastBaseW), color=foilColor)
                self.objects["main_v2"] = vertex(pos=vector(*sailBase), color=foilColor)
                self.objects["main_v3"] = vertex(pos=vector(*mastTop), color=foilColor)
                self.objects["main_v4"] = vertex(pos=vector(*sailTop), color=foilColor)
                self.objects["main"] = quad(vs = [self.objects["main_v1"], self.objects["main_v3"], self.objects["main_v4"], self.objects["main_v2"]])
                self.objects["jib_v1"] = vertex(pos=vector(*jibBaseFore), color=foilColor)
                self.objects["jib_v2"]  = vertex(pos=vector(*sailJibTop), color=foilColor)
                self.objects["jib_v3"]  = vertex(pos=vector(*jibBaseAft), color=foilColor)
                self.objects["jib"] = triangle(vs = [self.objects["jib_v1"], self.objects["jib_v2"] , self.objects["jib_v3"] ])


            else:
                foilPos = foil["positionInB"]
                span = foil["span"]
                Rfoil = Rbn(np.hstack((np.zeros(3), foilAtt)))
                p1 = to_world(foilPos + Rfoil @ np.array([0, -span/2, 0]))
                p2 = to_world(foilPos + Rfoil @ np.array([0,  span/2, 0]))

                self.objects[ftype] = curve(pos=[vector(*p1), vector(*p2)],
                    color=foilColor,
                    radius=0.07)
        
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

        self.objects["wind_arrow"] = arrow(pos=vector(*pWind),
            axis=vector(*windVec),
            shaftwidth=0.3,
            color=color.black)

        self.objects["wind_label"] = label(pos=vector(*(pWind + 0.5*windVec)),
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
    # Main draw function
    # -----------------------------
    def updateBoat(self, eta, foils, wind):
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

        self.objects["star_hull"].modify(0, pos=vector(*p1))
        self.objects["star_hull"].modify(1, pos=vector(*p2))

        # -----------------------------
        # Draw port hull
        # -----------------------------
        p1 = to_world(np.array([hullLength-5.75, -hullSeparation/2, 0]))
        p2 = to_world(np.array([-5.75, -hullSeparation/2, 0]))

        self.objects["port_hull"].modify(0, pos=vector(*p1))
        self.objects["port_hull"].modify(1, pos=vector(*p2))

        # -----------------------------
        # Draw foils
        # -----------------------------
        for idx, foil in enumerate(foils):
            foilAtt = foil["attitudeInB"]
            ftype = foil["type"].lower()
            Rfoil = Rbn(np.hstack((np.zeros(3), foilAtt)))
            if "sail" in ftype:
                mastBase = np.array([1.3, 0, 0])
                mastBaseW = to_world(mastBase)
                sailBase = to_world(mastBase + Rfoil @ np.array([-4.5, 0, 0]))
                mastTop = to_world(mastBase + Rfoil @ np.array([0, 22.5, 0]))
                sailTop = to_world(mastBase + Rfoil @ np.array([-2.7, 22.5, 0]))
                jibBase = np.array([hullLength - 2.7 - 5.75, 0, 0])
                jibBaseFore = to_world(jibBase)
                jibBaseAft = to_world(jibBase + Rfoil @ np.array([-4.65, 0, 0]))
                sailJibTop = to_world(jibBase + Rfoil @ np.array([-4.65, 13.6, 0]))
                self.objects["main_v1"].pos = vector(*mastBaseW)
                self.objects["main_v2"].pos = vector(*sailBase)
                self.objects["main_v3"].pos = vector(*mastTop)
                self.objects["main_v4"].pos = vector(*sailTop)

                self.objects["jib_v1"].pos = vector(*jibBaseFore)
                self.objects["jib_v2"].pos = vector(*sailJibTop)
                self.objects["jib_v3"].pos = vector(*jibBaseAft)


            else:
                foilPos = foil["positionInB"]
                span = foil["span"]
                p1 = to_world(foilPos + Rfoil @ np.array([0, -span/2, 0]))
                p2 = to_world(foilPos + Rfoil @ np.array([0,  span/2, 0]))

                self.objects[ftype].modify(0, pos=vector(*p1))
                self.objects[ftype].modify(1, pos=vector(*p2))
        
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

        self.objects["wind_arrow"].pos = vector(*pWind)
        self.objects["wind_arrow"].axis = vector(*windVec)

        self.objects["wind_label"].pos = vector(*(pWind + 0.5*windVec))
        self.objects["wind_label"].text = f"Wind {wind['speedInN']:.1f} m/s"

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
    boat = Boat()
    eta = np.array([0, 0, -2.5, 0.1, 0.05, 0.0])
    nu = np.zeros(6)

    foils = loadFoilDescription()

    wind = {"speedInN": 9.231, "direction": 30 * np.pi / 180}
    boat.drawBoat(eta, foils, wind)

    while True:
        rate(30)
        # foils[0]["attitudeInB"][2] += 1*np.pi/180
        # boat.updateBoat(eta, foils, wind)

        # Dummy motion example
        #eta[3] += 0.01  # roll oscillation
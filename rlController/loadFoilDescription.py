import numpy as np

def loadFoilDescription():
    # % Returns a structure containing the description of the foils: sail,
    # % rudders, T-foils, and the L-foil.
    # % NB: only the foils of interest when the wind comes from port are described
    # % And apparently we should also add the central rudder
    # % 
    # % A coordinate system {f} is attached to each foil
    # % Origin F: center of the foil (mid-chord, mid-span)
    # % f1 is along the chord
    # % f2 is along the span
    # % f3 is orthogonal to the foil

    # % In the structure below, one gives the position of F, the chord and span [m], 
    # % and the attitude of {f} in {b}, which defines, among other the angle of
    # % attack

    # % NB: these values are not exactly consistent with the ones for Macif
    # % (I played a bit with them to get reasonably close to static equiilibrium)

    # % Dihedron causes starboard floater to be .92 (10.5*sin(5)) m above
    # % central floater, and 5deg tilted
    # %% sail
    foil_1 = {
        "type": "sail",
        "positionInB": np.array([-1.3, -10*np.sin(np.deg2rad(5)), -10.62]),
        "attitudeInB": np.array([-85, 0, -20]) * np.pi/180,
        "chord": 13,
        "span": 22
    }

    # %% L foil on the starboard floater (horizontal part)
    foil_2 = {
        "type": "starboard L foil horizontal part",
        "positionInB": np.array([1.85,3.2,2.6-0.935]),
        # Paper numbers
        # Real numbers were: [-5+20;3.3;0] * pi/180
        "attitudeInB": np.array([0, 4.5, 0]) * np.pi/180,
        "chord": 0.4,
        "span": 2
    }

    # %% L foil on the starboard floater (vertical part)
    foil_3 = {
        "type": "starboard L foil vertical part",
        "positionInB": np.array([1.85, 4.1, 0.935]),
        # Paper numbers
        # Real numbers were: [-5 + 105, 3.5, 0] * pi/180
        "attitudeInB": np.array([85, 4.5, 0]) * np.pi/180,
        "chord": 0.4,
        "span": 2
    }

    # %% rudder + foil on the starboard
    foil_4 = {
        "type": "starboard T foil",
        "positionInB": np.array([-5.75, 3.744, 0.96704 + 0.944]),
        # Paper numbers
        # Real numbers were: [-5, 1, 1.4] * pi/180
        "attitudeInB": np.array([0, -5.1, 1.6]) * np.pi/180,
        "chord": 0.2,
        "span": 1.2
    }

    foil_5 = {
        "type": "starboard rudder",
        "positionInB": np.array([-5.75, 3.744, 0.96704]),
        # Paper numbers
        # Real numbers were: [-5 + 90, 0, 1.4] * pi/180
        "attitudeInB": np.array([90, 0, 1.6]) * np.pi/180,
        "chord": 0.2,
        "span": 2.032
    }

    # %% rudder + foil on the port
    foil_6 = {
        "type": "port T foil",
        "positionInB": np.array([-5.75, -3.744, 0.96704 + 0.944]),
        # Paper numbers
        # Real numbers were: [-5, 0.9, 1.4] * pi/180
        "attitudeInB": np.array([0, 1.6, 1.6]) * np.pi/180,
        "chord": 0.2,
        "span": 1.2
    }

    foil_7 = {
        "type": "port rudder",
        "positionInB": np.array([-5.75, -3.744, 0.96704]),
        # Paper numbers
        # Real numbers were: [-5 + 90, 0, 1.4] * pi/180
        "attitudeInB": np.array([90, 0, 1.6]) * np.pi/180,
        "chord": 0.2,
        "span": 2.032
    }

    return np.array([foil_1, foil_2, foil_3, foil_4, foil_5, foil_6, foil_7])

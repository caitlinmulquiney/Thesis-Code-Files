import numpy as np
from Rbn import Rbn 
# Very rough model of the air friction on the hulls, lateral beams, mast, etc... 
# Lift should be added as the new designs have been optimized to generate lift (issue when the heel is 15 deg...)


def aerodynamicLoadSuperstructure(eta,nu,wind):
    Cd = 1
    A = np.pi/4*(2.5**2+1.5**2+1.5**2) + 25*.2 + .5*8.6 # hulls + mast + beams
    rho_air = 1
    newref = np.array([5,0,-1])
    #newref = np.array([0,0,0])
    applicationPoint = np.array([5.92896,0,-12])-newref
    windVelocityInN = np.array([wind["speedInN"]*np.cos(wind["direction"]),wind["speedInN"]*np.sin(wind["direction"]),0])
    flowLinearVelocityInB = Rbn(eta) @ windVelocityInN
    relativeLinearVelocityInB = nu[:3] - flowLinearVelocityInB
    aeroForceInB = np.array([-1/2*rho_air*Cd*A*np.square(relativeLinearVelocityInB[0]),0,0])
    aeroLoadInB = np.hstack((aeroForceInB, np.cross(applicationPoint, aeroForceInB)))
    return aeroLoadInB

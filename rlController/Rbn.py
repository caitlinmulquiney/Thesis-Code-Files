import numpy as np

def Rbn(eta):
   phi = eta[3]
   theta = eta[4]
   psi= eta[5]
   cphi = np.cos(phi)
   sphi = np.sin(phi)
   cth  = np.cos(theta)
   sth  = np.sin(theta)
   cpsi = np.cos(psi)
   spsi = np.sin(psi)
 
   R = np.array([
      [cpsi*cth,-spsi*cphi+cpsi*sth*sphi,spsi*sphi+cpsi*cphi*sth],
      [spsi*cth,cpsi*cphi+sphi*sth*spsi,-cpsi*sphi+sth*spsi*cphi],
      [-sth, cth*sphi, cth*cphi]
   ])
   return R

function R = Rbn(eta)
phi = eta(4);
theta = eta(5);
psi= eta(6);
cphi = cos(phi);
sphi = sin(phi);
cth  = cos(theta);
sth  = sin(theta);
cpsi = cos(psi);
spsi = sin(psi);
 
R = [...
   cpsi*cth  -spsi*cphi+cpsi*sth*sphi  spsi*sphi+cpsi*cphi*sth
   spsi*cth  cpsi*cphi+sphi*sth*spsi   -cpsi*sphi+sth*spsi*cphi
   -sth      cth*sphi                  cth*cphi ];

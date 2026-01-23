function T = Tbn(eta)
phi = eta(4);
theta = eta(5);
cphi = cos(phi);
sphi = sin(phi);
cth  = cos(theta);
sth  = sin(theta);
tth = sth/cth;
T = [   1   sphi*tth    cphi*tth;
        0   cphi        -sphi;
        0   sphi/cth    cphi/cth];
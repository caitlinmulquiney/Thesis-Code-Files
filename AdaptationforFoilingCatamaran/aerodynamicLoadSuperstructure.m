function aeroLoadInB = aerodynamicLoadSuperstructure(eta,nu,wind,verbose)
% Very rough model of the air friction on the hulls,
% lateral beams, mast, etc... 
% Lift should be added as the new designs have been optimized to generate
% lift (issue when the heel is 15 deg...)
Cd = 1;
A = pi/4*(2.5^2+1.5^2+1.5^2) ...%hulls part 
    + 25*.2 ... %mast 
    + .5*8.6; %  beams
rho_air=1;
newref = [5;0;-1];
newref = [0;0;0];
applicationPoint = [5.92896;0;-12]-newref;

windVelocityInN = [wind.speedInN*cos(wind.direction);wind.speedInN*sin(wind.direction);0];
flowLinearVelocityInB = Rbn(eta).'*windVelocityInN;
relativeLinearVelocityInB = nu(1:3) - flowLinearVelocityInB;

aeroForceInB = [-1/2*rho_air*Cd*A*relativeLinearVelocityInB(1)^2;0;0];
aeroLoadInB  = [aeroForceInB;cross(applicationPoint,aeroForceInB)];

if verbose
    fprintf(1,'a  |                                                | %+10.1f %+10.1f %+10.1f | %+10.1f %+10.1f %+10.1f | \n',aeroLoadInB.'/1000);
end
end
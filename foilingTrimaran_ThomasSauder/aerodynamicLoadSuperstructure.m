function aeroLoadInB = aerodynamicLoadSuperstructure(eta,nu,wind,verbose)
% Very rough model of the air friction on the hulls,
% lateral beams, mast, etc... 
% Lift should be added as the new designs have been optimized to generate
% lift (issue when the heel is 15 deg...)
Cd = 1;
A = pi/4*(5^2+3^2+3^2) ...%hulls part 
    + 20*.5 ... %mast 
    + .5*30; %  beams
rho_air=1;
applicationPoint = [0;0;-10];

windVelocityInN = [wind.speedInN*cos(wind.direction);wind.speedInN*sin(wind.direction);0];
flowLinearVelocityInB = Rbn(eta).'*windVelocityInN;
relativeLinearVelocityInB = nu(1:3) - flowLinearVelocityInB;

aeroForceInB = [-1/2*rho_air*Cd*A*relativeLinearVelocityInB(1)^2;0;0];
aeroLoadInB  = [aeroForceInB;cross(applicationPoint,aeroForceInB)];

% if verbose
%     fprintf(1,'a  |                                                | %+10.1f %+10.1f %+10.1f | %+10.1f %+10.1f %+10.1f | \n',aeroLoadInB.'/1000);
% end
end
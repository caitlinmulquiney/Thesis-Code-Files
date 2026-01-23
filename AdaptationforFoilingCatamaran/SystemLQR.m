function [F] = System(t, F, K, A, B, foil)
global uMatrix;
eta0 = F(1:6);
nu0 = F(7:12);

verbose = false;
wind.speedInN = 9.231; % wind speed in m/s

wind.direction = 30*pi/180; %(propagation direction, positive=wind from port side) 60deg= "-120" in classical terms
wave=[]; % no waves for now, but it will come

%% Controller 2
J = configurationMatrix(foil,eta0,nu0,wind,wave);
etaD =[-1.3;2.6*pi/180;-0.5*pi/180;-0.8*pi/180];
nuD = [0; 0; 0; 0];

if t >= 50
    wind.speedInN = 6.17;
end
etadot = Jbn(eta0)*nu0;
e = etaD - eta0(3:6);
e_dot = nuD - nu0(3:6); %nuD - nu0(3:6)
u = K(:,3:6)*e + K(:,9:12)*e_dot;
% tauN = [0; 0; tau];
% pinv(J)
% u = pinv(J)*tauN;

Fdot = A*F+B*u;
F = [etadot; Fdot(7:12)];

%% System Logic
%saturate actuator angles (radians)
u = max([ -10; -10; -10; -10; -10]*pi/180, ...
    min([  10; 10; 10; 10; 10]*pi/180, u));

% totalLoad = zeros(6,1);
% 
% foil{2}.attitudeInB = foil{2}.attitudeInB + [0; u(1); 0];
% totalLoad = totalLoad + foilLoad(eta0,nu0,foil{2},wind,wave,false);
% foil{3}.attitudeInB = foil{3}.attitudeInB + [0; u(1); 0];
% totalLoad = totalLoad + foilLoad(eta0,nu0,foil{2},wind,wave,false);
% foil{4}.attitudeInB = foil{4}.attitudeInB + [0; u(2); u(3)];
% totalLoad = totalLoad + foilLoad(eta0,nu0,foil{4},wind,wave,false);
% foil{5}.attitudeInB = foil{5}.attitudeInB + [0; 0; u(3)];
% totalLoad = totalLoad + foilLoad(eta0,nu0,foil{5},wind,wave,false);
% foil{6}.attitudeInB = foil{6}.attitudeInB + [0; u(4); u(3)];
% totalLoad = totalLoad + foilLoad(eta0,nu0,foil{6},wind,wave,false);
% foil{7}.attitudeInB = foil{7}.attitudeInB + [0; 0; u(3)];
% totalLoad = totalLoad + foilLoad(eta0,nu0,foil{7},wind,wave,false);
% 
% totalLoad = totalLoad + foilLoad(eta0,nu0,foil{1},wind,wave,false);
% 
% totalLoad= totalLoad + weightLoad(eta0,false);
% totalLoad = totalLoad + aerodynamicLoadSuperstructure(eta0,nu0,wind,false);

foilList = [2 4 5 6 7];
otherFoilList = [1 3];

%u = [ 0; 0; 0; 0;5*pi/180;];
totalLoad = zeros(6,1);
for idx = 1:length(foilList)
    if foilList(idx) == 5 || foilList(idx) == 7
        dFoilAngle = [0;0;u(idx)]; %for this foil (rudder), the control input is the "yaw" of the foil
    else 
        dFoilAngle = [0;u(idx);0]; %for the other ones, it is the "pitch"
    end
    foil{foilList(idx)}.attitudeInB = foil{foilList(idx)}.attitudeInB + dFoilAngle;
    totalLoad = totalLoad + foilLoad(eta0,nu0,foil{foilList(idx)},wind,wave,verbose);
end

for idx = 1:length(otherFoilList)
    totalLoad = totalLoad + foilLoad(eta0,nu0,foil{otherFoilList(idx)},wind,wave,verbose);
end

totalLoad= totalLoad + weightLoad(eta0,verbose);
totalLoad = totalLoad + aerodynamicLoadSuperstructure(eta0,nu0,wind,verbose);

M = massDistribTrimaran(verbose);
C = coriolisCentripetal(M,nu0);

nud = M^(-1)*(totalLoad - C*nu0);
% nud = max([ -0.5; -0.5; -0.5; -0.5; -0.5; -0.5], ...
    % min([  0.5; 0.5; 0.5; 0.5; 0.5; 0.5], nud));
etadot = max([ -0.5; -0.5; -0.5; -0.5; -0.5; -0.5], ...
    min([  0.5; 0.5; 0.5; 0.5; 0.5; 0.5], etadot));
F = [etadot; nud];
uMatrix = [uMatrix; t,u'];
t
return

function [F] = System(t, F, K, A, B, foil)
global uMatrix;
eta0 = F(1:6);
nu0 = F(7:12);

verbose = false;
wind.speedInN = 9.231; % wind speed in m/s

wind.direction = 45*pi/180; %(propagation direction, positive=wind from port side) 60deg= "-120" in classical terms
wave=[]; % no waves for now, but it will come

%% Controller 2
J = configurationMatrix(foil,eta0,nu0,wind,wave);
etaD =[-1.3;2.6*pi/180;-0.5*pi/180;0*pi/180];
nuD = [0; 0; 0; 0];

% if t >= 50
%     wind.speedInN = 6.17;
% end
% 
% if t >= 100
%     wind.speedInN = 8.17;
% end
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
u = max([ -15; -15; -15; -10; -10; -10]*pi/180, ...
    min([  15; 15; 15; 10; 10; 10]*pi/180, u));
% u(1) = 0;
% u(2) = 0;
% u(3) = 0;
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

foilList = [1 2 3 4 5 6 7];

totalLoad = zeros(6,1);

%% --- Apply control inputs ---

% u(1) : sail sheet (yaw)
% u(2) : sail camber (beta)
% u(3) : sail twist
% u(4) : port foil pitch
% u(5) : starboard foil pitch
% u(6) : rudder yaw (foil 5)
% u(7) : rudder yaw (foil 7)
% u(8) : stabilizer pitch (foil 6)

% Sail (foil 1)
% foil{1}.attitudeInB = foil{1}.attitudeInB + [0;0;u(1)];
% foil{1}.beta        = foil{1}.beta + u(2);
foil{1}.twist       = foil{1}.twist + u(6);

% Port foil (foil 2)
foil{2}.attitudeInB = foil{2}.attitudeInB + [0;u(1);0];

% Starboard foil (foil 4)
foil{4}.attitudeInB = foil{4}.attitudeInB + [0;u(2);0];

% Rudder foils
foil{5}.attitudeInB = foil{5}.attitudeInB + [0;0;u(3)];
foil{7}.attitudeInB = foil{7}.attitudeInB + [0;0;u(4)];

% Stabilizer foil
foil{6}.attitudeInB = foil{6}.attitudeInB + [0;u(5);0];

%% --- Compute loads from controlled foils ---
for idx = 1:length(foilList)
    totalLoad = totalLoad + foilLoad(eta0,nu0,foil{foilList(idx)},wind,wave,verbose);
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

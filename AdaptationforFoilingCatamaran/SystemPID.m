function [F] = System(t, F, foil)
% clc;

%{b} is defined with origin at free surface, below mastfoot
global uMatrix;

% Preliminary computations to come close to equilibrium
eta0 = F(1:6);
nu0 = F(7:12);
eta0int = F(13:18);

verbose = false;
wind.speedInN = 9.231; % wind speed in m/s
wind.direction = 30*pi/180; %(propagation direction, positive=wind from port side) 60deg= "-120" in classical terms
wave=[]; % no waves for now, but it will come

%% Controller 1
% Kp1 = 2e3;
% Kp3 = 20e3;
% Kp4 = 2e3;
% Kd1 = 20;
% Kd3 = 0.2e3;
% Kd4 = 20e3;
% 
% % desireds
% z_e     = desZ     - eta0(3);
% theta_e = desTheta - eta0(5);
% psi_e   = desPsi   - eta0(6);
% 
% % proper derivatives
% xyz_dot   = nu0(1:3);                 % [ẋ;ẏ;ż]
% eul_dot   = nu0(4:6);% [φ̇;θ̇;ψ̇]
% z_dot     = xyz_dot(3);
% theta_dot = eul_dot(2);
% psi_dot   = eul_dot(3);
% 
% % thesis PID (outputs are actuator commands, in radians)
% tau1 =  Kp1*z_e     - Kd1*z_dot;        % heave loop -> centerboard T-foil flap
% tau3 =  Kp3*theta_e - Kd3*theta_dot;    % pitch loop  -> rudder T-foils
% tau4 =  Kp4*psi_e   - Kd4*psi_dot;      % yaw loop    -> rudders
% 
% % thesis allocation (five actuators order must match your indices)
% u = [0.5*tau3; 0.5*tau4; tau1; 0.5*tau3; 0.5*tau4];

%% Controller 2
Kd = 2e2*eye(4);
Kd(1,1) = 2e5;
Ki = 2e1*eye(4);
Kp = 3e4*eye(4);
Kp(1,1) = 2e8;
Kp(3,3) = 2e5;
%Kp = [1000 0 0 0; 0 1000 0 0; 0 0 1000 0; 0 0 0 1000];
%Kd = 0;
%Ki = 0;
J = configurationMatrix(foil,eta0,nu0,wind,wave);
etaD = [-1.3;2.6*pi/180;-0.5*pi/180;-0.7*pi/180];
nuD = [0; 0; 0; 0];

etadot = Jbn(eta0)*nu0;
e = etaD - eta0(3:6);
e_dot = nuD - nu0(3:6); %nuD - nu0(3:6)

tau = Kp*(e) + Kd*(-nu0(3:6)) + Ki*(eta0int(3:6));
tauN = [0; 0; tau];
u = pinv(J)*tauN;

%% System Logic
% saturate actuator angles (radians)
u = max([ -20; -20; -20; -20; -20]*pi/180, ...
    min([  20; 20; 20; 20; 20]*pi/180, u));
foilList = [2 4 5 6 7];
otherFoilList = [1 3];

totalLoad = zeros(6,1);
for idx = 1:length(foilList)
    if foilList(idx) == 5 || foilList(idx) == 7
        dFoilAngle = [0;0;u(idx)]; %for this foil (rudder), the control input is the "yaw" of the foil
    else 
        dFoilAngle = [0;u(idx);0]; %for the other ones, it is the "pitch"
    end
    foil{foilList(idx)}.attitudeInB = foil{foilList(idx)}.attitudeInB + dFoilAngle;
    totalLoad = totalLoad + foilLoad(eta0,nu0,foil{foilList(idx)},wind,wave,false);
end

for idx = 1:length(otherFoilList)
    totalLoad = totalLoad + foilLoad(eta0,nu0,foil{otherFoilList(idx)},wind,wave,false);
end

totalLoad= totalLoad + weightLoad(eta0,false);
totalLoad = totalLoad + aerodynamicLoadSuperstructure(eta0,nu0,wind,false);
M = massDistribTrimaran(verbose);
C = coriolisCentripetal(M,nu0);

nud = M^(-1)*(totalLoad - C*nu0);

F = [etadot; nud; [0;0;e]];
uMatrix = [uMatrix; t,u'];
t
return
function [F] = System(t, Fo)
% clc;

%{b} is defined with origin at free surface, below mastfoot


% Preliminary computations to come close to equilibrium


foil=loadFoilDescription;
eta0 = Fo(1:6);
nu0 = Fo(7:12);
eta0int = Fo(13:18);

verbose = false;
desZ = -1.6;
desPhi = 5*pi/180;
desTheta = 2*pi/180;
desPsi = 0;

wind.speedInN = 10; % wind speed in m/s
wind.direction = 60*pi/180; %(propagation direction, positive=wind from port side) 60deg= "-120" in classical terms
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
Kp = [200e6 0 0 0; 0 10e6 0 0; 0 0 0.8e9 0; 0 0 0 1.5e9];
Kd = [2e3 0 0 0; 0 1e3 0 0; 0 0 2e3 0; 0 0 0 2e3];
Ki = [5e6 0 0 0; 0 200e3 0 0; 0 0 20e6 0; 0 0 0 20e6];
J = configurationMatrix(foil,eta0,nu0,wind,wave);
etaD = [-1.6; 4.5*pi/180; 2.3*pi/180; -0.03*pi/180];
U0 = 21.19; 
beta0 = 1.2*pi/180;
vel = Rbn(eta0).'* [U0*cos(beta0);U0*sin(beta0);0.81];
nuD = [vel(3); 0; 0; 0];
etaintD = [0; 0; 0; 0];
tau = Kp*(etaD - eta0(3:6)) + Kd*(nuD - nu0(3:6)) + Ki*(etaintD - eta0int(3:6))
tauN = [0; 0; tau];
u = pinv(J)*tauN;

%% System Logic
% saturate actuator angles (radians)
u = max([ -30; -40; -30; -30; -30]*pi/180, ...
    min([  30;  40;  30;  30;  30]*pi/180, u))
foilList = [4 5 6 8 9];
otherFoilList = [1 2 3 7];

totalLoad = zeros(6,1);
for idx = 1:length(foilList)
    foil_ = foil{foilList(idx)};
    if foilList(idx) == 5 || foilList(idx) == 9
        dFoilAngle = [0;0;u(idx)]; %for this foil (rudder), the control input is the "yaw" of the foil
    else
        dFoilAngle = [0;u(idx);0]; %for the other ones, it is the "pitch"
    end
    angle = [zeros(3,1);foil_.attitudeInB]
    Rbn(angle)*dFoilAngle
    foil_.attitudeInB = foil{foilList(idx)}.attitudeInB + dFoilAngle;
    totalLoad = totalLoad + foilLoad(eta0,nu0,foil_,wind,wave,true);
end

for idx = 1:length(otherFoilList)
    totalLoad = totalLoad + foilLoad(eta0,nu0,foil{otherFoilList(idx)},wind,wave,true);
end

totalLoad= totalLoad + weightLoad(eta0,true);
totalLoad = totalLoad + aerodynamicLoadSuperstructure(eta0,nu0,wind,true);
M = massDistribTrimaran(verbose);
C = coriolisCentripetal(M,nu0);

nud = M^(-1)*(totalLoad - C*nu0);

F = [Jbn(eta0)*nu0; nud; eta0];
return
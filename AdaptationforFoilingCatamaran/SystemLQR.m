function [F] = SystemLQR(t, F, K, A, B, foil, windData, off)
persistent lastUpdateTime currentWind
global uMatrix vmgMatrix eMatrix vmgEMatrix;
eta0 = F(1:6);
nu0 = F(7:12);

verbose = false;
windIndex = round(t,1)*10+1;
if isempty(lastUpdateTime)
    lastUpdateTime = t;
    currentWind.speedInN = 5 + (10-5)*rand();
    currentWind.direction =  75*pi/180 + (105*pi/180-75*pi/180)*rand(); %140*pi/180 + (140*pi/180-140*pi/180)*rand(); %30*pi/180;%
end

if t - lastUpdateTime >= 1
    % foil=loadFoilDescription;
    off = -0.2+0.4*rand();
    lastUpdateTime = t;
    currentWind.speedInN = 5 + (10-5)*rand();
    currentWind.direction =  75*pi/180 + (105*pi/180-75*pi/180)*rand(); %currentWind.direction+1*pi/180;%70*pi/180; %140*pi/180 + (140*pi/180-140*pi/180)*rand(); %currentWind.direction+5*pi/180; %
end
currentWind.speedInN = 8.23;
currentWind.direction = 60*pi/180;
wind = currentWind;
%wind.speedInN = 13.231; %windData(windIndex); % wind speed in m/s
ws  = clip(wind.speedInN, 5, 13);
wd  = clip(wind.direction, 30*pi/180, 150*pi/180);
% 
% wind.direction = 45*pi/180; %(propagation direction, positive=wind from port side) 60deg= "-120" in classical terms
wave=[]; % no waves for now, but it will come

%% Controller 2
% J = configurationMatrix(foil,eta0,nu0,wind,wave);
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
u = max([ -5; -10; -15; -15; -30; -15; -30]*pi/180, ...
    min([  5; 10; 15; 15; 30; 15; 30]*pi/180, u));
boat_speed_knots = sqrt(nu0(2)^2 + nu0(1)^2) * 1.944;
% windVelocityInN = [wind.speedInN * cos(wind.direction); wind.speedInN * sin(wind.direction);0.0;];
% flowLinearVelocityInB = Rbn(eta0).' * windVelocityInN;
% twa = arctan2(flowLinearVelocityInB(2), flowLinearVelocityInB(1));
twa = -pi + wind.direction;
vmg = abs(boat_speed_knots * cos(twa));
target_vmg = -0.00000272*rad2deg(abs(twa))^4+0.001025*rad2deg(abs(twa))^3-0.12778*rad2deg(abs(twa))^2+6.012*rad2deg(abs(twa))-74;
vmgE = vmg-target_vmg;
foilList = [1 2 3 4 5 6 7];

totalLoad = zeros(6,1);

%% --- Apply control inputs ---
foil{1}.beta        = foil{1}.beta + u(1);
foil{1}.twist       = foil{1}.twist + u(2);
foil{2}.attitudeInB = foil{2}.attitudeInB + [0;u(3);0];
foil{4}.attitudeInB = foil{4}.attitudeInB + [0;u(4);0];
foil{5}.attitudeInB = foil{5}.attitudeInB + [0;0;u(5)];
foil{6}.attitudeInB = foil{6}.attitudeInB + [0;u(6);0];
foil{7}.attitudeInB = foil{7}.attitudeInB + [0;0;u(7)];

%% --- Compute loads from controlled foils ---
for idx = 1:length(foilList)
    totalLoad = totalLoad + foilLoad(eta0,nu0,foil{foilList(idx)},wind,wave, off,verbose);
end

totalLoad= totalLoad + weightLoad(eta0,verbose);
totalLoad = totalLoad + aerodynamicLoadSuperstructure(eta0,nu0,wind,verbose);

M = massDistribTrimaran(verbose);
C = coriolisCentripetal(M,nu0);

nud = M^(-1)*(totalLoad - C*nu0);
% nud = max([ -0.5; -0.5; -0.5; -0.5; -0.5; -0.5], ...
    % min([  0.5; 0.5; 0.5; 0.5; 0.5; 0.5], nud));
%etadot = max([ -0.5; -0.5; -0.5; -0.5; -0.5; -0.5], ...
    % min([  0.5; 0.5; 0.5; 0.5; 0.5; 0.5], etadot));
F = [etadot; nud];
uMatrix = [uMatrix; t,u'];
eMatrix = [eMatrix; t, e', e_dot'];
vmgMatrix = [vmgMatrix; t, vmg'];
vmgEMatrix = [vmgEMatrix; t, vmgE'];
fprintf('t=%.2f, wind=%.2f m/s, wd=%.2f rad\n', t, ws, wd);
return

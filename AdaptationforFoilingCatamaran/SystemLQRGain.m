function [F] = SystemLQRGain(t, F, K_grid, A_grid, B_grid, windSpeeds, windDirs, foil0, windData)
persistent lastUpdateTime currentWind
global uMatrix;
eta0 = F(1:6);
nu0 = F(7:12);

verbose = false;
windIndex = round(t,1)*10+1;
if isempty(lastUpdateTime)
    lastUpdateTime = t;
    currentWind.speedInN = 5 + (10-5)*rand();
    currentWind.direction = 45*pi/180 + (75*pi/180-45*pi/180)*rand();
end

if t - lastUpdateTime >= 1
    foil0=loadFoilDescription;
    currentWind.speedInN = 5 + (10-5)*rand();
    currentWind.direction = 45*pi/180 + (75*pi/180-45*pi/180)*rand();
    lastUpdateTime = t;
end
wind = currentWind;
% wind.speedInN = 8.23;
% wind.direction = 60*pi/180;
% windIndex      = round(t,1)*10 + 1;
% windIndex      = min(windIndex, size(windData,1));  % clamp to data length
% wind.speedInN  = windData(windIndex);
wave = [];

% --- Interpolate gain for current wind speed ---
ws  = clip(wind.speedInN, 5, 13);
wd  = clip(wind.direction, 30*pi/180, 150*pi/180);
idxS = find(windSpeeds <= ws, 1, 'last');
idxD = find(windDirs <= wd, 1, 'last');

K = K_grid{idxS, idxD};
% A = (1-alpha) * A_grid{idx} + alpha * A_grid{idx+1};
% B = (1-alpha) * B_grid{idx} + alpha * B_grid{idx+1};

% --- Controller ---
etaD  = [-1.3; 2.6*pi/180; -0.5*pi/180; 0*pi/180];
nuD   = [0; 0; 0; 0];

etadot  = Jbn(eta0) * nu0;
e       = etaD - eta0(3:6);
e_dot   = nuD  - nu0(3:6);
u       = K(:,3:6)*e + K(:,9:12)*e_dot;

% --- Saturate actuators ---
u = max([-5;-10;-15;-15;-30;-15;-30]*pi/180, ...
    min([ 5; 10; 15; 15; 30; 15; 30]*pi/180, u));

% --- Apply control inputs ---
foil = foil0;
foil{1}.beta        = foil0{1}.beta        + u(1);
foil{1}.twist       = foil0{1}.twist       + u(2);
foil{2}.attitudeInB = foil0{2}.attitudeInB + [0; u(3); 0];
foil{4}.attitudeInB = foil0{4}.attitudeInB + [0; u(4); 0];
foil{5}.attitudeInB = foil0{5}.attitudeInB + [0; 0;    u(5)];
foil{6}.attitudeInB = foil0{6}.attitudeInB + [0; u(6); 0];
foil{7}.attitudeInB = foil0{7}.attitudeInB + [0; 0;    u(7)];

% --- Compute loads ---
totalLoad = zeros(6,1);
for i = 1:7
    totalLoad = totalLoad + foilLoad(eta0, nu0, foil{i}, wind, wave, verbose);
end
totalLoad = totalLoad + weightLoad(eta0, verbose);
totalLoad = totalLoad + aerodynamicLoadSuperstructure(eta0, nu0, wind, verbose);

% --- Dynamics ---
M    = massDistribTrimaran(verbose);
C    = coriolisCentripetal(M, nu0);
nud  = M^(-1) * (totalLoad - C*nu0);

F = [etadot; nud];
uMatrix = [uMatrix; t, u'];
fprintf('t=%.2f, wind=%.2f m/s, wd=%.2f rad\n', t, ws, wd);
end
function [F] = SystemLQRGain(t, F, currentWind, foil)
persistent actionQueue lastUpdateTime currentWind
global uMatrix vmgMatrix eMatrix vmgEMatrix;
eta0 = F(1:6);
nu0 = F(7:12);

verbose = false;
windIndex = round(t,1)*10+1;
if isempty(lastUpdateTime)
    lastUpdateTime = t;
    currentWind.speedInN = 7.7 + (8.7-7.7)*rand();
    currentWind.direction =  30*pi/180 + (150*pi/180-30*pi/180)*rand();
end

if t - lastUpdateTime >= 1
    lastUpdateTime = t;
    currentWind.speedInN = 7.7 + (8.7-7.7)*rand();
    currentWind.direction =  30*pi/180 + (150*pi/180-30*pi/180)*rand();
end
wind = currentWind;

% windIndex      = round(t,1)*10 + 1;
% windIndex      = min(windIndex, size(windData,1));  % clamp to data length
% wind.speedInN  = windData(windIndex);
wave = [];

boat_speed_knots = sqrt(nu0(2)^2 + nu0(1)^2) * 1.944;
% windVelocityInN = [wind.speedInN * cos(wind.direction); wind.speedInN * sin(wind.direction);0.0;];
% flowLinearVelocityInB = Rbn(eta0).' * windVelocityInN;
% twa = arctan2(flowLinearVelocityInB(2), flowLinearVelocityInB(1));
twa = -pi + wind.direction;
vmg = abs(boat_speed_knots * cos(twa));
target_vmg = -0.00000272*rad2deg(abs(twa))^4+0.001025*rad2deg(abs(twa))^3-0.12778*rad2deg(abs(twa))^2+6.012*rad2deg(abs(twa))-74;
vmgE = vmg-target_vmg;

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
vmgMatrix = [vmgMatrix; t, vmg'];
vmgEMatrix = [vmgEMatrix; t, vmgE'];
fprintf('t=%.2f, wind=%.2f m/s, wd=%.2f rad\n', t, ws, wd);
end
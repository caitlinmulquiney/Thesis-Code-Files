clear all;
close all;
% clc;

%{b} is defined with origin at free surface, below mastfoot
global uMatrix vmgMatrix eMatrix vmgEMatrix;

% Preliminary computations to come close to equilibrium
load('K_scheduled.mat');
tspan = [0,150];
U0 = 18; % boat speed
beta0 = 1.3*pi/180; % boat drift angle
eta0 = [0;0;-1.3;2.6*pi/180;-0.5*pi/180;0*pi/180]; % boat attitude
nu0 = [Rbn(eta0).'* [U0*cos(beta0);U0*sin(beta0);0]; zeros(3,1)]; % boat velocity in {b}
windData = readmatrix('wind170528.txt');

foil0=loadFoilDescription;
Fo = [eta0; nu0];
options = odeset('RelTol',1e-2);
dt = 0.05;  % match your Python timestep exactly
%[t, F] = simulate_rk4(@(t,F) SystemLQRGain(t, F, K_grid, A_grid, B_grid, windSpeeds, windDirs, foil0, windData), tspan, Fo, dt);
%[t, F] = ode45(@SystemLQR, tspan, Fo, options);
% remove: global uMatrix
%sys = @(t,F) SystemLQRGain(t, F, K_grid, A_grid, B_grid, windSpeeds, windDirs, foil0, windData);
%sys_u = @(t,F) deal(SystemLQRGain(t, F, K_grid, A_grid, B_grid, windSpeeds, windDirs, foil0, windData));

%[t, F, uMatrix] = simulate_rk4(sys, tspan, Fo, dt);
off = 0;
[t, F] = ode15s(@(t,F) SystemLQRGain(t,F,K_grid,A_grid,B_grid,windSpeeds, windDirs, off, foil0, windData), tspan, Fo, options);
save('lqrGain_lift_variation.mat', 't', 'F', 'uMatrix', 'eMatrix')
%save('lqrGain_vmg_validation.mat', 'vmgMatrix', 'vmgEMatrix')
plotErrorSignal(t, F(:,1:6), F(:,7:12),uMatrix, vmgMatrix, vmgEMatrix)

clear all;
close all;
% clc;

%{b} is defined with origin at free surface, below mastfoot
global uMatrix;

% Preliminary computations to come close to equilibrium
load('K_scheduled.mat');
tspan = [0,600];
U0 = 16.18; % boat speed
beta0 = 1.3*pi/180; % boat drift angle
eta0 = [0;0;-1.3;2.6*pi/180;-0.5*pi/180;0*pi/180]; % boat attitude
nu0 = [Rbn(eta0).'* [U0*cos(beta0);U0*sin(beta0);0]; zeros(3,1)]; % boat velocity in {b}
windData = readmatrix('wind170528.txt');

foil0=loadFoilDescription;
Fo = [eta0; nu0];
options = odeset('RelTol',1e-2);
dt = 0.05;  % match your Python timestep exactly
[t, F] = simulate_rk4(@(t,F) SystemLQRGain(t, F, K_grid, A_grid, B_grid, windSpeeds, windDirs, foil0, windData), tspan, Fo, dt);
%[t, F] = ode45(@SystemLQR, tspan, Fo, options);
%[t, F] = ode15s(@(t,F) SystemLQRGain(t,F,K_grid,A_grid,B_grid,windSpeeds, windDirs, foil0, windData), tspan, Fo, options);
%save('lqrGain_model_variation.mat', 't', 'F', 'uMatrix')
plotErrorSignal(t, F(:,1:6), F(:,7:12),uMatrix)

function [t_out, F_out] = simulate_rk4(sys_func, tspan, F0, dt)
    t0 = tspan(1);
    tf = tspan(2);
    t  = t0:dt:tf;
    N  = length(t);
    
    F_out      = zeros(N, length(F0));
    F_out(1,:) = F0(:)';
    t_out      = t';

    for i = 1:N-1
        s  = F_out(i,:)';
        ti = t(i);
        
        k1 = sys_func(ti,        s);
        k2 = sys_func(ti + dt/2, s + 0.5*dt*k1);
        k3 = sys_func(ti + dt/2, s + 0.5*dt*k2);
        k4 = sys_func(ti + dt,   s + dt*k3);
        
        F_out(i+1,:) = s + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
    end
end
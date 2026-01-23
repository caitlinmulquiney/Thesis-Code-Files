clear all;
close all;
% clc;

%{b} is defined with origin at free surface, below mastfoot


% Preliminary computations to come close to equilibrium
tspan = [0, 10];
U0 = 21; % boat speed
beta0 = 1.2*pi/180; % boat drift angle
eta0 = [0;0;-1.6;5*pi/180;2*pi/180;0]; % boat attitude
nu0 = [Rbn(eta0).'* [U0*cos(beta0);U0*sin(beta0);0]; zeros(3,1)]; % boat velocity in {b}

Fo = [eta0; nu0];
options = odeset('RelTol',1e-12);
[t, F] = ode45(@System, tspan, Fo, options);
plotErrorSignal(t, F(:,1:6), F(:,7:12))
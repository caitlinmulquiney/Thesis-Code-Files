clear all;
close all;
% clc;

%{b} is defined with origin at free surface, below mastfoot
global uMatrix;

% Preliminary computations to come close to equilibrium
load('K.mat');
tspan = [0,150];
U0 = 16.18; % boat speed
beta0 = 1.3*pi/180; % boat drift angle
eta0 = [0;0;-1.3;2.6*pi/180;-0.5*pi/180;-0.8*pi/180]; % boat attitude
nu0 = [Rbn(eta0).'* [U0*cos(beta0);U0*sin(beta0);0]; zeros(3,1)]; % boat velocity in {b}

foil=loadFoilDescription;
Fo = [eta0; nu0];
options = odeset('RelTol',1e-2);
%[t, F] = ode45(@SystemLQR, tspan, Fo, options);
[t, F] = ode15s(@(t,F) SystemLQR(t,F,K,A,B,foil), tspan, Fo, options);
plotErrorSignal(t, F(:,1:6), F(:,7:12),uMatrix)
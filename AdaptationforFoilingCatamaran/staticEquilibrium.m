
clear all;
close all;
% clc;

%{b} is defined with origin at free surface, below mastfoot


% Preliminary computations to come close to equilibrium
U0 = 16.18; % boat speed
beta0 = 1.3*pi/180; % boat drift angle
%eta0 = [0;0;-1.3;2.6*pi/180;-0.5*pi/180;-0.8*pi/180]; % boat attitude
eta0 = [0;0;-1.3;2.6*pi/180;-0.5*pi/180;-0.7*pi/180]; % boat attitude paper numbers
nu0 = [Rbn(eta0).'* [U0*cos(beta0);U0*sin(beta0);0]; zeros(3,1)]; % boat velocity in {b}

wind.speedInN = 9.231; % wind speed in m/s
wind.direction = 30*pi/180; %(propagation direction, positive=wind from port side) 60deg= "-120" in classical terms
wave=[]; % no waves for now, but it will come

% Now, sum up foil loads, weight and aerodynamic load on superstucture.
foil=loadFoilDescription;
fprintf(1,'Initial values: U=%g beta=%g sinkage=%g heel=%g trim=%g\n',U0,beta0*180/pi,eta0(3),eta0(4)*180/pi,eta0(5)*180/pi);
fprintf(1,'---------------------------------------------------------------------------------------------------------------------------------- \n');
fprintf(1,'#  |       speed      alpha |       lift      drag  |         f1         f2         f3 |         m1         m2         m3 | comment \n');
fprintf(1,'---------------------------------------------------------------------------------------------------------------------------------- \n');
totalLoad = zeros(6,1);
for idxFoil=1:length(foil)
    totalLoad = totalLoad + foilLoad(eta0,nu0,foil{idxFoil},wind,wave,true);
end
totalLoad= totalLoad + weightLoad(eta0,true);
totalLoad = totalLoad + aerodynamicLoadSuperstructure(eta0,nu0,wind,true);

fprintf(1,'---------------------------------------------------------------------------------------------------------------------------------- \n');
fprintf(1,'T  |                                                | %+10.1f %+10.1f %+10.1f | %+10.1f %+10.1f %+10.1f \n',totalLoad.'/1000);

% 
% % Rough sketch showing the hull, the foils, the generated forces + their application point
% figure('Position',[680 1 1920 961]);
% drawBoat(eta0,nu0,foil,wind,wave)

%% Adjust the foils to come closer to equilibrium

options = optimset('MaxFunEvals',1e8,'TolFun',1e-10,'MaxIter',1e8);
[sol,val] = fminsearch(@(x)computeResidual(x,eta0,nu0,foil,wind),[0;0;0;0],options); %L foil, starboard rudder foil, port rudder foil, rudders

foil{2}.attitudeInB = foil{2}.attitudeInB + [0;sol(1);0];
foil{3}.attitudeInB = foil{3}.attitudeInB + [0;sol(1);0];
foil{4}.attitudeInB = foil{4}.attitudeInB + [0;sol(2);sol(4)]; 
foil{5}.attitudeInB = foil{5}.attitudeInB + [0;0;sol(4)]; 
foil{6}.attitudeInB = foil{6}.attitudeInB + [0;sol(3);sol(4)]; 
foil{7}.attitudeInB = foil{7}.attitudeInB + [0;0;sol(4)]; 


fprintf(1,'Delta_foilAngles = rakeL:%4.1f Starboard Tfoil:%4.1f Port Tfoil:%4.1f Rudder:%4.1f \n',sol.'*180/pi);
fprintf(1,'---------------------------------------------------------------------------------------------------------------------------------- \n');
fprintf(1,'#  |       speed      alpha |       lift      drag  |         f1         f2         f3 |         m1         m2         m3 | comment \n');
fprintf(1,'---------------------------------------------------------------------------------------------------------------------------------- \n');
totalLoad = zeros(6,1);
foilForce = zeros(6,7);
for idxFoil=1:length(foil)
    foilForce(:,idxFoil) = foilLoad(eta0,nu0,foil{idxFoil},wind,wave,true);
    totalLoad = totalLoad + foilForce(:,idxFoil);
end
weight = weightLoad(eta0,true);
totalLoad= totalLoad + weight;
aerodynamic = aerodynamicLoadSuperstructure(eta0,nu0,wind,true);
totalLoad = totalLoad + aerodynamic;
fprintf(1,'---------------------------------------------------------------------------------------------------------------------------------- \n');
fprintf(1,'T  |                                                | %+10.1f %+10.1f %+10.1f | %+10.1f %+10.1f %+10.1f \n',totalLoad.'/1000);

loads = [foilForce'; aerodynamic'; weight'];

foil0 = foil;

% %% Rough sketch showing the hull, the foils, the generated forces + their application point
% figure('Position',[680 1 1920 961]);
% drawBoat(eta0,nu0,foil0,wind,wave)
% 
% figure('Position',[680 1 1920 961]);
% t = tiledlayout(1,2);
% ax1 = nexttile;
% labels = {foil{1}.type, foil{2}.type, foil{3}.type, foil{4}.type, foil{5}.type, foil{6}.type, foil{7}.type, foil{8}.type, foil{9}.type, 'Aerodynamic load'};
% pie(ax1, loads(:,1));
% legend(labels);
% title('Positive X force');
% ax1 = nexttile;
% labels = {foil{3}.type, foil{4}.type, foil{5}.type, foil{6}.type, foil{7}.type, foil{8}.type, foil{9}.type, 'Aerodynamic load', 'Weight load'};
% pie(ax1, -1*loads(:,1));
% legend(labels);
% title('Negative X force');
% 
% figure('Position',[680 1 1920 961]);
% t = tiledlayout(1,2);
% ax1 = nexttile;
% labels = {foil{1}.type, foil{2}.type, foil{5}.type, foil{6}.type, 'Aerodynamic load', 'Weight load'};
% pie(ax1, loads(:,2));
% legend(labels);
% title('Positive Y force');
% ax1 = nexttile;
% labels = {foil{3}.type, foil{4}.type, foil{5}.type, foil{7}.type, foil{9}.type, 'Aerodynamic load'};
% pie(ax1, -1*loads(:,2));
% legend(labels);
% title('Negative Y force');
% 
% figure('Position',[680 1 1920 961]);
% t = tiledlayout(1,2);
% ax1 = nexttile;
% labels = {foil{9}.type, 'Aerodynamic load', 'Weight load'};
% pie(ax1, loads(:,3));
% legend(labels);
% title('Positive Z force');
% ax1 = nexttile;
% labels = {foil{1}.type, foil{2}.type, foil{3}.type, foil{4}.type, foil{5}.type, foil{6}.type, foil{7}.type, foil{8}.type, 'Aerodynamic load'};
% pie(ax1, -1*loads(:,3));
% legend(labels);
% title('Negative Z force');

%% Look at the sensitivity of forces/moment to foil angles, for the foils we might use as control inputs
% NB: valid for this eta, nu, foil angles.
J = configurationMatrix(foil0,eta0,nu0,wind,wave);

% figure('Position',[680 1 1920 961]);
% foilNames = {'T-foil floater'; 'floater rudder'; 'T-foil centerboard'; 'T-foil rud. centerhull';'centerhull rudder'};
% % Show jacobian of forces
% bar3(J(1:3,:)/1000/(180/pi))
% set(gca,'xtick',1:5,'xticklabel',foilNames)
% ylabel('Force component', 'FontSize', 15)
% zlabel('dF_i/du [kN/deg]', 'FontSize', 15)
% 
% figure('Position',[680 1 1920 961]);
% foilNames = {'T-foil floater'; 'floater rudder'; 'T-foil centerboard'; 'T-foil rud. centerhull';'centerhull rudder'};
% % Show jacobian of moments (usually one order of magnitude larger)
% bar3(J(4:6,:)/1000/(180/pi))
% set(gca,'xtick',1:5,'xticklabel',foilNames)
% ylabel('Moment component', 'FontSize', 15)
% zlabel('dM_i/du [kN/deg]', 'FontSize', 15);
% Foil names
foilNames = {'L foil', 'starboard T foil', 'starboard rudder', ...
             'port T foil', 'port rudder'};

% Scale factor
scalingFactor = 1000 / (180 / pi);

% Plotting Force Components (dF_i/du)
figure('Position', [680 1 1920 961]);
bar(J(1:3, :) * scalingFactor, 'grouped');  % Force components are in rows 1 to 3
set(gca, 'xtick', 1:5, 'xticklabel', {'F_z dF_i/du [kN/deg]', });  % Foil names on x-axis
xlabel('Force component', 'FontSize', 15);
title('Effect of Yawing foils', 'FontSize', 15);
legend(foilNames, 'Location', 'Best', 'FontSize', 12);

% Plotting Moment Components (dM_i/du)
figure('Position', [680 1 1920 961]);
bar(J(4:6, :) * scalingFactor, 'grouped');  % Moment components are in rows 4 to 6
set(gca, 'xtick', 1:5, 'xticklabel', {'M_x', 'M_y', 'M_z'});  % Foil names on x-axis
xlabel('Moment component', 'FontSize', 15);
ylabel('dM_i/du [kN/deg]', 'FontSize', 15);
title('Effect of Pitching Foils', 'FontSize', 15);
legend(foilNames, 'Location', 'Best', 'FontSize', 12);
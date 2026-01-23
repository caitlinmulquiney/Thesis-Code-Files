function [A, B, x_eq, u_eq] = linearizeSystem(eta0, nu0, wind, wave)
% Linearize trimaran dynamics around equilibrium point
% Outputs: A (12x12), B (12x5), x_eq (12x1), u_eq (5x1)

wind.speedInN = 9.231;
wind.direction = 30*pi/180;

foil = loadFoilDescription;
x_eq = [eta0; nu0];

% Perturbation sizes
delta_eta = [0.1; 0.1; 0.05; 0.01; 0.01; 0.01];
delta_nu = [0.1; 0.1; 0.1; 0.01; 0.01; 0.01];
delta_u = 0.5 * pi/180;

% Nominal forces
F_nom = systemDynamics(eta0, nu0, foil, wind, wave);
u_eq = extractControlAngles(foil);

% A matrix (df/dx)
A = zeros(12, 12);
for i = 1:6
    eta_pert = eta0;
    eta_pert(i) = eta_pert(i) + delta_eta(i);
    F_pert = systemDynamics(eta_pert, nu0, foil, wind, wave);
    A(:, i) = (F_pert - F_nom) / delta_eta(i);
end
for i = 1:6
    nu_pert = nu0;
    nu_pert(i) = nu_pert(i) + delta_nu(i);
    F_pert = systemDynamics(eta0, nu_pert, foil, wind, wave);
    A(:, 6+i) = (F_pert - F_nom) / delta_nu(i);
end

% B matrix (df/du)
B = zeros(12, 5);
control_defs = getControlDefinitions();
for i = 1:5
    foil_pert = foil;
    ctrl = control_defs{i};
    for j = 1:length(ctrl.foils)
        foil_idx = ctrl.foils(j);
        axis_idx = ctrl.axis(j);
        foil_pert{foil_idx}.attitudeInB(axis_idx) = ...
            foil{foil_idx}.attitudeInB(axis_idx) + delta_u;
    end
    F_pert = systemDynamics(eta0, nu0, foil_pert, wind, wave);
    B(:, i) = (F_pert - F_nom) / delta_u;
end

end

function F = systemDynamics(eta, nu, foil, wind, wave)
% Compute state derivative

totalLoad = zeros(6,1);
for idxFoil = 1:length(foil)
    totalLoad = totalLoad + foilLoad(eta, nu, foil{idxFoil}, wind, wave, false);
end
totalLoad = totalLoad + weightLoad(eta, false);
totalLoad = totalLoad + aerodynamicLoadSuperstructure(eta, nu, wind, false);

M = massDistribTrimaran(false);
C = coriolisCentripetal(M, nu);
nu_dot = M \ (totalLoad - C * nu);
eta_dot = Jbn(eta) * nu;

F = [eta_dot; nu_dot];
end

function u_eq = extractControlAngles(foil)
% Extract equilibrium control angles from foil structure
u_eq = [
    % foil{2}.attitudeInB(2);  % L-foil rake (stbd)
    % foil{4}.attitudeInB(2);  % T-foil rake (stbd)
    % foil{4}.attitudeInB(3);  % Rudder yaw (stbd)
    % foil{6}.attitudeInB(2);  % T-foil rake (port)

    foil{2}.attitudeInB(2);  % L-foil rake (stbd)
    foil{4}.attitudeInB(2);  % T-foil rake (stbd)
    foil{5}.attitudeInB(3);  % Rudder yaw (stbd)
    foil{6}.attitudeInB(2);  % T-foil rake (port)
    foil{7}.attitudeInB(3);  % T-foil rake (port)
];
end

function ctrl = getControlDefinitions()
% Define which foils move with each control input
ctrl = {
    % struct('foils', [2, 3], 'axis', [2, 2])  % L-foil rake (both parts)
    % struct('foils', [4], 'axis', [2])        % T-foil rake (stbd)
    % struct('foils', [4,5,6,7], 'axis', [3,3,3,3])        % Rudder yaw (stbd)
    % struct('foils', [6], 'axis', [2])        % T-foil rake (port)

    struct('foils', [2], 'axis', [2])  % L-foil rake (both parts)
    struct('foils', [4], 'axis', [2])        % T-foil rake (stbd)
    struct('foils', [5], 'axis', [3])        % Rudder yaw (stbd)
    struct('foils', [6], 'axis', [2])        % T-foil rake (port)
    struct('foils', [7], 'axis', [3])        % Rudder yaw (stbd)
};
end
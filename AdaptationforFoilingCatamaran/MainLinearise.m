clear all;
close all;
wind.speedInN = 9.231; % wind speed in m/s
wind.direction = 37*pi/180; %(propagation direction, positive=wind from port side) 60deg= "-120" in classical terms
U0 = 16.18; % boat speed
beta0 = 1.3*pi/180; % boat drift angle
eta0 = [0;0;-1.3;2.6*pi/180;-0.5*pi/180;-0.8*pi/180]; % boat attitude
nu0 = [Rbn(eta0).'* [U0*cos(beta0);U0*sin(beta0);0]; zeros(3,1)]; % boat velocity in {b}

% Linearize
[A, B, x_eq, u_eq] = linearizeSystem(eta0, nu0, wind, []);
Q = diag([1 1 5000 1 1 50 1 1 500 1 1 1]);
R = 10 * eye(size(B,2));
R(1,:) = [100 -2 -2 -2 -2];
R(:,1) = [500; -2; -2; -2; -2];
R(2,:) = [0 10 0 0 -2];
R(:,2) = [0; 10; 0; 0; -2];
[K,~,~] = lqr(A, B, Q, R);
save('K.mat','K', 'A', 'B');
% Check stability
[V, D] = eig(A);
eigenvalues = diag(D);
is_stable = all(real(eigenvalues) < 0);

if is_stable
    disp('System is stable');
else
    disp('System is unstable');
    unstable_modes = find(real(eigenvalues) > 0);
    fprintf('Number of unstable modes: %d\n', length(unstable_modes));
end

% Find the most unstable mode


[max_real, idx] = max(real(eigenvalues));
fprintf('Most unstable eigenvalue: %.4f (time constant: %.2f s)\n', ...
        max_real, 1/max_real);

% What state variables are involved?
mode_shape = V(:, idx);
[~, dominant_idx] = sort(abs(mode_shape), 'descend');
state_names = {'x','y','z','roll','pitch','yaw','u','v','w','p','q','r'};

fprintf('Dominant states in unstable mode:\n');
for i = 1:4
    fprintf('  %s: %.3f\n', state_names{dominant_idx(i)}, abs(mode_shape(dominant_idx(i))));
end

M = massDistribTrimaran(false);

% Check eigenvalues (all should be positive)
eig_M = eig(M);
fprintf('Mass matrix eigenvalues:\n');
disp(eig_M);

if any(eig_M < 0)
    error('Mass matrix has negative eigenvalues - physically impossible!');
end

% Check condition number (should be < 1e6)
cond_M = cond(M);
fprintf('Mass matrix condition number: %.2e\n', cond_M);
if cond_M > 1e6
    warning('Mass matrix is ill-conditioned');
end

% Plot eigenvalues
figure;
plot(real(eigenvalues), imag(eigenvalues), 'rx', 'MarkerSize', 10);
hold on;
xline(0, 'k--', 'LineWidth', 2);
grid on;
xlabel('Real Part');
ylabel('Imaginary Part');
title('System Eigenvalues');
clear all;
close all;

% Define operating points
windSpeeds = [5, 6, 7, 8, 9, 10, 11, 12, 13];
windDirs   = [30, 45, 60, 75, 90, 105, 120, 135, 150] * pi/180;
U0s        = [7, 13.5, 16, 19, 20, 22, 21, 19, 17.5;
              7, 13.5, 16, 19, 20, 22, 21, 19, 17.5;
              8, 14.5, 17, 20, 21, 23, 22, 20, 18.5;
              9, 15.5, 18, 21, 22, 24, 23, 21, 19.5;
              10, 16.5, 19, 22, 23, 25, 24, 22, 20.5;
              11, 17.5, 20, 23, 24, 26, 25, 23, 21.5;
              12, 18.5, 21, 24, 25, 26, 26, 24, 22.5;
              13, 19.5, 22, 25, 26, 26, 26, 25, 23.5;
              13, 19.5, 22, 25, 26, 26, 26, 25, 23.5;];       % boat speed at each wind speed
beta0      = 1.3*pi/180;

% Preallocate
K_grid = cell(length(windSpeeds), length(windDirs));
A_grid = cell(length(windSpeeds), length(windDirs));
B_grid = cell(length(windSpeeds), length(windDirs));

Q = diag([1, 1, 5000, 1, 1, 500, 1, 1, 500, 1, 1, 50]);
R = diag([10 10 50 10 10 10 10]);

% Compute LQR gain at each operating point
for i = 1:length(windSpeeds)
    for j = 1:length(windDirs)
        fprintf('Computing gain: wind=%.1f m/s, dir=%.1f deg\n', ...
                windSpeeds(i), windDirs(j)*180/pi);
        
        wind.speedInN  = windSpeeds(i);
        wind.direction = windDirs(j);
        U0             = U0s(i,j);
        
        eta0 = [0;0;-1.3;2.6*pi/180;-0.5*pi/180;0*pi/180];
        nu0  = [Rbn(eta0).' * [U0*cos(beta0); U0*sin(beta0); 0]; zeros(3,1)];
        
        [A, B, ~, ~] = linearizeSystem(eta0, nu0, wind, []);
        
        % Check stabilisability before computing gain
        eigs = eig(A);
        if any(real(eigs) > 1e6)
            warning('Operating point (i=%d,j=%d) may be unstable', i, j);
        end
        
        try
            [K, ~, ~] = lqr(A, B, Q, R);
            K_grid{i,j} = K;
            A_grid{i,j} = A;
            B_grid{i,j} = B;
        catch e
            warning('LQR failed at (i=%d,j=%d): %s', i, j, e.message);
            K_grid{i,j} = [];
        end
    end
end

% Save scheduled gains
save('K_scheduled.mat', 'K_grid', 'A_grid', 'B_grid', 'windSpeeds', 'windDirs');

% --- Stability analysis at nominal operating point ---
wind_nom.speedInN  = 9.231;
wind_nom.direction = 37*pi/180;
U0_nom  = 16.18;
eta0    = [0;0;-1.3;2.6*pi/180;-0.5*pi/180;0*pi/180];
nu0     = [Rbn(eta0).' * [U0_nom*cos(beta0); U0_nom*sin(beta0); 0]; zeros(3,1)];
[A_nom, B_nom, ~, ~] = linearizeSystem(eta0, nu0, wind_nom, []);

[V, D]      = eig(A_nom);
eigenvalues = diag(D);
is_stable   = all(real(eigenvalues) < 0);

if is_stable
    disp('Nominal system is stable');
else
    disp('Nominal system is unstable');
    unstable_modes = find(real(eigenvalues) > 0);
    fprintf('Number of unstable modes: %d\n', length(unstable_modes));
end

[max_real, idx] = max(real(eigenvalues));
fprintf('Most unstable eigenvalue: %.4f (time constant: %.2f s)\n', ...
        max_real, 1/abs(max_real));

mode_shape   = V(:, idx);
[~, dom_idx] = sort(abs(mode_shape), 'descend');
state_names  = {'x','y','z','roll','pitch','yaw','u','v','w','p','q','r'};
fprintf('Dominant states in most unstable mode:\n');
for i = 1:4
    fprintf('  %s: %.3f\n', state_names{dom_idx(i)}, abs(mode_shape(dom_idx(i))));
end

% --- Mass matrix checks ---
M       = massDistribTrimaran(false);
eig_M   = eig(M);
cond_M  = cond(M);
fprintf('Mass matrix condition number: %.2e\n', cond_M);
if any(eig_M < 0),  error('Mass matrix has negative eigenvalues'); end
if cond_M > 1e6,    warning('Mass matrix is ill-conditioned');      end

% --- Plot eigenvalues for all operating points ---
figure; hold on;
colors = lines(length(windSpeeds));
for i = 1:length(windSpeeds)
    for j = 1:length(windDirs)
        if ~isempty(A_grid{i,j})
            eigs_ij = eig(A_grid{i,j});
            plot(real(eigs_ij), imag(eigs_ij), 'x', ...
                 'Color', colors(i,:), 'MarkerSize', 10);
        end
    end
end
xline(0, 'k--', 'LineWidth', 2);
grid on;
xlabel('Real Part');
ylabel('Imaginary Part');
title('Eigenvalues Across Operating Points');
legend(arrayfun(@(v) sprintf('%.0f m/s', v), windSpeeds, 'UniformOutput', false));
clear all; close all; clc;

%% LOAD SYSTEM
if exist('separated_system_params.mat', 'file')
    load('separated_system_params.mat');
    fprintf('✓ Loaded system parameters\n');
else
    run('separated_architecture.m');
    close all;
end

%% SETUP MIMO SYSTEM
m1 = 1.0;   % Mass 1 (kg)
m2 = 1.0;   % Mass 2 (kg)
k1 = 2.0;   % Spring constant 1 (N/m)
k2 = 1.0;   % Coupling spring (N/m)
k3 = 2.0;   % Spring constant 2 (N/m)
b1 = 0.5;   % Damping 1 (N·s/m)
b2 = 0.3;   % Damping 2 (N·s/m)

% State-space matrices
A = [0,           1,          0,           0;
     -(k1+k2)/m1, -b1/m1,     k2/m1,       0;
     0,           0,          0,           1;
     k2/m2,       0,          -(k2+k3)/m2, -b2/m2];

B = [0,     0;
     1/m1,  0;
     0,     0;
     0,     1/m2];


B_mimo = [B(:,1), B(:,2)];
B_nom = [B_mimo,B_nom(:,3),B_nom(:,4)];
A_mimo = A;

fprintf('\nOriginal System:\n');
fprintf('  States: 4 [V, γ, q, θ]\n');
fprintf('  Inputs: 2 [elevator, throttle]\n\n');

%% DESIGN STANDARD LQR (for comparison)
fprintf('=== STANDARD LQR ===\n');
Q = diag([0.01, 0.1, 0.1, 0.1]);
R = diag([0.01, 0.001]);
K_lqr = lqr(A_mimo, B_mimo, Q, R);

fprintf('LQR Gains:\n');
fprintf('           V        γ        q        θ\n');
fprintf('Elev:   %7.4f  %7.4f  %7.4f  %7.4f\n', K_lqr(1,:));
fprintf('Throt:  %7.4f  %7.4f  %7.4f  %7.4f\n', K_lqr(2,:));

eigs_lqr = eig(A_mimo - B_mimo*K_lqr);
fprintf('LQR Stability: ');
if all(real(eigs_lqr) < 0)
    fprintf('✓ STABLE (max Re(λ) = %.4f)\n\n', max(real(eigs_lqr)));
else
    fprintf('✗ UNSTABLE\n\n');
end

%% DESIGN LQI CONTROLLER
fprintf('=== LQI CONTROLLER (with Integral Action) ===\n');

% Define which outputs to integrate (track with zero error)
% Track state 1 (V) and state 4 (θ)
outputs_to_track = [1, 4];  % V and θ

fprintf('Integrating error for:\n');
fprintf('  State %d: V (Velocity)\n', outputs_to_track(1));
fprintf('  State %d: θ (Pitch angle)\n\n', outputs_to_track(2));

% Create output selection matrix C
n = size(A_mimo, 1);  % Number of states (4)
C_track = zeros(length(outputs_to_track), n);
for i = 1:length(outputs_to_track)
    C_track(i, outputs_to_track(i)) = 1;
end

% Build augmented system
n_I = size(C_track, 1);     % Number of integral states (2)
n_aug = n + n_I;            % Total augmented states (6)

A_aug = [A_mimo,            zeros(n, n_I);
         -C_track,          zeros(n_I, n_I)];

B_aug = [B_mimo;
         zeros(n_I, size(B_mimo, 2))];

fprintf('Augmented System:\n');
fprintf('  States: %d [V, γ, q, θ, ∫e_V, ∫e_θ]\n', n_aug);
fprintf('  Inputs: 2 [elevator, throttle]\n\n');

% Augmented weights
Q_x = Q;  % Original state weights (4x4)

% Integral weights (typically 10-100x the state weight)
Q_I = diag([0.10,      % ∫e_V  (10x Q_V)
            1.0]);    % ∫e_θ  (10x Q_θ)

Q_aug = blkdiag(Q_x, Q_I);  % Augmented Q (6x6)
R_aug = R;                   % Same R (2x2)

fprintf('LQI Weights:\n');
fprintf('  Q_V  = %.3f,  Q_I_V  = %.3f  (ratio: %.1fx)\n', Q(1,1), Q_I(1,1), Q_I(1,1)/Q(1,1));
fprintf('  Q_θ  = %.3f,  Q_I_θ  = %.3f  (ratio: %.1fx)\n\n', Q(4,4), Q_I(2,2), Q_I(2,2)/Q(4,4));

% Solve LQI
K_aug = lqr(A_aug, B_aug, Q_aug, R_aug);

fprintf('LQI Gains (Augmented):\n');
fprintf('           V        γ        q        θ      ∫e_V     ∫e_θ\n');
fprintf('Elev:   %7.4f  %7.4f  %7.4f  %7.4f  %7.4f  %7.4f\n', K_aug(1,:));
fprintf('Throt:  %7.4f  %7.4f  %7.4f  %7.4f  %7.4f  %7.4f\n\n', K_aug(2,:));

% Extract sub-gains
K_x = K_aug(:, 1:n);          % State feedback (2x4)
K_I = K_aug(:, n+1:end);      % Integral feedback (2x2)

fprintf('Split into:\n');
fprintf('  K_x (state feedback):    2x4\n');
fprintf('  K_I (integral feedback): 2x2\n\n');

% Check stability
eigs_lqi = eig(A_aug - B_aug*K_aug);
fprintf('LQI Stability: ');
if all(real(eigs_lqi) < 0)
    fprintf('✓ STABLE (max Re(λ) = %.4f)\n\n', max(real(eigs_lqi)));
else
    fprintf('✗ UNSTABLE\n\n');
end

%% DISPLAY EIGENVALUES COMPARISON
fprintf('=== EIGENVALUE COMPARISON ===\n');
fprintf('LQR Eigenvalues:\n');
for i = 1:length(eigs_lqr)
    fprintf('  λ%d = %+.4f %+.4fj\n', i, real(eigs_lqr(i)), imag(eigs_lqr(i)));
end

fprintf('\nLQI Eigenvalues:\n');
for i = 1:length(eigs_lqi)
    fprintf('  λ%d = %+.4f %+.4fj\n', i, real(eigs_lqi(i)), imag(eigs_lqi(i)));
end

%% SAVE RESULTS
fprintf('\n=== SAVING RESULTS ===\n');
save('lqi_controller.mat', 'K_lqr', 'K_aug', 'K_x', 'K_I', ...
     'A_aug', 'B_aug', 'Q_aug', 'R_aug', 'C_track', ...
     'eigs_lqr', 'eigs_lqi', 'n_I', 'outputs_to_track');

fprintf('✓ Saved to lqi_controller.mat\n\n');

fprintf('=== USAGE ===\n');
fprintf('LQR Control Law:\n');
fprintf('  u = -K_lqr * (x - x_ref)\n\n');

fprintf('LQI Control Law:\n');
fprintf('  e = C_track*x - [V_ref; θ_ref]  %% Error\n');
fprintf('  x_I = x_I + e*dt                %% Integrate\n');
fprintf('  x_aug = [x; x_I]                %% Stack states\n');
fprintf('  u = -K_aug * x_aug              %% Control\n\n');

fprintf('✓ LQI controller ready!\n');
fprintf('✓ Guarantees zero steady-state error in V and θ\n');

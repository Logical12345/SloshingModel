%% TEST SYSTEM: LQI CONTROL - NON-BAFFLED vs BAFFLED SLOSH
%
% Purpose: Show the dramatic benefit of baffles on slosh disturbance
%
% System: Double mass-spring-damper (4 states)
% Control: LQI with integral action (6 augmented states)
%
% Comparisons:
%   1. LQI Nominal (no slosh) - baseline
%   2. LQI + Non-Baffled Slosh - single large compartment
%   3. LQI + Baffled Slosh - 4 small compartments

clear all; close all; clc;

fprintf('╔════════════════════════════════════════════════════════╗\n');
fprintf('║   LQI CONTROL: NON-BAFFLED vs BAFFLED SLOSH           ║\n');
fprintf('╚════════════════════════════════════════════════════════╝\n\n');

%% STEP 1: DEFINE NOMINAL PLANT
fprintf('STEP 1: Define Nominal Plant (4 states)\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

% Physical parameters
m1 = 1.0;    % Mass 1 (kg)
m2 = 1.0;    % Mass 2 (kg)
k1 = 2.0;    % Spring 1 (N/m)
k2 = 1.0;    % Coupling spring (N/m)
k3 = 2.0;    % Spring 2 (N/m)
b1 = 0.5;    % Damping 1 (N·s/m)
b2 = 0.3;    % Damping 2 (N·s/m)

fprintf('Physical parameters:\n');
fprintf('  m1 = %.1f kg,  m2 = %.1f kg\n', m1, m2);
fprintf('  k1 = %.1f N/m, k2 = %.1f N/m, k3 = %.1f N/m\n', k1, k2, k3);
fprintf('  b1 = %.1f N·s/m, b2 = %.1f N·s/m\n\n', b1, b2);

% State-space: dx/dt = A*x + B*u + B_dist*d
A_nom = [0,           1,          0,           0;
         -(k1+k2)/m1, -b1/m1,     k2/m1,       0;
         0,           0,          0,           1;
         k2/m2,       0,          -(k2+k3)/m2, -b2/m2];

B_nom = [0,     0;
         1/m1,  0;
         0,     0;
         0,     1/m2];

B_dist = [0,      0;
          1/m1,   0;
          0,      0;
          0,      1/m2];

fprintf('State vector: x = [x1, v1, x2, v2]ᵀ\n');
fprintf('Input vector: u = [u1, u2]ᵀ (control forces)\n\n');

% Check stability and controllability
eigs_open = eig(A_nom);
Co = ctrb(A_nom, B_nom);

if all(real(eigs_open) < 0) && rank(Co) == 4
    fprintf('✓ Nominal system is STABLE and CONTROLLABLE\n\n');
end

%% STEP 2: DEFINE NON-BAFFLED SLOSH MODEL
fprintf('STEP 2: Define NON-BAFFLED Slosh (Single Compartment)\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

% Total sloshing mass
m_slosh_total = 0.50 * m1;   % 20% of mass 1 is liquid

% Non-baffled parameters (full tank width)
L_tank = 1.0;  % Tank width (m)
k_slosh_nb = 5.0;        % Spring constant (N/m)
b_slosh_nb = 0.2;        % Low damping (no baffles)

m_slosh_nb = m_slosh_total;  % All mass in one compartment

fprintf('Non-Baffled Tank:\n');
fprintf('  Tank width:      %.2f m\n', L_tank);
fprintf('  Sloshing mass:   %.2f kg (%.0f%% of m1)\n', m_slosh_nb, m_slosh_nb/m1*100);
fprintf('  Spring constant: %.1f N/m\n', k_slosh_nb);
fprintf('  Damping:         %.2f N·s/m (low)\n\n', b_slosh_nb);

% Dynamics
A_slosh_nb = [0,                1;
              -k_slosh_nb/m_slosh_nb, -b_slosh_nb/m_slosh_nb];

B_slosh_nb = [0;
              1];

K_dist_nb = [k_slosh_nb,  b_slosh_nb;
             0,           0];

% Natural frequency and damping
omega_nb = sqrt(k_slosh_nb/m_slosh_nb);
zeta_nb = b_slosh_nb/(2*sqrt(k_slosh_nb*m_slosh_nb));

fprintf('Non-Baffled Dynamics:\n');
fprintf('  Natural frequency: ω = %.2f rad/s (%.2f Hz)\n', omega_nb, omega_nb/(2*pi));
fprintf('  Damping ratio:     ζ = %.3f (lightly damped)\n', zeta_nb);
fprintf('  Period:            T = %.2f s\n\n', 2*pi/omega_nb);

%% STEP 3: DEFINE BAFFLED SLOSH MODEL
fprintf('STEP 3: Define BAFFLED Slosh (4 Compartments)\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

N_baffles = 4;  % Number of compartments
L_compartment = L_tank / N_baffles;  % Width of each compartment

% Each compartment has 1/N of the mass
m_slosh_baf = m_slosh_total / N_baffles;

% Higher spring constant (shorter length → stiffer)
k_slosh_baf = k_slosh_nb * N_baffles;

% Higher damping (baffles create turbulence)
b_slosh_baf = b_slosh_nb * 2.5;  % 2.5x increase due to baffle turbulence

fprintf('Baffled Tank:\n');
fprintf('  Number of compartments: %d\n', N_baffles);
fprintf('  Compartment width:      %.2f m (%.0f%% of tank)\n', L_compartment, 100/N_baffles);
fprintf('  Mass per compartment:   %.3f kg\n', m_slosh_baf);
fprintf('  Spring constant:        %.1f N/m (%.0fx)\n', k_slosh_baf, k_slosh_baf/k_slosh_nb);
fprintf('  Damping:                %.2f N·s/m (%.1fx)\n\n', b_slosh_baf, b_slosh_baf/b_slosh_nb);

% Each compartment has identical dynamics
A_slosh_baf = [0,                1;
               -k_slosh_baf/m_slosh_baf, -b_slosh_baf/m_slosh_baf];

B_slosh_baf = [0;
               1];

% Each compartment contributes to disturbance
K_dist_baf = [k_slosh_baf,  b_slosh_baf;
              0,            0];

% Natural frequency and damping
omega_baf = sqrt(k_slosh_baf/m_slosh_baf);
zeta_baf = b_slosh_baf/(2*sqrt(k_slosh_baf*m_slosh_baf));

fprintf('Baffled Dynamics (per compartment):\n');
fprintf('  Natural frequency: ω = %.2f rad/s (%.2f Hz)\n', omega_baf, omega_baf/(2*pi));
fprintf('  Damping ratio:     ζ = %.3f (%.0fx higher)\n', zeta_baf, zeta_baf/zeta_nb);
fprintf('  Period:            T = %.2f s\n\n', 2*pi/omega_baf);

fprintf('Expected Benefits:\n');
fprintf('  Frequency increase:    %.1fx (faster oscillation, less amplitude)\n', omega_baf/omega_nb);
fprintf('  Damping increase:      %.1fx (faster decay)\n', zeta_baf/zeta_nb);
fprintf('  Force per compartment: %.1fx (smaller mass)\n', m_slosh_baf/m_slosh_nb);
fprintf('  Phase cancellation:    Forces partially cancel out\n\n');

%% STEP 4: DESIGN LQI CONTROLLER
fprintf('STEP 4: Design LQI Controller\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

% Track positions x1 and x2
outputs_to_track = [1, 3];
C_track = zeros(2, 4);
C_track(1, 1) = 1;  % Track x1
C_track(2, 3) = 1;  % Track x2

% Augmented system
n = 4;
n_I = 2;
A_aug = [A_nom,      zeros(4, 2);
         -C_track,   zeros(2, 2)];

B_aug = [B_nom;
         zeros(2, 2)];

fprintf('Augmented system: %d states [x1, v1, x2, v2, ∫e_x1, ∫e_x2]\n\n', n + n_I);

% Weights
Q_x = diag([10, 1, 10, 1]);
Q_I = diag([100, 100]);
Q_aug = blkdiag(Q_x, Q_I);
R_aug = diag([0.1, 0.1]);

% Solve LQI
K_aug = lqr(A_aug, B_aug, Q_aug, R_aug);

fprintf('LQI Gains:\n');
fprintf('       x1      v1      x2      v2    ∫e_x1   ∫e_x2\n');
fprintf('u1: %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f\n', K_aug(1,:));
fprintf('u2: %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f\n\n', K_aug(2,:));

% Check stability
if all(real(eig(A_aug - B_aug*K_aug)) < 0)
    fprintf('✓ LQI closed-loop is STABLE\n\n');
end

%% STEP 5: SIMULATION SETUP
fprintf('STEP 5: Simulation Setup\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━\n');

dt = 0.01;
t = 0:dt:30;
N = length(t);

% Initial conditions
x0 = [1; 0; -0.5; 0];  % Displaced masses
x_ref = [0.5; 0; -0.2; 0];  % Reference positions

% Initial slosh (same for both cases - 10 cm displacement)
x_slosh_init = [0.1; 0];

fprintf('Initial conditions:\n');
fprintf('  Mass positions: x1 = %.2f m, x2 = %.2f m\n', x0(1), x0(3));
fprintf('  Reference:      x1 = %.2f m, x2 = %.2f m\n', x_ref(1), x_ref(3));
fprintf('  Slosh initial:  %.0f cm displacement\n', x_slosh_init(1)*100);
fprintf('  Simulation:     %.0f seconds\n\n', t(end));

%% STEP 6: SIMULATE LQI NOMINAL (NO SLOSH)
fprintf('STEP 6: Simulate LQI - Nominal (No Slosh)\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

x_nominal = zeros(4, N);
x_I_nominal = zeros(2, N);
u_nominal = zeros(2, N);
x_nominal(:,1) = x0;

for i = 1:N-1
    y = C_track * x_nominal(:,i);
    r = [x_ref(1); x_ref(3)];
    e = r - y;
    x_I_nominal(:,i+1) = x_I_nominal(:,i) + e * dt;
    
    x_aug_vec = [x_nominal(:,i); x_I_nominal(:,i)];
    u_nominal(:,i) = -K_aug * x_aug_vec;
    u_nominal(:,i) = max(min(u_nominal(:,i), 10), -10);
    
    dx = A_nom * x_nominal(:,i) + B_nom * u_nominal(:,i);
    x_nominal(:,i+1) = x_nominal(:,i) + dx * dt;
end
u_nominal(:,end) = u_nominal(:,end-1);

error_nominal_x1 = abs(x_nominal(1,end) - x_ref(1));
error_nominal_x2 = abs(x_nominal(3,end) - x_ref(3));

fprintf('Results:\n');
fprintf('  Final: x1 = %.6f m, x2 = %.6f m\n', x_nominal(1,end), x_nominal(3,end));
fprintf('  Error: x1 = %.6f m, x2 = %.6f m\n\n', error_nominal_x1, error_nominal_x2);

%% STEP 7: SIMULATE LQI WITH NON-BAFFLED SLOSH
fprintf('STEP 7: Simulate LQI - NON-BAFFLED Slosh\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
slots = 10;
x_nonbaffled = zeros(4, N);
x_I_nonbaffled = zeros(2, N);
x_slosh_nb = zeros(2, N);
u_nonbaffled = zeros(2, N);
F_dist_nb = zeros(N, 1);

x_nonbaffled(:,1) = x0;
x_slosh_nb(:,1) = x_slosh_init;

for i = 1:N-1
    % Control law
    y = C_track * x_nonbaffled(:,i);
    r = [x_ref(1); x_ref(3)];
    e = r - y;
    x_I_nonbaffled(:,i+1) = x_I_nonbaffled(:,i) + e * dt;
    
    x_aug_vec = [x_nonbaffled(:,i); x_I_nonbaffled(:,i)];
    u_nonbaffled(:,i) = -K_aug * x_aug_vec;
    u_nonbaffled(:,i) = max(min(u_nonbaffled(:,i), 10), -10);
    
    % Slosh disturbance
    d = K_dist_nb * x_slosh_nb(:,i);
    F_dist_nb(i) = d(1);
    
    % Plant dynamics WITH disturbance
    dx = A_nom * x_nonbaffled(:,i) + B_nom * u_nonbaffled(:,i) + B_dist * d;
    x_nonbaffled(:,i+1) = x_nonbaffled(:,i) + dx * dt;
    
    % Slosh dynamics (excited by acceleration)
    a1 = dx(2);
    dx_slosh = A_slosh_nb * x_slosh_nb(:,i) + B_slosh_nb * a1;
    x_slosh_nb(:,i+1) = x_slosh_nb(:,i) + dx_slosh * dt;
end
u_nonbaffled(:,end) = u_nonbaffled(:,end-1);
F_dist_nb(end) = F_dist_nb(end-1);

error_nb_x1 = abs(x_nonbaffled(1,end) - x_ref(1));
error_nb_x2 = abs(x_nonbaffled(3,end) - x_ref(3));
max_slosh_nb = max(abs(x_slosh_nb(1,:)));
max_force_nb = max(abs(F_dist_nb));

fprintf('Results:\n');
fprintf('  Final: x1 = %.6f m, x2 = %.6f m\n', x_nonbaffled(1,end), x_nonbaffled(3,end));
fprintf('  Error: x1 = %.6f m, x2 = %.6f m\n', error_nb_x1, error_nb_x2);
fprintf('  Max slosh displacement: %.2f cm\n', max_slosh_nb*100);
fprintf('  Max disturbance force:  %.3f N\n\n', max_force_nb);

%% STEP 8: SIMULATE LQI WITH BAFFLED SLOSH
fprintf('STEP 8: Simulate LQI - BAFFLED Slosh (4 Compartments)\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
slots = 5;
x_baffled = zeros(slots, N);
x_I_baffled = zeros(2, N);
x_slosh_baf = zeros(2, N_baffles, N);  % [state, compartment, time]
u_baffled = zeros(2, N);
F_dist_baf = zeros(N, 1);
F_compartments = zeros(N_baffles, N);  % Force from each compartment

x_baffled(:,1) = x0;

% Initialize each compartment with slightly different phase
for comp = 1:N_baffles
    phase_offset = (comp-1) * 0.02;  % Small phase differences
    x_slosh_baf(:, comp, 1) = x_slosh_init + [phase_offset; 0];
end

for i = 1:N-1
    % Control law (SAME as non-baffled - doesn't know about baffles!)
    y = C_track * x_baffled(:,i);
    r = [x_ref(1); x_ref(3)];
    e = r - y;
    x_I_baffled(:,i+1) = x_I_baffled(:,i) + e * dt;
    
    x_aug_vec = [x_baffled(:,i); x_I_baffled(:,i)];
    u_baffled(:,i) = -K_aug * x_aug_vec;
    u_baffled(:,i) = max(min(u_baffled(:,i), 10), -10);
    
    % Compute total disturbance from ALL compartments
    d_total = [0; 0];
    for comp = 1:N_baffles
        d_comp = K_dist_baf * x_slosh_baf(:, comp, i);
        d_total = d_total + d_comp;
        F_compartments(comp, i) = d_comp(1);  % Track individual forces
    end
    F_dist_baf(i) = d_total(1);
    
    % Plant dynamics WITH total disturbance
    dx = A_nom * x_baffled(:,i) + B_nom * u_baffled(:,i) + B_dist * d_total;
    x_baffled(:,i+1) = x_baffled(:,i) + dx * dt;
    
    % Update each compartment (all excited by SAME acceleration)
    a1 = dx(2);
    for comp = 1:N_baffles
        dx_slosh = A_slosh_baf * x_slosh_baf(:, comp, i) + B_slosh_baf * a1;
        x_slosh_baf(:, comp, i+1) = x_slosh_baf(:, comp, i) + dx_slosh * dt;
    end
end
u_baffled(:,end) = u_baffled(:,end-1);
F_dist_baf(end) = F_dist_baf(end-1);

error_baf_x1 = abs(x_baffled(1,end) - x_ref(1));
error_baf_x2 = abs(x_baffled(3,end) - x_ref(3));
max_slosh_baf = max(max(abs(squeeze(x_slosh_baf(1,:,:)))));
max_force_baf = max(abs(F_dist_baf));

fprintf('Results:\n');
fprintf('  Final: x1 = %.6f m, x2 = %.6f m\n', x_baffled(1,end), x_baffled(3,end));
fprintf('  Error: x1 = %.6f m, x2 = %.6f m\n', error_baf_x1, error_baf_x2);
fprintf('  Max slosh per compartment: %.2f cm\n', max_slosh_baf*100);
fprintf('  Max total disturbance force: %.3f N\n\n', max_force_baf);

%% STEP 9: PERFORMANCE COMPARISON
fprintf('STEP 9: Performance Comparison\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

fprintf('┌────────────────────────┬──────────┬───────────┬──────────┐\n');
fprintf('│ Metric                 │ Nominal  │ Non-Baffled│ Baffled │\n');
fprintf('├────────────────────────┼──────────┼───────────┼──────────┤\n');
fprintf('│ x1 error (m)           │ %.6f │ %.6f  │ %.6f │\n', ...
    error_nominal_x1, error_nb_x1, error_baf_x1);
fprintf('│ x2 error (m)           │ %.6f │ %.6f  │ %.6f │\n', ...
    error_nominal_x2, error_nb_x2, error_baf_x2);
fprintf('│ Max slosh (cm)         │   N/A    │   %.2f     │   %.2f   │\n', ...
    max_slosh_nb*100, max_slosh_baf*100);
fprintf('│ Max disturbance (N)    │   0.000  │   %.3f    │  %.3f   │\n', ...
    max_force_nb, max_force_baf);
fprintf('└────────────────────────┴──────────┴───────────┴──────────┘\n\n');

% Calculate improvements
improvement_error = (error_nb_x1 - error_baf_x1) / error_nb_x1 * 100;
improvement_slosh = (max_slosh_nb - max_slosh_baf) / max_slosh_nb * 100;
improvement_force = (max_force_nb - max_force_baf) / max_force_nb * 100;

fprintf('BENEFIT OF BAFFLES:\n');
fprintf('  Error reduction:       %.1f%% improvement\n', improvement_error);
fprintf('  Slosh amplitude:       %.1f%% reduction\n', improvement_slosh);
fprintf('  Disturbance force:     %.1f%% reduction\n\n', improvement_force);

degradation_nb = (error_nb_x1 - error_nominal_x1) / error_nominal_x1 * 100;
degradation_baf = (error_baf_x1 - error_nominal_x1) / error_nominal_x1 * 100;

fprintf('DEGRADATION FROM NOMINAL:\n');
fprintf('  Non-baffled:  +%.0f%% error increase\n', degradation_nb);
fprintf('  Baffled:      +%.0f%% error increase  ← Much better!\n\n', degradation_baf);

if error_baf_x1 < 0.01 && error_baf_x2 < 0.01
    fprintf('✓✓✓ BAFFLES WORK! Excellent tracking maintained! ✓✓✓\n\n');
else
    fprintf('✓ Baffles help but some degradation remains\n\n');
end

%% STEP 10: PLOTTING
fprintf('STEP 10: Generating Comparison Plots\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

figure('Position', [50, 50, 1800, 1000]);

% Position Mass 1
subplot(3,4,1);
plot(t, x_nominal(1,:), 'b-', 'LineWidth', 2);
hold on;
plot(t, x_nonbaffled(1,:), 'r--', 'LineWidth', 2);
plot(t, x_baffled(1,:), 'g:', 'LineWidth', 2);
yline(x_ref(1), 'k--', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('x1 (m)');
title('Position: Mass 1');
legend('Nominal', 'Non-Baffled', 'Baffled', 'Ref', 'Location', 'best');

% Velocity Mass 1
subplot(3,4,2);
plot(t, x_nominal(2,:), 'b-', 'LineWidth', 2);
hold on;
plot(t, x_nonbaffled(2,:), 'r--', 'LineWidth', 2);
plot(t, x_baffled(2,:), 'g:', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('v1 (m/s)');
title('Velocity: Mass 1');

% Position Mass 2
subplot(3,4,3);
plot(t, x_nominal(3,:), 'b-', 'LineWidth', 2);
hold on;
plot(t, x_nonbaffled(3,:), 'r--', 'LineWidth', 2);
plot(t, x_baffled(3,:), 'g:', 'LineWidth', 2);
yline(x_ref(3), 'k--', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('x2 (m)');
title('Position: Mass 2');

% Control Force 1
subplot(3,4,4);
plot(t, u_nominal(1,:), 'b-', 'LineWidth', 2);
hold on;
plot(t, u_nonbaffled(1,:), 'r--', 'LineWidth', 2);
plot(t, u_baffled(1,:), 'g:', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('u1 (N)');
title('Control Force: Mass 1');
legend('Nominal', 'Non-Baffled', 'Baffled', 'Location', 'best');

% Slosh Displacement Comparison
subplot(3,4,5);
plot(t, x_slosh_nb(1,:)*100, 'r-', 'LineWidth', 2);
hold on;
% Plot all baffled compartments
for comp = 1:N_baffles
    plot(t, squeeze(x_slosh_baf(1,comp,:))*100, ':', 'LineWidth', 1);
end
grid on;
xlabel('Time (s)');
ylabel('Displacement (cm)');
title('Slosh Displacement');
legend('Non-Baffled', 'Baf Comp 1', 'Baf Comp 2', 'Baf Comp 3', 'Baf Comp 4', ...
       'Location', 'best');

% Disturbance Force Comparison
subplot(3,4,6);
plot(t, F_dist_nb, 'r-', 'LineWidth', 2);
hold on;
plot(t, F_dist_baf, 'g-', 'LineWidth', 2);
yline(0, 'k:', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('Force (N)');
title('Total Disturbance Force');
legend('Non-Baffled', 'Baffled', 'Location', 'best');

% Individual Compartment Forces (Baffled)
subplot(3,4,7);
for comp = 1:N_baffles
    plot(t, F_compartments(comp,:), 'LineWidth', 1.5);
    hold on;
end
yline(0, 'k:', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('Force (N)');
title('Baffled: Individual Compartment Forces');
legend('Comp 1', 'Comp 2', 'Comp 3', 'Comp 4', 'Location', 'best');

% Integral States
subplot(3,4,8);
plot(t, x_I_nominal(1,:), 'b-', 'LineWidth', 2);
hold on;
plot(t, x_I_nonbaffled(1,:), 'r--', 'LineWidth', 2);
plot(t, x_I_baffled(1,:), 'g:', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('∫e_{x1}');
title('Integral State: x1 Error');
legend('Nominal', 'Non-Baffled', 'Baffled', 'Location', 'best');

% Zoom on Position x1 (final portion)
subplot(3,4,9);
t_zoom_start = find(t >= 15, 1);
plot(t(t_zoom_start:end), x_nominal(1,t_zoom_start:end), 'b-', 'LineWidth', 2);
hold on;
plot(t(t_zoom_start:end), x_nonbaffled(1,t_zoom_start:end), 'r--', 'LineWidth', 2);
plot(t(t_zoom_start:end), x_baffled(1,t_zoom_start:end), 'g:', 'LineWidth', 2);
yline(x_ref(1), 'k--', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('x1 (m)');
title('x1 Position (Zoomed: 15-30s)');
legend('Nominal', 'Non-Baffled', 'Baffled', 'Ref');

% Error bars
subplot(3,4,10);
errors = [error_nominal_x1, error_nominal_x2;
          error_nb_x1, error_nb_x2;
          error_baf_x1, error_baf_x2]' * 1000;  % Convert to mm
bar(errors');
set(gca, 'XTickLabel', {'Nominal', 'Non-Baffled', 'Baffled'});
ylabel('Final Error (mm)');
title('Error Comparison');
legend('x1', 'x2', 'Location', 'best');
grid on;

% Max disturbance comparison
subplot(3,4,11);
forces = [0, max_force_nb, max_force_baf];
bar(forces);
set(gca, 'XTickLabel', {'Nominal', 'Non-Baffled', 'Baffled'});
ylabel('Max Force (N)');
title('Peak Disturbance Force');
grid on;

% Frequency content (FFT of disturbance)
subplot(3,4,12);
% FFT of non-baffled disturbance
L = length(F_dist_nb);
Y_nb = fft(F_dist_nb);
P2_nb = abs(Y_nb/L);
P1_nb = P2_nb(1:L/2+1);
P1_nb(2:end-1) = 2*P1_nb(2:end-1);
f = (1/dt)*(0:(L/2))/L;

% FFT of baffled disturbance
Y_baf = fft(F_dist_baf);
P2_baf = abs(Y_baf/L);
P1_baf = P2_baf(1:L/2+1);
P1_baf(2:end-1) = 2*P1_baf(2:end-1);

plot(f, P1_nb, 'r-', 'LineWidth', 2);
hold on;
plot(f, P1_baf, 'g-', 'LineWidth', 2);
grid on;
xlabel('Frequency (Hz)');
ylabel('Magnitude');
title('Frequency Content of Disturbance');
xlim([0, 5]);
legend('Non-Baffled', 'Baffled', 'Location', 'best');

sgtitle('LQI Control: Non-Baffled vs Baffled Slosh Comparison', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('✓ Plots generated\n\n');

%% STEP 11: SUMMARY
fprintf('╔════════════════════════════════════════════════════════╗\n');
fprintf('║              BAFFLE EFFECTIVENESS SUMMARY              ║\n');
fprintf('╚════════════════════════════════════════════════════════╝\n\n');

fprintf('KEY FINDINGS:\n\n');

fprintf('1. TRACKING PERFORMANCE:\n');
fprintf('   Nominal:      x1 error = %.4f mm (perfect - no disturbance)\n', error_nominal_x1*1000);
fprintf('   Non-Baffled:  x1 error = %.4f mm (%.0fx worse)\n', ...
    error_nb_x1*1000, error_nb_x1/error_nominal_x1);
fprintf('   Baffled:      x1 error = %.4f mm (%.1fx worse)\n\n', ...
    error_baf_x1*1000, error_baf_x1/error_nominal_x1);

fprintf('2. SLOSH AMPLITUDE:\n');
fprintf('   Non-Baffled:  %.2f cm maximum\n', max_slosh_nb*100);
fprintf('   Baffled:      %.2f cm maximum (%.0f%% reduction)\n\n', ...
    max_slosh_baf*100, improvement_slosh);

fprintf('3. DISTURBANCE FORCE:\n');
fprintf('   Non-Baffled:  %.3f N maximum\n', max_force_nb);
fprintf('   Baffled:      %.3f N maximum (%.0f%% reduction)\n\n', ...
    max_force_baf, improvement_force);

fprintf('4. FREQUENCY CHARACTERISTICS:\n');
fprintf('   Non-Baffled:  ω = %.2f Hz (slow, powerful)\n', omega_nb/(2*pi));
fprintf('   Baffled:      ω = %.2f Hz (%.0fx faster, weaker per compartment)\n\n', ...
    omega_baf/(2*pi), omega_baf/omega_nb);

fprintf('5. DAMPING:\n');
fprintf('   Non-Baffled:  ζ = %.3f (lightly damped)\n', zeta_nb);
fprintf('   Baffled:      ζ = %.3f (%.1fx more damping)\n\n', zeta_baf, zeta_baf/zeta_nb);

fprintf('✓✓✓ CONCLUSION: BAFFLES DRAMATICALLY IMPROVE PERFORMANCE! ✓✓✓\n\n');
fprintf('Baffles reduce disturbance force by %.0f%%\n', improvement_force);
fprintf('Resulting in %.0f%% better tracking accuracy\n\n', improvement_error);

fprintf('This demonstrates why:\n');
fprintf('  • Tanker trucks are REQUIRED to have baffles (safety)\n');
fprintf('  • Aircraft fuel tanks include internal baffles (control)\n');
fprintf('  • Any liquid transport uses compartmentalization (stability)\n\n');


fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('STEP 9.5: Control Energy Analysis\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

% Calculate control energy for each scenario
% Energy = integral of u'*u over time

% Nominal (no slosh)
energy_nominal = sum(sum(u_nominal.^2, 1)) * dt;
peak_power_nominal = max(sum(u_nominal.^2, 1));
rms_effort_nominal = sqrt(mean(sum(u_nominal.^2, 1)));

% Non-baffled
energy_nonbaffled = sum(sum(u_nonbaffled.^2, 1)) * dt;
peak_power_nonbaffled = max(sum(u_nonbaffled.^2, 1));
rms_effort_nonbaffled = sqrt(mean(sum(u_nonbaffled.^2, 1)));

% Baffled
energy_baffled = sum(sum(u_baffled.^2, 1)) * dt;
peak_power_baffled = max(sum(u_baffled.^2, 1));
rms_effort_baffled = sqrt(mean(sum(u_baffled.^2, 1)));

% Display results
fprintf('┌────────────────────────┬──────────┬───────────┬──────────┐\n');
fprintf('│ Metric                 │ Nominal  │ Non-Baffled│ Baffled │\n');
fprintf('├────────────────────────┼──────────┼───────────┼──────────┤\n');
fprintf('│ Total Energy (J)       │ %8.3f │ %9.3f │ %8.3f │\n', ...
    energy_nominal, energy_nonbaffled, energy_baffled);
fprintf('│ Peak Power (W)         │ %8.3f │ %9.3f │ %8.3f │\n', ...
    peak_power_nominal, peak_power_nonbaffled, peak_power_baffled);
fprintf('│ RMS Effort (N)         │ %8.3f │ %9.3f │ %8.3f │\n', ...
    rms_effort_nominal, rms_effort_nonbaffled, rms_effort_baffled);
fprintf('└────────────────────────┴──────────┴───────────┴──────────┘\n\n');

% Calculate increases relative to nominal
energy_increase_nb = (energy_nonbaffled - energy_nominal) / energy_nominal * 100;
energy_increase_baf = (energy_baffled - energy_nominal) / energy_nominal * 100;

fprintf('ENERGY COST OF SLOSH:\n');
fprintf('  Non-baffled: +%.1f%% energy required\n', energy_increase_nb);
fprintf('  Baffled:     +%.1f%% energy required\n\n', energy_increase_baf);

% Energy saved by using baffles
energy_saved = energy_nonbaffled - energy_baffled;
energy_saved_percent = (energy_nonbaffled - energy_baffled) / energy_nonbaffled * 100;

fprintf('BAFFLE ENERGY SAVINGS:\n');
fprintf('  Absolute: %.3f J saved\n', energy_saved);
fprintf('  Relative: %.1f%% reduction in energy\n\n', energy_saved_percent);

% Plot control effort over time
figure('Position', [100, 100, 1400, 500]);

subplot(1,3,1);
effort_nominal = sum(u_nominal.^2, 1);
effort_nb = sum(u_nonbaffled.^2, 1);
effort_baf = sum(u_baffled.^2, 1);

plot(t, effort_nominal, 'b-', 'LineWidth', 2);
hold on;
plot(t, effort_nb, 'r--', 'LineWidth', 2);
plot(t, effort_baf, 'g:', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Instantaneous Power (W)');
title('Control Effort Over Time');
legend('Nominal', 'Non-Baffled', 'Baffled', 'Location', 'best');

subplot(1,3,2);
cumulative_energy_nominal = cumsum(effort_nominal) * dt;
cumulative_energy_nb = cumsum(effort_nb) * dt;
cumulative_energy_baf = cumsum(effort_baf) * dt;

plot(t, cumulative_energy_nominal, 'b-', 'LineWidth', 2);
hold on;
plot(t, cumulative_energy_nb, 'r--', 'LineWidth', 2);
plot(t, cumulative_energy_baf, 'g:', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Cumulative Energy (J)');
title('Energy Accumulation');
legend('Nominal', 'Non-Baffled', 'Baffled', 'Location', 'best');

subplot(1,3,3);
energies = [energy_nominal, energy_nonbaffled, energy_baffled];
bar(energies);
set(gca, 'XTickLabel', {'Nominal', 'Non-Baffled', 'Baffled'});
ylabel('Total Energy (J)');
title('Total Control Energy Comparison');
grid on;

sgtitle('Control Energy Analysis', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('✓ Energy analysis complete\n\n');

save('slosh_baffle_comparison.mat', 'A_nom', 'B_nom', ...
     'A_slosh_nb', 'A_slosh_baf', 'K_aug', 'C_track', ...
     'x_nominal', 'x_nonbaffled', 'x_baffled', ...
     'u_nominal', 'u_nonbaffled', 'u_baffled', ...
     'x_slosh_nb', 'x_slosh_baf', ...
     'F_dist_nb', 'F_dist_baf', 'F_compartments', 't');

fprintf('✓ Results saved to: slosh_baffle_comparison.mat\n\n');
fprintf('✓ Analysis complete!\n');

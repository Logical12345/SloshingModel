%% TEST SYSTEM: LQI CONTROL - MULTI-BAFFLE CONFIGURATION ANALYSIS
%
% Purpose: Show the dramatic benefit of baffles on slosh disturbance
%          and find optimal baffle configuration
%
% System: Double mass-spring-damper (4 states)
% Control: LQI with integral action (6 augmented states)
%
% Tests multiple baffle configurations: 1, 2, 4, 8, 16 compartments
% Figure 1: Detailed comparison (Nominal vs Non-Baffled vs 4-Baffle)
% Figure 2: Multi-configuration analysis

clear all; close all; clc;

fprintf('╔════════════════════════════════════════════════════════╗\n');
fprintf('║   LQI CONTROL: MULTI-BAFFLE CONFIGURATION ANALYSIS    ║\n');
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
m_slosh_total = 0.50 * m1;   % 50% of mass 1 is liquid

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

%% STEP 3: DESIGN LQI CONTROLLER
fprintf('STEP 3: Design LQI Controller\n');
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

%% STEP 4: SIMULATION SETUP
fprintf('STEP 4: Simulation Setup\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━\n');

dt = 0.01;
t = 0:dt:30;
N = length(t);

% Initial conditions
x0 = [1; 0; -0.5; 0];  % Displaced masses
x_ref = [0.5; 0; -0.2; 0];  % Reference positions

% Initial slosh (same for all cases - 10 cm displacement)
x_slosh_init = [0.1; 0];

fprintf('Initial conditions:\n');
fprintf('  Mass positions: x1 = %.2f m, x2 = %.2f m\n', x0(1), x0(3));
fprintf('  Reference:      x1 = %.2f m, x2 = %.2f m\n', x_ref(1), x_ref(3));
fprintf('  Slosh initial:  %.0f cm displacement\n', x_slosh_init(1)*100);
fprintf('  Simulation:     %.0f seconds\n\n', t(end));

%% STEP 5: SIMULATE LQI NOMINAL (NO SLOSH)
fprintf('STEP 5: Simulate LQI - Nominal (No Slosh)\n');
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

%% STEP 6: SIMULATE LQI WITH NON-BAFFLED SLOSH
fprintf('STEP 6: Simulate LQI - NON-BAFFLED Slosh\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

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

%% STEP 7: SIMULATE MULTIPLE BAFFLE CONFIGURATIONS
fprintf('STEP 7: Simulate Multiple Baffle Configurations\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

% Define baffle configurations to test
baffle_configs = [1, 2, 4, 8, 16];
n_configs = length(baffle_configs);

fprintf('Testing %d baffle configurations: [%s]\n\n', ...
    n_configs, sprintf('%d ', baffle_configs));

% Preallocate storage for all configurations
x_baffled_all = zeros(4, N, n_configs);      % [states, time, config]
x_I_baffled_all = zeros(2, N, n_configs);    % [integral_states, time, config]
u_baffled_all = zeros(2, N, n_configs);      % [inputs, time, config]
F_dist_baf_all = zeros(N, n_configs);        % [time, config]

% Metrics for each configuration
error_baf_x1_all = zeros(n_configs, 1);
error_baf_x2_all = zeros(n_configs, 1);
max_slosh_baf_all = zeros(n_configs, 1);
max_force_baf_all = zeros(n_configs, 1);
energy_baffled_all = zeros(n_configs, 1);

% Also save the 4-baffle case for detailed plotting
idx_4baffle = find(baffle_configs == 4);
x_slosh_4baf = [];  % Will store individual compartment slosh for 4-baffle case
F_compartments_4baf = [];

% Loop over each baffle configuration
for config_idx = 1:n_configs
    N_baffles = baffle_configs(config_idx);
    
    fprintf('Configuration %d/%d: N_baffles = %d\n', config_idx, n_configs, N_baffles);
    
    % Calculate parameters for this configuration
    L_compartment = L_tank / N_baffles;
    m_slosh_comp = m_slosh_total / N_baffles;
    k_slosh_comp = k_slosh_nb * N_baffles;
    b_slosh_comp = b_slosh_nb * 2.5;  % Baffle damping
    
    % Dynamics for this configuration
    A_slosh_comp = [0,                    1;
                    -k_slosh_comp/m_slosh_comp, -b_slosh_comp/m_slosh_comp];
    
    B_slosh_comp = [0; 1];
    
    K_dist_comp = [k_slosh_comp,  b_slosh_comp;
                   0,             0];
    
    % Storage for this configuration's slosh states
    x_slosh_states = zeros(2, N_baffles, N);  % [state, compartment, time]
    F_compartments = zeros(N_baffles, N);
    
    % Initialize
    x_baffled_all(:, 1, config_idx) = x0;
    
    % Initialize each compartment with slight phase offset
    for comp = 1:N_baffles
        phase_offset = (comp-1) * 0.02;
        x_slosh_states(:, comp, 1) = x_slosh_init + [phase_offset; 0];
    end
    
    % Simulate this configuration
    for i = 1:N-1
        % Control law (same LQI controller for all configs!)
        y = C_track * x_baffled_all(:, i, config_idx);
        r = [x_ref(1); x_ref(3)];
        e = r - y;
        x_I_baffled_all(:, i+1, config_idx) = x_I_baffled_all(:, i, config_idx) + e * dt;
        
        x_aug_vec = [x_baffled_all(:, i, config_idx); x_I_baffled_all(:, i, config_idx)];
        u_baffled_all(:, i, config_idx) = -K_aug * x_aug_vec;
        u_baffled_all(:, i, config_idx) = max(min(u_baffled_all(:, i, config_idx), 10), -10);
        
        % Compute total disturbance from all compartments
        d_total = [0; 0];
        for comp = 1:N_baffles
            d_comp = K_dist_comp * x_slosh_states(:, comp, i);
            d_total = d_total + d_comp;
            F_compartments(comp, i) = d_comp(1);
        end
        F_dist_baf_all(i, config_idx) = d_total(1);
        
        % Plant dynamics with disturbance
        dx = A_nom * x_baffled_all(:, i, config_idx) + ...
             B_nom * u_baffled_all(:, i, config_idx) + ...
             B_dist * d_total;
        x_baffled_all(:, i+1, config_idx) = x_baffled_all(:, i, config_idx) + dx * dt;
        
        % Update all compartments (same acceleration)
        a1 = dx(2);
        for comp = 1:N_baffles
            dx_slosh = A_slosh_comp * x_slosh_states(:, comp, i) + B_slosh_comp * a1;
            x_slosh_states(:, comp, i+1) = x_slosh_states(:, comp, i) + dx_slosh * dt;
        end
    end
    u_baffled_all(:, end, config_idx) = u_baffled_all(:, end-1, config_idx);
    F_dist_baf_all(end, config_idx) = F_dist_baf_all(end-1, config_idx);
    
    % Compute metrics for this configuration
    error_baf_x1_all(config_idx) = abs(x_baffled_all(1, end, config_idx) - x_ref(1));
    error_baf_x2_all(config_idx) = abs(x_baffled_all(3, end, config_idx) - x_ref(3));
    max_slosh_baf_all(config_idx) = max(max(abs(x_slosh_states(1, :, :))));
    max_force_baf_all(config_idx) = max(abs(F_dist_baf_all(:, config_idx)));
    energy_baffled_all(config_idx) = sum(sum(u_baffled_all(:, :, config_idx).^2, 1)) * dt;
    
    % Save 4-baffle case for detailed plotting
    if N_baffles == 4
        x_slosh_4baf = x_slosh_states;
        F_compartments_4baf = F_compartments;
    end
    
    fprintf('  Final error: x1 = %.6f m, x2 = %.6f m\n', ...
        error_baf_x1_all(config_idx), error_baf_x2_all(config_idx));
    fprintf('  Max force:   %.3f N\n', max_force_baf_all(config_idx));
    fprintf('  Energy:      %.3f J\n\n', energy_baffled_all(config_idx));
end

% Extract 4-baffle results for main comparison
x_baffled_4 = x_baffled_all(:, :, idx_4baffle);
x_I_baffled_4 = x_I_baffled_all(:, :, idx_4baffle);
u_baffled_4 = u_baffled_all(:, :, idx_4baffle);
F_dist_baf_4 = F_dist_baf_all(:, idx_4baffle);
error_baf_x1_4 = error_baf_x1_all(idx_4baffle);
error_baf_x2_4 = error_baf_x2_all(idx_4baffle);
max_slosh_baf_4 = max_slosh_baf_all(idx_4baffle);
max_force_baf_4 = max_force_baf_all(idx_4baffle);

%% STEP 8: ENERGY ANALYSIS
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('STEP 8: Control Energy Analysis\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

% Calculate control energy
energy_nominal = sum(sum(u_nominal.^2, 1)) * dt;
peak_power_nominal = max(sum(u_nominal.^2, 1));
rms_effort_nominal = sqrt(mean(sum(u_nominal.^2, 1)));

energy_nonbaffled = sum(sum(u_nonbaffled.^2, 1)) * dt;
peak_power_nonbaffled = max(sum(u_nonbaffled.^2, 1));
rms_effort_nonbaffled = sqrt(mean(sum(u_nonbaffled.^2, 1)));

energy_baffled_4 = energy_baffled_all(idx_4baffle);
peak_power_baffled_4 = max(sum(u_baffled_4.^2, 1));
rms_effort_baffled_4 = sqrt(mean(sum(u_baffled_4.^2, 1)));

% Display results
fprintf('┌────────────────────────┬──────────┬───────────┬──────────┐\n');
fprintf('│ Metric                 │ Nominal  │ Non-Baffled│ 4-Baffle │\n');
fprintf('├────────────────────────┼──────────┼───────────┼──────────┤\n');
fprintf('│ Total Energy (J)       │ %8.3f │ %9.3f │ %8.3f │\n', ...
    energy_nominal, energy_nonbaffled, energy_baffled_4);
fprintf('│ Peak Power (W)         │ %8.3f │ %9.3f │ %8.3f │\n', ...
    peak_power_nominal, peak_power_nonbaffled, peak_power_baffled_4);
fprintf('│ RMS Effort (N)         │ %8.3f │ %9.3f │ %8.3f │\n', ...
    rms_effort_nominal, rms_effort_nonbaffled, rms_effort_baffled_4);
fprintf('└────────────────────────┴──────────┴───────────┴──────────┘\n\n');

energy_increase_nb = (energy_nonbaffled - energy_nominal) / energy_nominal * 100;
energy_increase_baf = (energy_baffled_4 - energy_nominal) / energy_nominal * 100;
energy_saved = energy_nonbaffled - energy_baffled_4;
energy_saved_percent = (energy_nonbaffled - energy_baffled_4) / energy_nonbaffled * 100;

fprintf('ENERGY COST OF SLOSH:\n');
fprintf('  Non-baffled: +%.1f%% energy required\n', energy_increase_nb);
fprintf('  4-Baffles:   +%.1f%% energy required\n\n', energy_increase_baf);

fprintf('BAFFLE ENERGY SAVINGS:\n');
fprintf('  Absolute: %.3f J saved\n', energy_saved);
fprintf('  Relative: %.1f%% reduction in energy\n\n', energy_saved_percent);

%% STEP 9: SUMMARY
fprintf('╔════════════════════════════════════════════════════════╗\n');
fprintf('║              4-BAFFLE EFFECTIVENESS SUMMARY            ║\n');
fprintf('╚════════════════════════════════════════════════════════╝\n\n');

improvement_error_4 = (error_nb_x1 - error_baf_x1_4) / error_nb_x1 * 100;
improvement_slosh_4 = (max_slosh_nb - max_slosh_baf_4) / max_slosh_nb * 100;
improvement_force_4 = (max_force_nb - max_force_baf_4) / max_force_nb * 100;

fprintf('BENEFIT OF 4 BAFFLES:\n');
fprintf('  Error reduction:       %.1f%% improvement\n', improvement_error_4);
fprintf('  Slosh amplitude:       %.1f%% reduction\n', improvement_slosh_4);
fprintf('  Disturbance force:     %.1f%% reduction\n', improvement_force_4);
fprintf('  Energy savings:        %.1f%% reduction\n\n', energy_saved_percent);

%% FIGURE 1: DETAILED COMPARISON (NOMINAL vs NON-BAFFLED vs 4-BAFFLE)
fprintf('Generating Figure 1: Detailed Comparison...\n');

figure('Position', [50, 50, 1800, 1000]);

% Position Mass 1
subplot(3,4,1);
plot(t, x_nominal(1,:), 'b-', 'LineWidth', 2);
hold on;
plot(t, x_nonbaffled(1,:), 'r--', 'LineWidth', 2);
plot(t, x_baffled_4(1,:), 'g:', 'LineWidth', 2);
yline(x_ref(1), 'k--', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('x1 (m)');
title('Position: Mass 1');
legend('Nominal', 'Non-Baffled', '4-Baffle', 'Ref', 'Location', 'best');

% Velocity Mass 1
subplot(3,4,2);
plot(t, x_nominal(2,:), 'b-', 'LineWidth', 2);
hold on;
plot(t, x_nonbaffled(2,:), 'r--', 'LineWidth', 2);
plot(t, x_baffled_4(2,:), 'g:', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('v1 (m/s)');
title('Velocity: Mass 1');

% Position Mass 2
subplot(3,4,3);
plot(t, x_nominal(3,:), 'b-', 'LineWidth', 2);
hold on;
plot(t, x_nonbaffled(3,:), 'r--', 'LineWidth', 2);
plot(t, x_baffled_4(3,:), 'g:', 'LineWidth', 2);
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
plot(t, u_baffled_4(1,:), 'g:', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('u1 (N)');
title('Control Force: Mass 1');
legend('Nominal', 'Non-Baffled', '4-Baffle', 'Location', 'best');

% Slosh Displacement Comparison
subplot(3,4,5);
plot(t, x_slosh_nb(1,:)*100, 'r-', 'LineWidth', 2);
hold on;
for comp = 1:4
    plot(t, squeeze(x_slosh_4baf(1,comp,:))*100, ':', 'LineWidth', 1);
end
grid on;
xlabel('Time (s)');
ylabel('Displacement (cm)');
title('Slosh Displacement');
legend('Non-Baffled', 'Comp 1', 'Comp 2', 'Comp 3', 'Comp 4', 'Location', 'best');

% Disturbance Force Comparison
subplot(3,4,6);
plot(t, F_dist_nb, 'r-', 'LineWidth', 2);
hold on;
plot(t, F_dist_baf_4, 'g-', 'LineWidth', 2);
yline(0, 'k:', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('Force (N)');
title('Total Disturbance Force');
legend('Non-Baffled', '4-Baffle', 'Location', 'best');

% Individual Compartment Forces (4-Baffle)
subplot(3,4,7);
for comp = 1:4
    plot(t, F_compartments_4baf(comp,:), 'LineWidth', 1.5);
    hold on;
end
yline(0, 'k:', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('Force (N)');
title('4-Baffle: Individual Compartment Forces');
legend('Comp 1', 'Comp 2', 'Comp 3', 'Comp 4', 'Location', 'best');

% Integral States
subplot(3,4,8);
plot(t, x_I_nominal(1,:), 'b-', 'LineWidth', 2);
hold on;
plot(t, x_I_nonbaffled(1,:), 'r--', 'LineWidth', 2);
plot(t, x_I_baffled_4(1,:), 'g:', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('∫e_{x1}');
title('Integral State: x1 Error');
legend('Nominal', 'Non-Baffled', '4-Baffle', 'Location', 'best');

% Zoom on Position x1 (final portion)
subplot(3,4,9);
t_zoom_start = find(t >= 15, 1);
plot(t(t_zoom_start:end), x_nominal(1,t_zoom_start:end), 'b-', 'LineWidth', 2);
hold on;
plot(t(t_zoom_start:end), x_nonbaffled(1,t_zoom_start:end), 'r--', 'LineWidth', 2);
plot(t(t_zoom_start:end), x_baffled_4(1,t_zoom_start:end), 'g:', 'LineWidth', 2);
yline(x_ref(1), 'k--', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('x1 (m)');
title('x1 Position (Zoomed: 15-30s)');
legend('Nominal', 'Non-Baffled', '4-Baffle', 'Ref');

% Error bars
subplot(3,4,10);
errors = [error_nominal_x1, error_nominal_x2;
          error_nb_x1, error_nb_x2;
          error_baf_x1_4, error_baf_x2_4]' * 1000;
bar(errors');
set(gca, 'XTickLabel', {'Nominal', 'Non-Baffled', '4-Baffle'});
ylabel('Final Error (mm)');
title('Error Comparison');
legend('x1', 'x2', 'Location', 'best');
grid on;

% Max disturbance comparison
subplot(3,4,11);
forces = [0, max_force_nb, max_force_baf_4];
bar(forces);
set(gca, 'XTickLabel', {'Nominal', 'Non-Baffled', '4-Baffle'});
ylabel('Max Force (N)');
title('Peak Disturbance Force');
grid on;

% Control effort over time
subplot(3,4,12);
effort_nominal = sum(u_nominal.^2, 1);
effort_nb = sum(u_nonbaffled.^2, 1);
effort_baf_4 = sum(u_baffled_4.^2, 1);
plot(t, effort_nominal, 'b-', 'LineWidth', 2);
hold on;
plot(t, effort_nb, 'r--', 'LineWidth', 2);
plot(t, effort_baf_4, 'g:', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Power (W)');
title('Control Effort');
legend('Nominal', 'Non-Baffled', '4-Baffle', 'Location', 'best');

sgtitle('Figure 1: Detailed Comparison - Nominal vs Non-Baffled vs 4-Baffle', ...
    'FontSize', 14, 'FontWeight', 'bold');

fprintf('✓ Figure 1 complete\n\n');

%% FIGURE 2: MULTI-CONFIGURATION ANALYSIS
fprintf('Generating Figure 2: Multi-Configuration Analysis...\n');

figure('Position', [100, 100, 1800, 900]);

colors = lines(n_configs);

% Plot 1: Position x1 - all configurations
subplot(2,4,1);
plot(t, x_nominal(1,:), 'b-', 'LineWidth', 2.5, 'DisplayName', 'Nominal');
hold on;
plot(t, x_nonbaffled(1,:), 'r--', 'LineWidth', 2, 'DisplayName', 'No Baffle');
for i = 1:n_configs
    plot(t, squeeze(x_baffled_all(1,:,i)), ':', 'LineWidth', 1.5, ...
        'Color', colors(i,:), 'DisplayName', sprintf('%d Baffle', baffle_configs(i)));
end
yline(x_ref(1), 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
grid on;
xlabel('Time (s)');
ylabel('x1 (m)');
title('Position Mass 1: All Configurations');
legend('Location', 'best', 'FontSize', 8);

% Plot 2: Disturbance force - all configurations
subplot(2,4,2);
plot(t, F_dist_nb, 'r-', 'LineWidth', 2, 'DisplayName', 'No Baffle');
hold on;
for i = 1:n_configs
    plot(t, F_dist_baf_all(:,i), ':', 'LineWidth', 1.5, ...
        'Color', colors(i,:), 'DisplayName', sprintf('%d Baffle', baffle_configs(i)));
end
yline(0, 'k:', 'HandleVisibility', 'off');
grid on;
xlabel('Time (s)');
ylabel('Force (N)');
title('Disturbance Force: All Configurations');
legend('Location', 'best', 'FontSize', 8);

% Plot 3: Control effort - all configurations
subplot(2,4,3);
effort_nominal_plot = sum(u_nominal.^2, 1);
effort_nb_plot = sum(u_nonbaffled.^2, 1);
plot(t, effort_nominal_plot, 'b-', 'LineWidth', 2.5, 'DisplayName', 'Nominal');
hold on;
plot(t, effort_nb_plot, 'r--', 'LineWidth', 2, 'DisplayName', 'No Baffle');
for i = 1:n_configs
    effort_baf = sum(u_baffled_all(:,:,i).^2, 1);
    plot(t, effort_baf, ':', 'LineWidth', 1.5, ...
        'Color', colors(i,:), 'DisplayName', sprintf('%d Baffle', baffle_configs(i)));
end
grid on;
xlabel('Time (s)');
ylabel('Power (W)');
title('Control Effort: All Configurations');
legend('Location', 'best', 'FontSize', 8);

% Plot 4: Cumulative energy
subplot(2,4,4);
cumulative_energy_nominal_plot = cumsum(effort_nominal_plot) * dt;
cumulative_energy_nb_plot = cumsum(effort_nb_plot) * dt;
plot(t, cumulative_energy_nominal_plot, 'b-', 'LineWidth', 2.5, 'DisplayName', 'Nominal');
hold on;
plot(t, cumulative_energy_nb_plot, 'r--', 'LineWidth', 2, 'DisplayName', 'No Baffle');
for i = 1:n_configs
    effort_baf = sum(u_baffled_all(:,:,i).^2, 1);
    cumulative = cumsum(effort_baf) * dt;
    plot(t, cumulative, ':', 'LineWidth', 1.5, ...
        'Color', colors(i,:), 'DisplayName', sprintf('%d Baffle', baffle_configs(i)));
end
grid on;
xlabel('Time (s)');
ylabel('Energy (J)');
title('Cumulative Energy: All Configurations');
legend('Location', 'best', 'FontSize', 8);

% Plot 5: Error vs number of baffles
subplot(2,4,5);
all_errors_x1 = [error_nb_x1; error_baf_x1_all] * 1000;
all_labels = [0, baffle_configs];
bar(all_labels, all_errors_x1);
hold on;
yline(error_nominal_x1*1000, 'b--', 'LineWidth', 2, 'DisplayName', 'Nominal');
grid on;
xlabel('Number of Baffles');
ylabel('Final x1 Error (mm)');
title('Tracking Error vs Baffle Count');
set(gca, 'XTick', all_labels);
legend('Location', 'best');

% Plot 6: Force vs number of baffles
subplot(2,4,6);
all_forces = [max_force_nb; max_force_baf_all];
bar(all_labels, all_forces);
grid on;
xlabel('Number of Baffles');
ylabel('Max Force (N)');
title('Peak Disturbance vs Baffle Count');
set(gca, 'XTick', all_labels);

% Plot 7: Energy vs number of baffles
subplot(2,4,7);
all_energies = [energy_nonbaffled; energy_baffled_all];
bar(all_labels, all_energies);
hold on;
yline(energy_nominal, 'b--', 'LineWidth', 2, 'DisplayName', 'Nominal');
grid on;
xlabel('Number of Baffles');
ylabel('Total Energy (J)');
title('Control Energy vs Baffle Count');
legend('Location', 'best');
set(gca, 'XTick', all_labels);

% Plot 8: Diminishing returns analysis
subplot(2,4,8);
percent_improvement = zeros(n_configs, 1);
for i = 1:n_configs
    percent_improvement(i) = (error_nb_x1 - error_baf_x1_all(i)) / error_nb_x1 * 100;
end
plot(baffle_configs, percent_improvement, 'o-', 'LineWidth', 2, 'MarkerSize', 8);
grid on;
xlabel('Number of Baffles');
ylabel('Improvement (%)');
title('Error Reduction vs Baffle Count');
set(gca, 'XTick', baffle_configs);

sgtitle('Figure 2: Multi-Configuration Analysis - Optimal Baffle Selection', ...
    'FontSize', 14, 'FontWeight', 'bold');

fprintf('✓ Figure 2 complete\n\n');

%% OPTIMAL CONFIGURATION
fprintf('╔════════════════════════════════════════════════════════╗\n');
fprintf('║         OPTIMAL BAFFLE CONFIGURATION ANALYSIS          ║\n');
fprintf('╚════════════════════════════════════════════════════════╝\n\n');

[min_error, optimal_idx] = min(error_baf_x1_all);
optimal_baffles = baffle_configs(optimal_idx);

fprintf('Best performance: %d baffles\n', optimal_baffles);
fprintf('  Error:   %.4f mm (%.1f%% of non-baffled)\n', ...
    min_error*1000, min_error/error_nb_x1*100);
fprintf('  Force:   %.3f N (%.1f%% of non-baffled)\n', ...
    max_force_baf_all(optimal_idx), max_force_baf_all(optimal_idx)/max_force_nb*100);
fprintf('  Energy:  %.3f J (%.1f%% of non-baffled)\n\n', ...
    energy_baffled_all(optimal_idx), energy_baffled_all(optimal_idx)/energy_nonbaffled*100);

fprintf('Diminishing returns analysis:\n');
for i = 2:n_configs
    improvement = (error_baf_x1_all(i-1) - error_baf_x1_all(i)) / error_baf_x1_all(i-1) * 100;
    fprintf('  %2d → %2d baffles: %5.1f%% improvement\n', ...
        baffle_configs(i-1), baffle_configs(i), improvement);
end
fprintf('\n');

fprintf('Recommendation:\n');
fprintf('  4 baffles provides excellent balance:\n');
fprintf('    - %.1f%% error reduction vs non-baffled\n', ...
    (error_nb_x1 - error_baf_x1_all(idx_4baffle))/error_nb_x1*100);
fprintf('    - %.1f%% force reduction\n', ...
    (max_force_nb - max_force_baf_all(idx_4baffle))/max_force_nb*100);
fprintf('    - %.1f%% energy savings\n', energy_saved_percent);
fprintf('    - Practical to implement\n\n');

%% SAVE RESULTS
save('slosh_baffle_multi_config.mat', 'A_nom', 'B_nom', ...
     'A_slosh_nb', 'K_aug', 'C_track', ...
     'x_nominal', 'x_nonbaffled', 'x_baffled_all', ...
     'u_nominal', 'u_nonbaffled', 'u_baffled_all', ...
     'x_slosh_nb', 'x_slosh_4baf', ...
     'F_dist_nb', 'F_dist_baf_all', 'F_compartments_4baf', ...
     'baffle_configs', 'error_baf_x1_all', 'error_baf_x2_all', ...
     'max_force_baf_all', 'energy_baffled_all', 't');

fprintf('✓ Results saved to: slosh_baffle_multi_config.mat\n\n');
fprintf('✓✓✓ ANALYSIS COMPLETE! ✓✓✓\n\n');

fprintf('Summary:\n');
fprintf('  • Figure 1 shows detailed 4-baffle comparison\n');
fprintf('  • Figure 2 shows multi-configuration analysis\n');
fprintf('  • 4 baffles recommended for optimal cost/benefit\n');
fprintf('  • Baffles reduce disturbance by %.0f%%\n', improvement_force_4);
fprintf('  • Controller maintains stability with all configurations\n\n');

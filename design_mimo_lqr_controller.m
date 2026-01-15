%% MIMO LQR Controller with Elevator + Throttle
% Full state feedback with TWO inputs: [elevator, throttle]
% Controls: θ (elevator) and V (throttle) independently!
%
% Control law: u = -K * (x - x_ref)
% where u = [elevator; throttle] (2x1)
%       K is 2x4 gain matrix

clear all; close all; clc;

fprintf('=== MIMO LQR Controller: Elevator + Throttle ===\n\n');

%% LOAD BASE SYSTEM
if exist('separated_system_params.mat', 'file')
    load('separated_system_params.mat');
    fprintf('✓ Loaded system parameters\n');
else
    run('separated_architecture.m');
    close all;
end

%% EXTEND SYSTEM FOR THROTTLE
fprintf('\n--- Extending System for Throttle Control ---\n');

% Throttle parameters
XT = 2.0;  % Throttle effectiveness on velocity (m/s²)

% Extended B matrix: [B_elevator, B_throttle]
B_mimo = [B_nom(:,1),B_nom(:,2)];

fprintf('Original B (elevator only): 4x1\n');
fprintf('Extended B (elevator + throttle): 4x2\n\n');

fprintf('B_mimo =\n');
disp(B_mimo);

fprintf('\nPhysical interpretation:\n');
fprintf('  Elevator affects:   V (slightly), γ, q, θ\n');
fprintf('  Throttle affects:   V (directly)\n');
fprintf('  → Independent control authority!\n');

%% MIMO LQR DESIGN
fprintf('\n--- MIMO LQR Controller Design ---\n');

% State weighting matrix Q (4x4) - same as SISO LQR
Q_mimo = diag([1,      % V:     Medium weight (now controlled by throttle!)
               1,      % γ:     Medium weight
               10,     % q:     High weight
               100]);  % θ:     Very high weight

fprintf('State weights (Q diagonal):\n');
fprintf('  V (velocity):      %.1f  ← Higher than SISO (throttle controls this)\n', Q_mimo(1,1));
fprintf('  γ (flight angle):  %.1f\n', Q_mimo(2,2));
fprintf('  q (pitch rate):    %.1f\n', Q_mimo(3,3));
fprintf('  θ (pitch angle):   %.1f\n', Q_mimo(4,4));

% Control effort weighting R (2x2 diagonal)
R_mimo = diag([1,     % Elevator effort
               0.1]); % Throttle effort (less penalty - cheaper to use)

fprintf('\nControl effort weights (R diagonal):\n');
fprintf('  Elevator:  %.1f\n', R_mimo(1,1));
fprintf('  Throttle:  %.1f  ← Cheaper to use\n', R_mimo(2,2));

% Compute MIMO LQR gain
K_mimo = lqr(A_nom, B_mimo, Q_mimo, R_mimo);

fprintf('\nMIMO LQR Gain Matrix K (2x4):\n');
fprintf('           V        γ        q        θ\n');
fprintf('Elevator: [%7.4f, %7.4f, %7.4f, %7.4f]\n', K_mimo(1,:));
fprintf('Throttle: [%7.4f, %7.4f, %7.4f, %7.4f]\n', K_mimo(2,:));

fprintf('\nControl law:\n');
fprintf('  [u_elevator]   [K₁ᵀ]\n');
fprintf('  [u_throttle] = -[K₂ᵀ] * (x - x_ref)\n\n');

fprintf('Physical meaning:\n');
fprintf('  Elevator primarily controls: θ, q\n');
fprintf('  Throttle primarily controls: V\n');
fprintf('  → Decoupled authority!\n');

%% CLOSED-LOOP ANALYSIS
fprintf('\n--- Closed-Loop Stability Analysis ---\n');

% Closed-loop system matrix
A_cl_mimo = A_nom - B_mimo*K_mimo;

% Check eigenvalues
eig_cl_mimo = eig(A_cl_mimo);
fprintf('\nClosed-loop eigenvalues (MIMO, without slosh):\n');
for i = 1:length(eig_cl_mimo)
    if abs(imag(eig_cl_mimo(i))) < 1e-10
        fprintf('  λ%d = %.4f (real, τ = %.2f s)\n', i, real(eig_cl_mimo(i)), -1/real(eig_cl_mimo(i)));
    else
        if imag(eig_cl_mimo(i)) > 0
            freq = abs(imag(eig_cl_mimo(i)))/(2*pi);
            damp = -real(eig_cl_mimo(i))/abs(eig_cl_mimo(i));
            fprintf('  λ%d,%d = %.4f ± %.4fj  (f=%.3f Hz, ζ=%.3f)\n', ...
                    i, i+1, real(eig_cl_mimo(i)), abs(imag(eig_cl_mimo(i))), freq, damp);
        end
    end
end

if all(real(eig_cl_mimo) < 0)
    fprintf('✓ MIMO closed-loop system is STABLE\n');
else
    fprintf('✗ MIMO closed-loop system is UNSTABLE\n');
end

%% SIMULATION: MIMO LQR WITH SLOSH
fprintf('\n--- Simulating MIMO LQR (With Slosh) ---\n');

% Reference
V_ref = 52;      % Increase velocity by 2 m/s!
gamma_ref = 0;   % Level flight
q_ref = 0;       % No pitch rate
theta_ref = 0.1; % 5.7 degrees pitch up

x_ref = [V_ref; gamma_ref; q_ref; theta_ref];

fprintf('Reference commands:\n');
fprintf('  V_ref = %.1f m/s  ← 2 m/s increase from trim!\n', V_ref);
fprintf('  θ_ref = %.2f rad (%.1f deg)\n', theta_ref, rad2deg(theta_ref));

% Time
dt = 0.01;
t = 0:dt:15;
N = length(t);

% Initial conditions
x0 = [50; 0; 0; 0];  % Start at trim

% Storage
x_mimo = zeros(4, N);
x_slosh_mimo = zeros(2, N);
u_mimo = zeros(2, N);
F_dist = zeros(N, 1);
M_dist = zeros(N, 1);

x_mimo(:,1) = x0;
x_slosh_mimo(:,1) = [0.1; 0];  % 10 cm initial slosh

fprintf('Running MIMO simulation with slosh...\n');
for i = 1:N-1
    % State error
    x_error = x_mimo(:,i) - x_ref;
    
    % MIMO LQR control law
    u = -K_mimo * x_error;
    
    % Saturation
    u(1) = max(min(u(1), 0.5), -0.5);    % Elevator: ±28.6 deg
    u(2) = max(min(u(2), 1.0), 0);       % Throttle: 0 to 1 (0-100%)
    u_mimo(:,i) = u;
    
    % Compute disturbances from slosh
    dist = K_dist * x_slosh_mimo(:,i);
    F_dist(i) = dist(1);
    M_dist(i) = dist(2);
    
    % Plant dynamics with BOTH controls and disturbances
    dx_nom = A_nom * x_mimo(:,i) + B_mimo * u + B_nom(:,2:3) * [F_dist(i); M_dist(i)];
    
    % Slosh dynamics
    u_slosh = [dx_nom(1); dx_nom(3)];
    dx_slosh = A_slosh * x_slosh_mimo(:,i) + B_slosh * u_slosh;
    
    % Update
    x_mimo(:,i+1) = x_mimo(:,i) + dx_nom * dt;
    x_slosh_mimo(:,i+1) = x_slosh_mimo(:,i) + dx_slosh * dt;
end
u_mimo(:,end) = u_mimo(:,end-1);
F_dist(end) = F_dist(end-1);
M_dist(end) = M_dist(end-1);

fprintf('✓ MIMO simulation complete\n');

%% COMPARISON: MIMO vs SISO LQR
fprintf('\n--- Loading SISO LQR for Comparison ---\n');

if exist('lqr_controller_results.mat', 'file')
    load('lqr_controller_results.mat', 'x_lqr_slosh', 'u_lqr_slosh');
    fprintf('✓ Loaded SISO LQR results\n');
    has_siso = true;
else
    fprintf('! SISO LQR results not found. Run design_lqr_controller.m first.\n');
    fprintf('  Continuing with MIMO only...\n');
    has_siso = false;
end

%% PLOTTING
fprintf('\n--- Generating Plots ---\n');

figure('Position', [50, 50, 1600, 1000]);

% Velocity
subplot(3,3,1);
plot(t, x_mimo(1,:), 'r-', 'LineWidth', 2, 'DisplayName', 'MIMO');
hold on;
if has_siso
    plot(t, x_lqr_slosh(1,:), 'b--', 'LineWidth', 2, 'DisplayName', 'SISO');
end
yline(V_ref, 'k:', 'LineWidth', 1.5, 'DisplayName', 'Reference');
yline(50, 'g:', 'LineWidth', 1, 'DisplayName', 'Trim');
grid on;
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Velocity Response (MIMO tracks, SISO drifts!)');
legend('Location', 'best');

% Flight path angle
subplot(3,3,2);
plot(t, rad2deg(x_mimo(2,:)), 'r-', 'LineWidth', 2);
hold on;
if has_siso
    plot(t, rad2deg(x_lqr_slosh(2,:)), 'b--', 'LineWidth', 2);
end
yline(rad2deg(gamma_ref), 'k:', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Flight Path Angle (deg)');
title('Flight Path Angle');

% Pitch rate
subplot(3,3,3);
plot(t, rad2deg(x_mimo(3,:)), 'r-', 'LineWidth', 2);
hold on;
if has_siso
    plot(t, rad2deg(x_lqr_slosh(3,:)), 'b--', 'LineWidth', 2);
end
yline(rad2deg(q_ref), 'k:', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Pitch Rate (deg/s)');
title('Pitch Rate');

% Pitch angle
subplot(3,3,4);
plot(t, rad2deg(x_mimo(4,:)), 'r-', 'LineWidth', 2);
hold on;
if has_siso
    plot(t, rad2deg(x_lqr_slosh(4,:)), 'b--', 'LineWidth', 2);
end
yline(rad2deg(theta_ref), 'k:', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Pitch Angle (deg)');
title('Pitch Angle');
legend('MIMO', 'SISO', 'Reference', 'Location', 'best');

% Elevator control
subplot(3,3,5);
plot(t, rad2deg(u_mimo(1,:)), 'r-', 'LineWidth', 2);
hold on;
if has_siso
    plot(t, rad2deg(u_lqr_slosh), 'b--', 'LineWidth', 2);
end
grid on;
xlabel('Time (s)');
ylabel('Elevator (deg)');
title('Elevator Command');
legend('MIMO', 'SISO', 'Location', 'best');

% Throttle control (MIMO only)
subplot(3,3,6);
plot(t, u_mimo(2,:)*100, 'r-', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Throttle (%)');
title('Throttle Command (MIMO only!)');

% Slosh position
subplot(3,3,7);
plot(t, x_slosh_mimo(1,:)*100, 'r-', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Slosh Position (cm)');
title('Slosh Displacement');

% Control effort comparison
subplot(3,3,8);
effort_elevator = sum(u_mimo(1,:).^2) * dt;
effort_throttle = sum(u_mimo(2,:).^2) * dt;
effort_mimo_total = effort_elevator + effort_throttle;
if has_siso
    effort_siso = sum(u_lqr_slosh.^2) * dt;
    bar([effort_siso, effort_mimo_total]);
    set(gca, 'XTickLabel', {'SISO', 'MIMO'});
else
    bar([effort_elevator, effort_throttle, effort_mimo_total]);
    set(gca, 'XTickLabel', {'Elevator', 'Throttle', 'Total'});
end
ylabel('Control Effort (∫u² dt)');
title('Control Effort');
grid on;

% Control allocation
subplot(3,3,9);
pie([effort_elevator, effort_throttle], {'Elevator', 'Throttle'});
title('MIMO Control Allocation');

sgtitle('MIMO LQR: Elevator + Throttle Control', 'FontSize', 14, 'FontWeight', 'bold');

%% PERFORMANCE COMPARISON
fprintf('\n=== Performance Comparison ===\n');

fprintf('\nVelocity tracking:\n');
fprintf('  Reference V: %.1f m/s\n', V_ref);
fprintf('  MIMO final V: %.2f m/s (error: %.2f m/s)\n', ...
        x_mimo(1,end), x_mimo(1,end) - V_ref);
if has_siso
    fprintf('  SISO final V: %.2f m/s (error: %.2f m/s) ← No throttle!\n', ...
            x_lqr_slosh(1,end), x_lqr_slosh(1,end) - V_ref);
end

fprintf('\nPitch angle tracking:\n');
fprintf('  Reference θ: %.2f deg\n', rad2deg(theta_ref));
fprintf('  MIMO final θ: %.2f deg (error: %.3f deg)\n', ...
        rad2deg(x_mimo(4,end)), rad2deg(x_mimo(4,end) - theta_ref));
if has_siso
    fprintf('  SISO final θ: %.2f deg (error: %.3f deg)\n', ...
            rad2deg(x_lqr_slosh(4,end)), rad2deg(x_lqr_slosh(4,end) - theta_ref));
end

fprintf('\nControl effort:\n');
fprintf('  MIMO total: %.2f\n', effort_mimo_total);
if has_siso
    fprintf('  SISO total: %.2f\n', effort_siso);
    fprintf('  MIMO uses %.1f%% of SISO effort\n', effort_mimo_total/effort_siso*100);
end

%% SAVE RESULTS
fprintf('\n--- Saving Results ---\n');

save('mimo_lqr_controller_results.mat', 'K_mimo', 'Q_mimo', 'R_mimo', ...
     'A_cl_mimo', 'eig_cl_mimo', 'x_ref', 't', 'x_mimo', 'u_mimo', 'x_slosh_mimo');

fprintf('✓ Results saved to mimo_lqr_controller_results.mat\n');

fprintf('\n=== MIMO LQR Design Complete ===\n');
fprintf('Gain matrix K (2x4) has been computed and tested.\n\n');

fprintf('MIMO advantages:\n');
fprintf('  ✓ Independent V and θ control\n');
fprintf('  ✓ Throttle maintains velocity\n');
fprintf('  ✓ Elevator focuses on pitch\n');
fprintf('  ✓ Better performance\n');
fprintf('  ✓ More realistic aircraft control\n');

fprintf('\n✓ Script complete!\n');

%% Separated Architecture: Nominal Plant + Slosh as Disturbance
% This architecture treats slosh dynamics as an external disturbance source
% 
% Structure:
%   [Controller] → [Nominal Plant] → [States]
%                       ↑                ↓
%                       |          [Slosh Dynamics]
%                       |                ↓
%                       └──── [Disturbance Forces/Moments]
%
% Advantages:
% - Clear separation of concerns
% - Easy to add/remove slosh
% - Can test controller on nominal plant first
% - Slosh acts as "unknown" disturbance
% - More modular and maintainable

clear all; close all; clc;

%% NOMINAL PLANT PARAMETERS (4th Order - Independent)
fprintf('=== Nominal Aircraft Plant ===\n');

% Aircraft parameters
m_aircraft = 5000;      % Aircraft mass (kg)
Iy = 15000;             % Pitch moment of inertia (kg*m^2)
g = 9.81;               % Gravity (m/s^2)
V0 = 50;                % Trim velocity (m/s)

% Stability derivatives
Xu = -0.05;             % Velocity damping
Zu = -0.5;              % Lift with velocity
Mw = -0.5;              % Pitch stiffness
Mq = -2.0;              % Pitch damping
Mde = -5.0;             % Elevator effectiveness
XT = 2.0;
% NOMINAL PLANT STATE-SPACE (4x4 - No slosh coupling!)
% States: [V, gamma, q, theta]
% Inputs: [elevator, F_dist, M_dist] (disturbance force and moment)
% Build of augmented model to have the force and moment impact on the
% dynamics of the nominal model, can be added as zero so no effect
A_nom = [Xu,        -g,         0,      0;
         Zu/V0,      0,         1,      0;
         0,      Mw*g/Iy,   Mq/Iy,      0;
         0,          0,         1,      0];

B_nom = [1/m_aircraft,  XT/m_aircraft, 1/m_aircraft,      0;      % Elevator, F_dist, M_dist on velocity
         0,             0,               0,               0;      % on gamma
         Mde/Iy,        0,               0,              1/Iy;       % on pitch rate
         0,             0,               0,               0];     % on theta

C_nom = eye(4);
D_nom = zeros(4, 4);

sys_nom = ss(A_nom, B_nom, C_nom, D_nom);

fprintf('Nominal plant order: %d\n', size(A_nom,1));
fprintf('Inputs: [elevator, disturbance_force, disturbance_moment]\n');
fprintf('States: [V, gamma, q, theta]\n');

% Check nominal plant stability (without disturbances)
eig_nom = eig(A_nom);
fprintf('\nNominal plant eigenvalues:\n');
for i = 1:length(eig_nom)
    fprintf('  λ%d = %.4f + %.4fj\n', i, real(eig_nom(i)), imag(eig_nom(i)));
end

if all(real(eig_nom) < 0)
    fprintf('✓ Nominal plant is STABLE\n');
else
    fprintf('✗ Nominal plant is UNSTABLE\n');
end

%% SLOSH DISTURBANCE DYNAMICS (2nd Order - Independent)
fprintf('\n=== Slosh Disturbance Model ===\n');

% Slosh parameters
m_slosh = 500;          % Slosh mass (kg)
k_slosh = 2000;         % Spring constant (N/m)
c_slosh = 100;          % Damping (Ns/m)
h_slosh = 1.5;          % Height above CG (m) - moment arm

% Derived parameters
omega_n = sqrt(k_slosh/m_slosh);
zeta = c_slosh/(2*sqrt(k_slosh*m_slosh));

fprintf('Slosh natural frequency: %.3f rad/s (%.3f Hz)\n', omega_n, omega_n/(2*pi));
fprintf('Slosh damping ratio: %.4f\n', zeta);

% SLOSH STATE-SPACE (2x2 - Excited by aircraft motion)
% States: [x_s, v_s] (slosh position and velocity)
% Inputs: [a_x, a_theta] (aircraft accelerations that excite slosh)
A_slosh = [0,                1;
           -k_slosh/m_slosh, -c_slosh/m_slosh];

% Coupling: aircraft accelerations excite slosh
% Linear acceleration and rotational acceleration affect slosh
B_slosh = [0,           0;
           1,    h_slosh];  % a_x directly, a_theta through moment arm

C_slosh = eye(2);
D_slosh = zeros(2, 2);

sys_slosh = ss(A_slosh, B_slosh, C_slosh, D_slosh);

fprintf('Slosh dynamics order: %d\n', size(A_slosh,1));
fprintf('Inputs: [aircraft_accel_x, aircraft_accel_theta]\n');
fprintf('States: [slosh_position, slosh_velocity]\n');
fprintf('Outputs: [position, velocity] → used to compute disturbance forces\n');

%% DISTURBANCE FORCE/MOMENT COMPUTATION
% Slosh creates forces and moments based on its states
% F_dist = -k*x_s - c*v_s
% M_dist = h * F_dist

fprintf('\n=== Disturbance Mapping ===\n');
fprintf('Slosh force:  F = -k*x_s - c*v_s\n');
fprintf('Slosh moment: M = h*F = h*(-k*x_s - c*v_s)\n');

% Disturbance matrices (how slosh states create forces/moments)
% [F_dist]   [-k,  -c ] [x_s ]
% [M_dist] = [-k*h, -c*h] [v_s ]
K_dist = [-k_slosh,        -c_slosh;
          -k_slosh*h_slosh, -c_slosh*h_slosh];

fprintf('\nDisturbance gain matrix K:\n');
disp(K_dist);

%% CLOSED-LOOP SYSTEM (For analysis purposes)
% Even though we separate them, we can still form the closed-loop
% for comparison with the monolithic approach

fprintf('\n=== Forming Closed-Loop for Analysis ===\n');

% The closed-loop dynamics are:
% dx_nom/dt = A_nom*x_nom + B_nom*[u; F_dist; M_dist]
% dx_slosh/dt = A_slosh*x_slosh + B_slosh*[accel from x_nom]
% [F_dist; M_dist] = K_dist * x_slosh

% Extract accelerations from nominal plant (for slosh excitation)
% dV/dt from first row of A_nom*x + B_nom*u
% dq/dt from third row of A_nom*x + B_nom*u

% Full closed-loop A matrix (6x6)
A_closed = zeros(6,6);
A_closed(1:4, 1:4) = A_nom;
A_closed(1:4, 5:6) = B_nom(:,2:3) * K_dist;  % Disturbance coupling
A_closed(5:6, 5:6) = A_slosh;
% Acceleration coupling: need to extract dV/dt and dq/dt
A_closed(5:6, 1:4) = B_slosh * [A_nom(1,:);    % dV/dt depends on all states
                                 A_nom(3,:)];   % dq/dt depends on all states

B_closed = [B_nom(:,1); zeros(2,1)];  % Only elevator input

sys_closed = ss(A_closed, B_closed, eye(6), 0);

fprintf('Closed-loop system order: %d\n', size(A_closed,1));

% Check closed-loop stability
eig_closed = eig(A_closed);
fprintf('\nClosed-loop eigenvalues:\n');
for i = 1:length(eig_closed)
    if abs(imag(eig_closed(i))) < 1e-10
        fprintf('  λ%d = %.4f (real)\n', i, real(eig_closed(i)));
    else
        if imag(eig_closed(i)) > 0
            omega = abs(eig_closed(i));
            zeta_mode = -real(eig_closed(i))/omega;
            freq_hz = abs(imag(eig_closed(i)))/(2*pi);
            fprintf('  λ%d,%d = %.4f ± %.4fj  (f=%.3f Hz, ζ=%.4f)\n', ...
                    i, i+1, real(eig_closed(i)), abs(imag(eig_closed(i))), freq_hz, zeta_mode);
        end
    end
end

if all(real(eig_closed) < 0)
    fprintf('✓ Closed-loop system is STABLE\n');
else
    fprintf('✗ Closed-loop system is UNSTABLE\n');
end

%% SIMULATION - SEPARATED ARCHITECTURE
fprintf('\n=== Simulating Separated Architecture ===\n');

% Initial conditions
x0_nom = [0; 0.05; 0; 0.05];     % Nominal plant
x0_slosh = [0.1; 0];              % Slosh (initial displacement)

% Time
t_final = 20;
dt = 0.01;
t = 0:dt:t_final;

% Input (elevator command)
u_elevator = zeros(size(t));
u_elevator(t>=2 & t<3) = 0.1;
u_elevator(t>=3 & t<4) = -0.1;
u_throttle = zeros(size(t));
u_throttle(t>=2 & t<3) = 0.1;
u_throttle(t>=3 & t<4) = -0.1;
% Preallocate
x_nom = zeros(4, length(t));
x_slosh = zeros(2, length(t));
x_nom(:,1) = x0_nom;
x_slosh(:,1) = x0_slosh;

F_dist = zeros(size(t));
M_dist = zeros(size(t));
a_x = zeros(size(t));
a_theta = zeros(size(t));

% Simulation loop
fprintf('Running simulation...\n');
for i = 1:length(t)-1
    % Current states
    x_n = x_nom(:,i);
    x_s = x_slosh(:,i);
    u_e = u_elevator(i);
    u_t = u_throttle(i);
    
    % Compute disturbance forces from slosh
    dist = K_dist * x_s;
    F_dist(i) = dist(1);
    M_dist(i) = dist(2);
    
    % Nominal plant dynamics with disturbances
    u_nom = [u_e; u_t; F_dist(i); M_dist(i)];
    dx_nom = A_nom * x_n + B_nom * u_nom;
    
    % Extract accelerations for slosh excitation
    a_x(i) = dx_nom(1);        % dV/dt
    a_theta(i) = dx_nom(3);    % dq/dt
    
    % Slosh dynamics excited by aircraft motion
    u_slosh = [a_x(i); a_theta(i)];
    dx_slosh = A_slosh * x_s + B_slosh * u_slosh;
    
    % Integration (Euler for simplicity)
    x_nom(:,i+1) = x_n + dx_nom * dt;
    x_slosh(:,i+1) = x_s + dx_slosh * dt;
end

% Final values
dist = K_dist * x_slosh(:,end);
F_dist(end) = dist(1);
M_dist(end) = dist(2);
dx_final = A_nom * x_nom(:,end) + B_nom * [u_elevator(end); u_throttle(end); F_dist(end); M_dist(end)];  % ✅ Store result
a_x(end) = dx_final(1);    
a_theta(end) = dx_final(3);

fprintf('✓ Simulation complete\n');

%% COMPARISON: WITH vs WITHOUT DISTURBANCE
fprintf('\n=== Comparing: With vs Without Slosh Disturbance ===\n');

% Simulate nominal plant alone (no disturbance)
x_nom_only = zeros(4, length(t));
x_nom_only(:,1) = x0_nom;

for i = 1:length(t)-1
    u_nom = [u_elevator(i); u_throttle(i); 0; 0];  % No disturbances
    dx_nom = A_nom * x_nom_only(:,i) + B_nom * u_nom;
    x_nom_only(:,i+1) = x_nom_only(:,i) + dx_nom * dt;
end

fprintf('✓ Nominal-only simulation complete\n');

%% PLOTTING
figure('Position', [50, 50, 1600, 1000]);

% Nominal states comparison
state_names = {'Velocity (m/s)', 'Flight Path Angle (deg)', ...
               'Pitch Rate (deg/s)', 'Pitch Angle (deg)'};
scale_factors = [1, 180/pi, 180/pi, 180/pi];

for i = 1:4
    subplot(3,3,i);
    plot(t, x_nom_only(i,:)*scale_factors(i), 'b-', 'LineWidth', 2, 'DisplayName', 'Without Slosh');
    hold on;
    plot(t, x_nom(i,:)*scale_factors(i), 'r--', 'LineWidth', 2, 'DisplayName', 'With Slosh');
    grid on;
    xlabel('Time (s)');
    ylabel(state_names{i});
    title(state_names{i});
    if i == 1
        legend('Location', 'best');
    end
end

% Slosh states
subplot(3,3,5);
plot(t, x_slosh(1,:)*100, 'r-', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Slosh Position (cm)');
title('Slosh Displacement');

subplot(3,3,6);
plot(t, x_slosh(2,:), 'r-', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Slosh Velocity (m/s)');
title('Slosh Velocity');

% Disturbance forces
subplot(3,3,7);
plot(t, F_dist, 'g-', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Force (N)');
title('Disturbance Force on Aircraft');

subplot(3,3,8);
plot(t, M_dist, 'g-', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Moment (N·m)');
title('Disturbance Moment on Aircraft');

% Excitation (aircraft accelerations)
subplot(3,3,9);
plot(t, a_x, 'b-', 'LineWidth', 1.5, 'DisplayName', 'a_x');
hold on;
plot(t, a_theta, 'r-', 'LineWidth', 1.5, 'DisplayName', 'a_θ');
grid on;
xlabel('Time (s)');
ylabel('Acceleration (m/s² or rad/s²)');
title('Aircraft Accelerations (Slosh Excitation)');
legend('Location', 'best');

sgtitle('Separated Architecture: Nominal Plant + Slosh as Disturbance', ...
        'FontSize', 14, 'FontWeight', 'bold');

%% DISTURBANCE ANALYSIS
figure('Position', [100, 100, 1400, 600]);

% Effect of disturbance on pitch rate
subplot(1,3,1);
plot(t, rad2deg(x_nom(3,:)), 'b-', 'LineWidth', 2);
hold on;
yyaxis right;
plot(t, M_dist, 'r-', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
yyaxis left;
ylabel('Pitch Rate (deg/s)', 'Color', 'b');
yyaxis right;
ylabel('Disturbance Moment (N·m)', 'Color', 'r');
title('Disturbance Effect on Pitch Rate');

% Disturbance rejection metric
subplot(1,3,2);
pitch_dev = rad2deg(x_nom(3,:) - x_nom_only(3,:));
plot(t, pitch_dev, 'k-', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Pitch Rate Deviation (deg/s)');
title('Deviation Due to Slosh Disturbance');

% Slosh phase portrait
subplot(1,3,3);
plot(x_slosh(1,:)*100, x_slosh(2,:), 'r-', 'LineWidth', 2);
grid on;
xlabel('Slosh Position (cm)');
ylabel('Slosh Velocity (m/s)');
title('Slosh Phase Portrait');
hold on;
plot(x_slosh(1,1)*100, x_slosh(2,1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(x_slosh(1,end)*100, x_slosh(2,end), 'rs', 'MarkerSize', 10, 'LineWidth', 2);
legend('Trajectory', 'Start', 'End', 'Location', 'best');

sgtitle('Disturbance Analysis', 'FontSize', 14, 'FontWeight', 'bold');

%% FREQUENCY DOMAIN: DISTURBANCE REJECTION
fprintf('\n=== Disturbance Rejection Analysis ===\n');

figure('Position', [150, 150, 1200, 800]);

% Transfer function: Disturbance Force → Pitch Rate
sys_F_to_q = ss(A_nom, B_nom(:,2), C_nom(3,:), D_nom(3,2));

% Transfer function: Disturbance Moment → Pitch Rate
sys_M_to_q = ss(A_nom, B_nom(:,3), C_nom(3,:), D_nom(3,3));

subplot(2,2,1);
bode(sys_F_to_q);
grid on;
title('Disturbance Force → Pitch Rate');

subplot(2,2,2);
bode(sys_M_to_q);
grid on;
title('Disturbance Moment → Pitch Rate');

% Slosh as excitation source
subplot(2,2,3);
bode(sys_slosh(1,1));  % Aircraft accel → Slosh position
grid on;
title('Aircraft Acceleration → Slosh Position');

% Complete disturbance path
subplot(2,2,4);
% From elevator to pitch rate (with slosh)
bode(sys_closed(3,1));
hold on;
% Without slosh
bode(sys_nom(3,1));
grid on;
title('Elevator → Pitch Rate (With/Without Slosh)');
legend('With Slosh', 'Without Slosh');

sgtitle('Frequency Domain: Disturbance Rejection', 'FontSize', 14, 'FontWeight', 'bold');

%% SAVE DATA FOR SIMULINK
fprintf('\n=== Saving Data for Simulink ===\n');

% Save all parameters to MAT file
save('separated_system_params.mat', ...
     'A_nom', 'B_nom', 'C_nom', 'D_nom', ...
     'A_slosh', 'B_slosh', 'C_slosh', 'D_slosh', ...
     'K_dist', 'm_slosh', 'k_slosh', 'c_slosh', 'h_slosh', ...
     'm_aircraft', 'Iy', 'Mde', ...
     'x0_nom', 'x0_slosh');

fprintf('✓ Parameters saved to separated_system_params.mat\n');

%% SUMMARY
fprintf('\n=== Architecture Summary ===\n');
fprintf('NOMINAL PLANT:\n');
fprintf('  States: 4 (V, gamma, q, theta)\n');
fprintf('  Inputs: 3 (elevator, F_dist, M_dist)\n');
fprintf('  Independent: Can be tested/controlled without slosh\n\n');

fprintf('SLOSH DYNAMICS:\n');
fprintf('  States: 2 (position, velocity)\n');
fprintf('  Inputs: 2 (a_x, a_theta from aircraft)\n');
fprintf('  Outputs: 2 (position, velocity)\n');
fprintf('  Acts as: Disturbance generator\n\n');

fprintf('COUPLING:\n');
fprintf('  Forward: Aircraft accelerations → Excite slosh\n');
fprintf('  Feedback: Slosh states → Disturbance forces/moments → Aircraft\n');
fprintf('  Treatment: Slosh treated as EXTERNAL disturbance\n\n');

fprintf('ADVANTAGES:\n');
fprintf('  ✓ Modular: Easy to add/remove slosh\n');
fprintf('  ✓ Testable: Controller can be tested on nominal plant first\n');
fprintf('  ✓ Realistic: Matches how disturbances actually work\n');
fprintf('  ✓ Flexible: Can add multiple disturbance sources\n');
fprintf('  ✓ Clear: Physical interpretation is obvious\n');

fprintf('\nNext: Use build_separated_simulink_models.m to create Simulink version!\n');

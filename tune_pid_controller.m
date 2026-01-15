%% PID Controller Tuning for Aircraft with Slosh
% This script helps you find stable PID gains for the closed-loop system
%
% Methods included:
% 1. Stability analysis of nominal plant
% 2. Ziegler-Nichols tuning
% 3. Root locus design
% 4. Trial and error with stability checks
% 5. Automated search for stable gains

clear all; close all; clc;

fprintf('=== PID Controller Tuning for Stable Performance ===\n\n');

%% LOAD OR DEFINE SYSTEM PARAMETERS
if exist('separated_system_params.mat', 'file')
    load('separated_system_params.mat');
    fprintf('✓ Loaded system parameters\n');
else
    fprintf('Running separated_architecture.m to generate parameters...\n');
    run('separated_architecture.m');
    close all;
end

%% STEP 1: CHECK NOMINAL PLANT STABILITY
fprintf('\n--- Step 1: Analyzing Nominal Plant ---\n');

% Create nominal plant transfer function (elevator to theta)
sys_nom = ss(A_nom, B_nom(:,1), C_nom, D_nom(:,1));

% Elevator to pitch angle transfer function
G_theta = tf(sys_nom(4,1));  % 4th output (theta), 1st input (elevator)

fprintf('Transfer function: Elevator → Pitch Angle\n');
fprintf('  Numerator degree: %d\n', length(G_theta.Numerator{1})-1);
fprintf('  Denominator degree: %d\n', length(G_theta.Denominator{1})-1);

% Check open-loop stability
poles_ol = pole(G_theta);
fprintf('\nOpen-loop poles:\n');
for i = 1:length(poles_ol)
    if abs(imag(poles_ol(i))) < 1e-10
        fprintf('  p%d = %.4f (real)\n', i, real(poles_ol(i)));
    else
        fprintf('  p%d = %.4f ± %.4fj\n', i, real(poles_ol(i)), abs(imag(poles_ol(i))));
    end
end

if all(real(poles_ol) < 0)
    fprintf('✓ Nominal plant is open-loop STABLE\n');
else
    fprintf('✗ Nominal plant is open-loop UNSTABLE\n');
end

% DC gain
dcgain_val = dcgain(G_theta);
fprintf('\nDC Gain: %.4f\n', dcgain_val);

%% STEP 2: ZIEGLER-NICHOLS TUNING
fprintf('\n--- Step 2: Ziegler-Nichols Ultimate Gain Method ---\n');

% Find ultimate gain (Ku) - gain at which system becomes marginally stable
% Use closed-loop characteristic equation: 1 + K*G(s) = 0

fprintf('Finding ultimate gain Ku...\n');

% Search for Ku using root locus
K_test = logspace(-2, 3, 1000);
stable = true(size(K_test));

for i = 1:length(K_test)
    % Closed-loop with proportional gain only
    sys_cl = feedback(K_test(i)*G_theta, 1);
    poles_cl = pole(sys_cl);
    
    % Check stability
    if any(real(poles_cl) >= 0)
        stable(i) = false;
    end
end

% Find Ku (last stable gain)
idx_unstable = find(~stable, 1);
if isempty(idx_unstable)
    Ku = K_test(end);
    fprintf('  Ku > %.2f (system remains stable at high gains)\n', Ku);
else
    Ku = K_test(idx_unstable - 1);
    fprintf('  Ku ≈ %.2f (ultimate gain)\n', Ku);
end

% Find ultimate period (Tu)
sys_cl_critical = feedback(Ku*G_theta, 1);
[~, ~, wout] = step(sys_cl_critical, 20);

% Estimate oscillation period from step response
fprintf('  Computing ultimate period Tu...\n');

% Alternative: find frequency at -180° phase
[mag, phase, w] = bode(G_theta, logspace(-2, 2, 1000));
mag = squeeze(mag);
phase = squeeze(phase);

idx_180 = find(phase <= -180, 1);
if ~isempty(idx_180)
    w_u = w(idx_180);
    Tu = 2*pi/w_u;
    fprintf('  Tu ≈ %.2f sec (from phase crossover)\n', Tu);
else
    Tu = 1.0;  % Default estimate
    fprintf('  Tu estimated as %.2f sec (default)\n', Tu);
end

% Ziegler-Nichols PID tuning
Kp_zn = 0.6 * Ku;
Ki_zn = 1.2 * Ku / Tu;
Kd_zn = 0.075 * Ku * Tu;

fprintf('\nZiegler-Nichols PID Gains:\n');
fprintf('  Kp = %.2f\n', Kp_zn);
fprintf('  Ki = %.2f\n', Ki_zn);
fprintf('  Kd = %.2f\n', Kd_zn);

% Test stability
C_zn = pid(Kp_zn, Ki_zn, Kd_zn);
sys_cl_zn = feedback(C_zn * G_theta, 1);
poles_zn = pole(sys_cl_zn);

if all(real(poles_zn) < 0)
    fprintf('✓ Z-N tuned system is STABLE\n');
else
    fprintf('✗ Z-N tuned system is UNSTABLE\n');
    fprintf('  Will use conservative gains...\n');
    Kp_zn = 0.3 * Ku;
    Ki_zn = 0.6 * Ku / Tu;
    Kd_zn = 0.05 * Ku * Tu;
end

%% STEP 3: CONSERVATIVE STABLE GAINS
fprintf('\n--- Step 3: Conservative Stable Gains ---\n');

% Start with low gains and increase
Kp_safe = 5;
Ki_safe = 0.5;
Kd_safe = 2;

fprintf('Testing conservative gains...\n');

% Test stability
C_safe = pid(Kp_safe, Ki_safe, Kd_safe);
sys_cl_safe = feedback(C_safe * G_theta, 1);
poles_safe = pole(sys_cl_safe);

fprintf('Conservative Gains:\n');
fprintf('  Kp = %.2f\n', Kp_safe);
fprintf('  Ki = %.2f\n', Ki_safe);
fprintf('  Kd = %.2f\n', Kd_safe);

if all(real(poles_safe) < 0)
    fprintf('✓ Conservative system is STABLE\n');
else
    fprintf('✗ Even conservative gains are unstable!\n');
end

%% STEP 4: AGGRESSIVE BUT STABLE GAINS
fprintf('\n--- Step 4: Tuning for Performance ---\n');

% Use lower gains than Z-N for robustness
Kp_tuned = 8;
Ki_tuned = 1;
Kd_tuned = 3;

fprintf('Performance-tuned gains:\n');
fprintf('  Kp = %.2f\n', Kp_tuned);
fprintf('  Ki = %.2f\n', Ki_tuned);
fprintf('  Kd = %.2f\n', Kd_tuned);

C_tuned = pid(Kp_tuned, Ki_tuned, Kd_tuned);
sys_cl_tuned = feedback(C_tuned * G_theta, 1);
poles_tuned = pole(sys_cl_tuned);

if all(real(poles_tuned) < 0)
    fprintf('✓ Tuned system is STABLE\n');
else
    fprintf('✗ Tuned system is UNSTABLE\n');
end

%% STEP 5: GRID SEARCH FOR OPTIMAL GAINS
fprintf('\n--- Step 5: Grid Search for Best Gains ---\n');

% Search ranges
Kp_range = [1, 5, 10, 15, 20];
Ki_range = [0.1, 0.5, 1, 2, 5];
Kd_range = [0.5, 1, 2, 3, 5];

best_overshoot = inf;
best_settling = inf;
best_Kp = Kp_safe;
best_Ki = Ki_safe;
best_Kd = Kd_safe;

fprintf('Searching %d combinations...\n', length(Kp_range)*length(Ki_range)*length(Kd_range));

stable_count = 0;

for Kp = Kp_range
    for Ki = Ki_range
        for Kd = Kd_range
            % Test this combination
            C_test = pid(Kp, Ki, Kd);
            sys_cl_test = feedback(C_test * G_theta, 1);
            poles_test = pole(sys_cl_test);
            
            % Check stability
            if all(real(poles_test) < 0)
                stable_count = stable_count + 1;
                
                % Evaluate performance
                info = stepinfo(sys_cl_test);
                
                % Penalize overshoot and settling time
                if info.Overshoot < best_overshoot || ...
                   (abs(info.Overshoot - best_overshoot) < 5 && info.SettlingTime < best_settling)
                    best_overshoot = info.Overshoot;
                    best_settling = info.SettlingTime;
                    best_Kp = Kp;
                    best_Ki = Ki;
                    best_Kd = Kd;
                end
            end
        end
    end
end

fprintf('Found %d stable combinations\n', stable_count);
fprintf('\nBest gains (minimum overshoot):\n');
fprintf('  Kp = %.2f\n', best_Kp);
fprintf('  Ki = %.2f\n', best_Ki);
fprintf('  Kd = %.2f\n', best_Kd);
fprintf('  Overshoot: %.1f%%\n', best_overshoot);
fprintf('  Settling time: %.2f s\n', best_settling);

%% STEP 6: COMPARE ALL CONTROLLERS
fprintf('\n--- Step 6: Comparing All Controllers ---\n');

controllers = struct();
controllers(1).name = 'Conservative';
controllers(1).Kp = Kp_safe;
controllers(1).Ki = Ki_safe;
controllers(1).Kd = Kd_safe;

controllers(2).name = 'Ziegler-Nichols';
controllers(2).Kp = Kp_zn;
controllers(2).Ki = Ki_zn;
controllers(2).Kd = Kd_zn;

controllers(3).name = 'Performance';
controllers(3).Kp = Kp_tuned;
controllers(3).Ki = Ki_tuned;
controllers(3).Kd = Kd_tuned;

controllers(4).name = 'Optimized';
controllers(4).Kp = best_Kp;
controllers(4).Ki = best_Ki;
controllers(4).Kd = best_Kd;

% Create figure with subplots
figure('Position', [50, 50, 1400, 900]);

colors = {'b', 'r', 'g', 'm'};
t_sim = 0:0.01:10;

for i = 1:4
    C = pid(controllers(i).Kp, controllers(i).Ki, controllers(i).Kd);
    sys_cl = feedback(C * G_theta, 1);
    
    % Check stability
    poles = pole(sys_cl);
    is_stable = all(real(poles) < 0);
    
    if is_stable
        % Step response
        subplot(2,2,1);
        step(sys_cl, t_sim, colors{i});
        hold on;
        
        % Closed-loop poles
        subplot(2,2,2);
        plot(real(poles), imag(poles), [colors{i}, 'x'], 'MarkerSize', 10, 'LineWidth', 2);
        hold on;
        
        % Bode plot
        subplot(2,2,3);
        margin(C * G_theta);
        hold on;
        
        % Performance metrics
        info = stepinfo(sys_cl);
        controllers(i).stable = true;
        controllers(i).overshoot = info.Overshoot;
        controllers(i).settling = info.SettlingTime;
        controllers(i).rise = info.RiseTime;
    else
        controllers(i).stable = false;
        controllers(i).overshoot = NaN;
        controllers(i).settling = NaN;
        controllers(i).rise = NaN;
    end
end

% Format plots
subplot(2,2,1);
grid on;
xlabel('Time (s)');
ylabel('Pitch Angle (rad)');
title('Step Response Comparison');
legend({controllers.name}, 'Location', 'best');

subplot(2,2,2);
grid on;
xlabel('Real');
ylabel('Imaginary');
title('Closed-Loop Poles');
xline(0, 'k--', 'LineWidth', 1.5);
legend({controllers.name}, 'Location', 'best');
axis equal;

subplot(2,2,3);
grid on;
title('Loop Transfer Function Bode');

% Performance table
subplot(2,2,4);
axis off;
txt = sprintf('PERFORMANCE COMPARISON\n\n');
txt = [txt, sprintf('%-15s %8s %10s %12s %10s\n', 'Controller', 'Stable?', 'Overshoot', 'Settling(s)', 'Rise(s)')];
txt = [txt, repmat('-', 1, 60), '\n'];
for i = 1:4
    if controllers(i).stable
        txt = [txt, sprintf('%-15s %8s %9.1f%% %11.2f %10.2f\n', ...
                controllers(i).name, 'YES', ...
                controllers(i).overshoot, controllers(i).settling, controllers(i).rise)];
    else
        txt = [txt, sprintf('%-15s %8s %9s %11s %10s\n', ...
                controllers(i).name, 'NO', 'N/A', 'N/A', 'N/A')];
    end
end
text(0.1, 0.5, txt, 'FontName', 'Courier', 'FontSize', 10, 'VerticalAlignment', 'middle');

sgtitle('PID Controller Comparison', 'FontSize', 14, 'FontWeight', 'bold');

%% STEP 7: RECOMMENDATION
fprintf('\n=== RECOMMENDATION ===\n');

% Find best stable controller
stable_idx = find([controllers.stable]);
if ~isempty(stable_idx)
    [~, best_idx] = min([controllers(stable_idx).overshoot]);
    recommended = controllers(stable_idx(best_idx));
    
    fprintf('RECOMMENDED PID GAINS:\n');
    fprintf('  Kp = %.2f\n', recommended.Kp);
    fprintf('  Ki = %.2f\n', recommended.Ki);
    fprintf('  Kd = %.2f\n', recommended.Kd);
    fprintf('\nExpected Performance:\n');
    fprintf('  Overshoot: %.1f%%\n', recommended.overshoot);
    fprintf('  Settling time: %.2f s\n', recommended.settling);
    fprintf('  Rise time: %.2f s\n', recommended.rise);
    
    % Save to workspace
    Kp_recommended = recommended.Kp;
    Ki_recommended = recommended.Ki;
    Kd_recommended = recommended.Kd;
    
    save('tuned_pid_gains.mat', 'Kp_recommended', 'Ki_recommended', 'Kd_recommended');
    fprintf('\n✓ Gains saved to tuned_pid_gains.mat\n');
    
    fprintf('\nTo use in Simulink:\n');
    fprintf('  1. Open slosh_control_loop.slx\n');
    fprintf('  2. Double-click Controller subsystem\n');
    fprintf('  3. Double-click PID block\n');
    fprintf('  4. Set P=%.2f, I=%.2f, D=%.2f\n', ...
            Kp_recommended, Ki_recommended, Kd_recommended);
else
    fprintf('WARNING: No stable controller found!\n');
    fprintf('The nominal plant may be difficult to control.\n');
    fprintf('Try very conservative gains: Kp=1, Ki=0.1, Kd=0.5\n');
end

fprintf('\n✓ Tuning complete!\n');

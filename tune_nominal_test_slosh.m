%% Automated LQR Tuning: Nominal System with Slosh Impact Analysis
% 
% METHODOLOGY:
% 1. Tune LQR on NOMINAL system (no slosh)
% 2. Find optimal Q and R for best nominal performance
% 3. Test the tuned controller WITH slosh disturbance
% 4. Show performance degradation due to slosh
% 5. Quantify robustness

clear all; close all; clc;

fprintf('=== LQR Tuning: Nominal System → Slosh Impact ===\n\n');

%% LOAD SYSTEM
if exist('separated_system_params.mat', 'file')
    load('separated_system_params.mat');
    fprintf('✓ Loaded system parameters\n');
else
    run('separated_architecture.m');
    close all;
end

%% SETUP MIMO SYSTEM
fprintf('\n--- Setting up MIMO system ---\n');

% B matrix for MIMO: [elevator, throttle]
% For tuning purposes, we'll create a proper throttle input
% Throttle affects velocity directly
XT = 2.0;  % Throttle thrust effectiveness (m/s²)
B_mimo = [B_nom(:,1), [XT/m_aircraft; 0; 0; 0]];

fprintf('System: 4 states [V, γ, q, θ]\n');
fprintf('Inputs: 2 controls [elevator, throttle]\n');
fprintf('Tuning on: NOMINAL SYSTEM (no slosh)\n');
fprintf('Testing on: NOMINAL + SLOSH DISTURBANCE\n');

%% DEFINE PERFORMANCE OBJECTIVES
fprintf('\n--- Performance Objectives ---\n');

performance_weights = struct();
performance_weights.velocity_tracking = 1.0;    % Track velocity
performance_weights.pitch_tracking = 2.0;       % Track pitch (most important)
performance_weights.overshoot = 1.5;            % Penalize overshoot
performance_weights.settling_time = 1.0;        % Fast settling
performance_weights.control_effort = 0.5;       % Reasonable control use

fprintf('Tuning priorities:\n');
fprintf('  Velocity tracking:   %.1f\n', performance_weights.velocity_tracking);
fprintf('  Pitch tracking:      %.1f  ← Most important\n', performance_weights.pitch_tracking);
fprintf('  Overshoot penalty:   %.1f\n', performance_weights.overshoot);
fprintf('  Settling time:       %.1f\n', performance_weights.settling_time);
fprintf('  Control effort:      %.1f\n', performance_weights.control_effort);

%% TUNING METHOD SELECTION
fprintf('\n--- Select Tuning Method ---\n');
fprintf('1. Quick Grid Search (~5 min, 729 tests)\n');
fprintf('2. Comprehensive Grid Search (~15 min, 10k tests)\n');
fprintf('3. Optimization-based (~5 min, gradient descent)\n');

method = input('Select method [1-3]: ');
if isempty(method) || method < 1 || method > 3
    method = 1;
    fprintf('Using default: Quick Grid Search\n');
end

%% RUN TUNING ON NOMINAL SYSTEM
fprintf('\n=== TUNING ON NOMINAL SYSTEM ===\n');
fprintf('(No slosh disturbance during tuning)\n\n');

if method == 1
    % Quick grid search
    Q_V_range = [0.1, 1, 10];
    Q_gamma_range = [0.1, 1, 10];
    Q_q_range = [1, 10, 100];
    Q_theta_range = [10, 100, 1000];
    R_elevator_range = [0.1, 1, 10];
    R_throttle_range = [0.01, 0.1, 1];
    
    [best_Q, best_R, best_K, best_score, all_results] = ...
        grid_search_nominal(A_nom, B_mimo, ...
                           Q_V_range, Q_gamma_range, Q_q_range, Q_theta_range, ...
                           R_elevator_range, R_throttle_range, performance_weights);

elseif method == 2
    % Comprehensive grid search
    Q_V_range = logspace(-1, 2, 5);
    Q_gamma_range = logspace(-1, 2, 5);
    Q_q_range = logspace(0, 3, 5);
    Q_theta_range = logspace(1, 4, 5);
    R_elevator_range = logspace(-1, 1, 4);
    R_throttle_range = logspace(-2, 0, 4);
    
    [best_Q, best_R, best_K, best_score, all_results] = ...
        grid_search_nominal(A_nom, B_mimo, ...
                           Q_V_range, Q_gamma_range, Q_q_range, Q_theta_range, ...
                           R_elevator_range, R_throttle_range, performance_weights);

else
    % Optimization-based
    x0 = [log10(1), log10(1), log10(10), log10(100), log10(1), log10(0.1)];
    lb = [-2, -2, -1,  0, -2, -3];
    ub = [ 3,  3,  4,  5,  2,  1];
    
    obj_fun = @(x) evaluate_nominal_performance(x, A_nom, B_mimo, performance_weights);
    
    options = optimoptions('fmincon', 'Display', 'iter', ...
                          'MaxFunctionEvaluations', 300);
    
    [x_opt, best_score] = fmincon(obj_fun, x0, [], [], [], [], lb, ub, [], options);
    
    best_Q = diag(10.^x_opt(1:4));
    best_R = diag(10.^x_opt(5:6));
    best_K = lqr(A_nom, B_mimo, best_Q, best_R);
    all_results = [];
end

%% DISPLAY OPTIMAL GAINS
fprintf('\n=== OPTIMAL LQR GAINS (Tuned on Nominal) ===\n\n');

fprintf('Optimal Q (State Weights):\n');
fprintf('  Q_V:     %8.4f  (Velocity)\n', best_Q(1,1));
fprintf('  Q_γ:     %8.4f  (Flight path angle)\n', best_Q(2,2));
fprintf('  Q_q:     %8.4f  (Pitch rate)\n', best_Q(3,3));
fprintf('  Q_θ:     %8.4f  (Pitch angle)\n', best_Q(4,4));

fprintf('\nOptimal R (Control Weights):\n');
fprintf('  R_elevator: %8.4f\n', best_R(1,1));
fprintf('  R_throttle: %8.4f\n', best_R(2,2));

fprintf('\nOptimal K (Feedback Gains):\n');
fprintf('           V        γ        q        θ\n');
fprintf('Elevator: [%7.4f, %7.4f, %7.4f, %7.4f]\n', best_K(1,:));
fprintf('Throttle: [%7.4f, %7.4f, %7.4f, %7.4f]\n', best_K(2,:));

fprintf('\nNominal Performance Score: %.4f\n', best_score);

%% TEST ON NOMINAL SYSTEM (BASELINE)
fprintf('\n=== TEST 1: Nominal System (No Slosh) ===\n');

[metrics_nominal, x_nominal, u_nominal, t_sim] = ...
    simulate_mimo_controller(A_nom, B_mimo, best_K, [], [], []);

fprintf('Baseline Performance (No Disturbance):\n');
fprintf('  Velocity error:    %.3f m/s\n', metrics_nominal.V_error);
fprintf('  Pitch error:       %.3f deg\n', rad2deg(metrics_nominal.theta_error));
fprintf('  Pitch overshoot:   %.1f%%\n', metrics_nominal.theta_overshoot);
fprintf('  Settling time:     %.2f s\n', metrics_nominal.settling_time);
fprintf('  Control effort:    %.2f\n', metrics_nominal.control_effort);
fprintf('  ✓ This is the ideal performance\n');

%% TEST ON SYSTEM WITH SLOSH (ROBUSTNESS)
fprintf('\n=== TEST 2: System WITH Slosh Disturbance ===\n');
fprintf('(Using SAME controller tuned on nominal)\n\n');

[metrics_slosh, x_slosh, u_slosh, ~, x_slosh_states, F_dist, M_dist] = ...
    simulate_mimo_controller(A_nom, B_mimo, best_K, K_dist, A_slosh, B_slosh);

fprintf('Performance with Slosh Disturbance:\n');
fprintf('  Velocity error:    %.3f m/s\n', metrics_slosh.V_error);
fprintf('  Pitch error:       %.3f deg\n', rad2deg(metrics_slosh.theta_error));
fprintf('  Pitch overshoot:   %.1f%%\n', metrics_slosh.theta_overshoot);
fprintf('  Settling time:     %.2f s\n', metrics_slosh.settling_time);
fprintf('  Control effort:    %.2f\n', metrics_slosh.control_effort);

%% COMPUTE DEGRADATION
fprintf('\n=== PERFORMANCE DEGRADATION DUE TO SLOSH ===\n');

degradation = struct();
degradation.V_error_increase = (metrics_slosh.V_error - metrics_nominal.V_error) / metrics_nominal.V_error * 100;
degradation.theta_error_increase = (metrics_slosh.theta_error - metrics_nominal.theta_error) / metrics_nominal.theta_error * 100;
degradation.overshoot_increase = metrics_slosh.theta_overshoot - metrics_nominal.theta_overshoot;
degradation.settling_increase = (metrics_slosh.settling_time - metrics_nominal.settling_time) / metrics_nominal.settling_time * 100;
degradation.effort_increase = (metrics_slosh.control_effort - metrics_nominal.control_effort) / metrics_nominal.control_effort * 100;

fprintf('Metric                  | Nominal  | With Slosh | Degradation\n');
fprintf('------------------------|----------|------------|-------------\n');
fprintf('Velocity error (m/s)    | %8.3f | %10.3f | %+7.1f%%\n', ...
        metrics_nominal.V_error, metrics_slosh.V_error, degradation.V_error_increase);
fprintf('Pitch error (deg)       | %8.3f | %10.3f | %+7.1f%%\n', ...
        rad2deg(metrics_nominal.theta_error), rad2deg(metrics_slosh.theta_error), degradation.theta_error_increase);
fprintf('Overshoot (%%)           | %8.1f | %10.1f | %+7.1f pp\n', ...
        metrics_nominal.theta_overshoot, metrics_slosh.theta_overshoot, degradation.overshoot_increase);
fprintf('Settling time (s)       | %8.2f | %10.2f | %+7.1f%%\n', ...
        metrics_nominal.settling_time, metrics_slosh.settling_time, degradation.settling_increase);
fprintf('Control effort          | %8.2f | %10.2f | %+7.1f%%\n', ...
        metrics_nominal.control_effort, metrics_slosh.control_effort, degradation.effort_increase);

fprintf('\nSlosh Impact Summary:\n');
if metrics_slosh.theta_error < 2*metrics_nominal.theta_error
    fprintf('  ✓ Controller shows GOOD robustness to slosh\n');
elseif metrics_slosh.theta_error < 3*metrics_nominal.theta_error
    fprintf('  ⚠ Controller shows MODERATE robustness to slosh\n');
else
    fprintf('  ✗ Controller shows POOR robustness to slosh\n');
end

max_slosh_displacement = max(abs(x_slosh_states(1,:)));
fprintf('  Maximum slosh displacement: %.2f cm\n', max_slosh_displacement*100);
fprintf('  Peak disturbance moment: %.2f N·m\n', max(abs(M_dist)));

%% PLOTTING
fprintf('\n--- Generating Comparison Plots ---\n');

figure('Position', [50, 50, 1600, 1000]);

% Velocity
subplot(3,4,1);
plot(t_sim, x_nominal(1,:), 'b-', 'LineWidth', 2);
hold on;
plot(t_sim, x_slosh(1,:), 'r--', 'LineWidth', 2);
yline(52, 'k:', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Velocity Response');
legend('Nominal', 'With Slosh', 'Reference', 'Location', 'best');

% Flight path angle
subplot(3,4,2);
plot(t_sim, rad2deg(x_nominal(2,:)), 'b-', 'LineWidth', 2);
hold on;
plot(t_sim, rad2deg(x_slosh(2,:)), 'r--', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('γ (deg)');
title('Flight Path Angle');

% Pitch rate
subplot(3,4,3);
plot(t_sim, rad2deg(x_nominal(3,:)), 'b-', 'LineWidth', 2);
hold on;
plot(t_sim, rad2deg(x_slosh(3,:)), 'r--', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('q (deg/s)');
title('Pitch Rate');

% Pitch angle
subplot(3,4,4);
plot(t_sim, rad2deg(x_nominal(4,:)), 'b-', 'LineWidth', 2);
hold on;
plot(t_sim, rad2deg(x_slosh(4,:)), 'r--', 'LineWidth', 2);
yline(rad2deg(0.1), 'k:', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('θ (deg)');
title('Pitch Angle');
legend('Nominal', 'With Slosh', 'Reference', 'Location', 'best');

% Elevator
subplot(3,4,5);
plot(t_sim, rad2deg(u_nominal(1,:)), 'b-', 'LineWidth', 2);
hold on;
plot(t_sim, rad2deg(u_slosh(1,:)), 'r--', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Elevator (deg)');
title('Elevator Command');
legend('Nominal', 'With Slosh', 'Location', 'best');

% Throttle
subplot(3,4,6);
plot(t_sim, u_nominal(2,:)*100, 'b-', 'LineWidth', 2);
hold on;
plot(t_sim, u_slosh(2,:)*100, 'r--', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Throttle (%)');
title('Throttle Command');

% Slosh position
subplot(3,4,7);
if ~isempty(x_slosh_states)
    plot(t_sim, x_slosh_states(1,:)*100, 'r-', 'LineWidth', 2);
end
grid on;
xlabel('Time (s)');
ylabel('Position (cm)');
title('Slosh Displacement');

% Slosh velocity
subplot(3,4,8);
if ~isempty(x_slosh_states)
    plot(t_sim, x_slosh_states(2,:)*100, 'r-', 'LineWidth', 2);
end
grid on;
xlabel('Time (s)');
ylabel('Velocity (cm/s)');
title('Slosh Velocity');

% Disturbance force
subplot(3,4,9);
if ~isempty(F_dist)
    plot(t_sim, F_dist, 'r-', 'LineWidth', 2);
end
grid on;
xlabel('Time (s)');
ylabel('Force (N)');
title('Disturbance Force');

% Disturbance moment
subplot(3,4,10);
if ~isempty(M_dist)
    plot(t_sim, M_dist, 'r-', 'LineWidth', 2);
end
grid on;
xlabel('Time (s)');
ylabel('Moment (N·m)');
title('Disturbance Moment');

% Error comparison
subplot(3,4,11);
errors_nominal = [metrics_nominal.V_error, rad2deg(metrics_nominal.theta_error), ...
                  metrics_nominal.theta_overshoot, metrics_nominal.settling_time];
errors_slosh = [metrics_slosh.V_error, rad2deg(metrics_slosh.theta_error), ...
                metrics_slosh.theta_overshoot, metrics_slosh.settling_time];
bar([errors_nominal; errors_slosh]');
set(gca, 'XTickLabel', {'V err', 'θ err', 'O.S.', 'Settle'});
legend('Nominal', 'With Slosh', 'Location', 'best');
title('Performance Comparison');
grid on;

% Degradation percentages
subplot(3,4,12);
degrad_values = [degradation.V_error_increase, degradation.theta_error_increase, ...
                 degradation.settling_increase, degradation.effort_increase];
bar(degrad_values);
set(gca, 'XTickLabel', {'V err', 'θ err', 'Settle', 'Effort'});
ylabel('% Increase');
title('Performance Degradation (%)');
grid on;
yline(0, 'k-', 'LineWidth', 1);

sgtitle('LQR Performance: Nominal vs With Slosh', 'FontSize', 14, 'FontWeight', 'bold');

%% SAVE RESULTS
fprintf('\n--- Saving Results ---\n');

save('lqr_nominal_tuning_slosh_impact.mat', ...
     'best_Q', 'best_R', 'best_K', ...
     'metrics_nominal', 'metrics_slosh', 'degradation', ...
     'x_nominal', 'x_slosh', 'u_nominal', 'u_slosh', ...
     'x_slosh_states', 'F_dist', 'M_dist', 't_sim');

fprintf('✓ Results saved to lqr_nominal_tuning_slosh_impact.mat\n');

fprintf('\n=== SUMMARY ===\n');
fprintf('Tuning approach: Optimize on NOMINAL system\n');
fprintf('Testing approach: Validate robustness WITH slosh\n');
fprintf('\nThis is the CORRECT methodology:\n');
fprintf('  1. Tune controller for best nominal performance\n');
fprintf('  2. Test robustness to disturbances\n');
fprintf('  3. Quantify performance degradation\n');
fprintf('  4. Accept if degradation is acceptable\n');

fprintf('\n✓ Script complete!\n');

%% HELPER FUNCTIONS

function [best_Q, best_R, best_K, best_score, results] = ...
    grid_search_nominal(A, B, Q_V_range, Q_gamma_range, Q_q_range, Q_theta_range, ...
                        R_elevator_range, R_throttle_range, perf_weights)
    
    fprintf('Running grid search on NOMINAL system...\n');
    
    results = [];
    best_score = inf;
    best_Q = [];
    best_R = [];
    best_K = [];
    
    counter = 0;
    total = length(Q_V_range) * length(Q_gamma_range) * length(Q_q_range) * ...
            length(Q_theta_range) * length(R_elevator_range) * length(R_throttle_range);
    
    for Q_V = Q_V_range
        for Q_gamma = Q_gamma_range
            for Q_q = Q_q_range
                for Q_theta = Q_theta_range
                    for R_elev = R_elevator_range
                        for R_throt = R_throttle_range
                            counter = counter + 1;
                            
                            if mod(counter, 100) == 0
                                fprintf('  Progress: %d/%d (%.1f%%)\n', counter, total, counter/total*100);
                            end
                            
                            Q = diag([Q_V, Q_gamma, Q_q, Q_theta]);
                            R = diag([R_elev, R_throt]);
                            
                            try
                                K = lqr(A, B, Q, R);
                                
                                % Check stability
                                A_cl = A - B*K;
                                eigs_cl = eig(A_cl);
                                is_stable = all(real(eigs_cl) < 0);
                                
                                if is_stable
                                    % Evaluate on NOMINAL system (no slosh)
                                    [metrics, ~, ~, ~] = simulate_mimo_controller(A, B, K, [], [], []);
                                    
                                    % Compute score
                                    score = perf_weights.velocity_tracking * metrics.V_error^2 + ...
                                            perf_weights.pitch_tracking * metrics.theta_error^2 + ...
                                            perf_weights.overshoot * (max(0, metrics.theta_overshoot - 10)/10)^2 + ...
                                            perf_weights.settling_time * (metrics.settling_time/10)^2 + ...
                                            perf_weights.control_effort * (metrics.control_effort/100)^2;
                                    
                                    result = struct('Q', Q, 'R', R, 'K', K, 'score', score, 'stable', true);
                                    results = [results; result];
                                    
                                    if score < best_score
                                        best_score = score;
                                        best_Q = Q;
                                        best_R = R;
                                        best_K = K;
                                    end
                                else
                                    result = struct('Q', Q, 'R', R, 'score', inf, 'stable', false);
                                    results = [results; result];
                                end
                            catch
                                result = struct('Q', Q, 'R', R, 'score', inf, 'stable', false);
                                results = [results; result];
                            end
                        end
                    end
                end
            end
        end
    end
    
    fprintf('✓ Grid search complete: %d/%d stable\n', sum([results.stable]), length(results));
end

function score = evaluate_nominal_performance(x, A, B, perf_weights)
    Q = diag(10.^x(1:4));
    R = diag(10.^x(5:6));
    
    try
        K = lqr(A, B, Q, R);
        A_cl = A - B*K;
        
        if any(real(eig(A_cl)) >= 0)
            score = 1e6;
            return;
        end
        
        [metrics, ~, ~, ~] = simulate_mimo_controller(A, B, K, [], [], []);
        
        score = perf_weights.velocity_tracking * metrics.V_error^2 + ...
                perf_weights.pitch_tracking * metrics.theta_error^2 + ...
                perf_weights.overshoot * (max(0, metrics.theta_overshoot - 10)/10)^2 + ...
                perf_weights.settling_time * (metrics.settling_time/10)^2 + ...
                perf_weights.control_effort * (metrics.control_effort/100)^2;
    catch
        score = 1e6;
    end
end

function [metrics, x, u, t, x_slosh, F_dist, M_dist] = ...
    simulate_mimo_controller(A, B, K, K_dist, A_slosh, B_slosh)
    
    % Reference
    x_ref = [52; 0; 0; 0.1];
    
    % Time
    dt = 0.01;
    t = 0:dt:15;
    N = length(t);
    
    % Initialize
    x = zeros(4, N);
    u = zeros(2, N);
    x(:,1) = [50; 0; 0; 0];
    
    % Check if slosh is included
    has_slosh = ~isempty(K_dist) && ~isempty(A_slosh) && ~isempty(B_slosh);
    
    if has_slosh
        x_slosh = zeros(2, N);
        x_slosh(:,1) = [0.1; 0];  % 10 cm initial displacement
        F_dist = zeros(N, 1);
        M_dist = zeros(N, 1);
    else
        x_slosh = [];
        F_dist = [];
        M_dist = [];
    end
    
    % Simulate
    for i = 1:N-1
        % Control law
        u(:,i) = -K * (x(:,i) - x_ref);
        u(1,i) = max(min(u(1,i), 0.5), -0.5);
        u(2,i) = max(min(u(2,i), 1.0), 0);
        
        % Plant dynamics
        if has_slosh
            % Compute disturbances
            dist = K_dist * x_slosh(:,i);
            F_dist(i) = dist(1);
            M_dist(i) = dist(2);
            
            % Add disturbances to dynamics
            dx = A * x(:,i) + B * u(:,i) + [0; 0; M_dist(i)/15000; 0];
            
            % Slosh dynamics
            dx_slosh = A_slosh * x_slosh(:,i) + B_slosh * [dx(1); dx(3)];
            x_slosh(:,i+1) = x_slosh(:,i) + dx_slosh * dt;
        else
            dx = A * x(:,i) + B * u(:,i);
        end
        
        x(:,i+1) = x(:,i) + dx * dt;
    end
    u(:,end) = u(:,end-1);
    if has_slosh
        F_dist(end) = F_dist(end-1);
        M_dist(end) = M_dist(end-1);
    end
    
    % Compute metrics
    metrics = struct();
    metrics.V_error = abs(x(1,end) - x_ref(1));
    metrics.theta_error = abs(x(4,end) - x_ref(4));
    
    theta_max = max(x(4,:));
    metrics.theta_overshoot = max(0, (theta_max - x_ref(4))/x_ref(4) * 100);
    
    theta_settled = abs(x(4,:) - x_ref(4)) < 0.02*abs(x_ref(4));
    idx_settled = find(theta_settled, 1);
    if ~isempty(idx_settled)
        metrics.settling_time = t(idx_settled);
    else
        metrics.settling_time = t(end);
    end
    
    metrics.control_effort = sum(u(1,:).^2 + u(2,:).^2) * dt;
end

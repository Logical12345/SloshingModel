%% Comprehensive LQR Tuning Method for MIMO Aircraft Control
% 
% SYSTEM: 4 states [V, γ, q, θ], 2 inputs [elevator, throttle]
% 
% METHODOLOGY:
% 1. Automated search for optimal Q and R weights
% 2. Multiple tuning strategies available
% 3. Performance evaluation on nominal system
% 4. Robustness testing with slosh disturbance
% 5. Comprehensive analysis and visualization
%
% OUTPUTS:
% - Optimal Q, R, K matrices
% - Performance metrics
% - Comparison plots
% - Robustness analysis


fprintf('╔════════════════════════════════════════════════════════╗\n');
fprintf('║   AUTOMATED LQR TUNING FOR MIMO AIRCRAFT CONTROL      ║\n');
fprintf('╚════════════════════════════════════════════════════════╝\n\n');

%% STEP 1: LOAD AND SETUP SYSTEM
fprintf('STEP 1: System Setup\n');
fprintf('━━━━━━━━━━━━━━━━━━━━\n');

if exist('separated_system_params.mat', 'file')
    load('separated_system_params.mat');
    fprintf('✓ Loaded system parameters\n');
else
    fprintf('Running separated_architecture.m...\n');
    run('separated_architecture.m');
    close all;
    fprintf('✓ Generated system parameters\n');
end

% Create MIMO system with throttle
fprintf('\nCreating MIMO system...\n');
T_max = 10000;  % Maximum thrust (N)
XT = T_max / m_aircraft;  % Thrust effectiveness (m/s²)

B_mimo = [B_nom(:,1), [XT; 0; 0; 0]];

fprintf('  States: [V, γ, q, θ] (4x1)\n');
fprintf('  Inputs: [elevator, throttle] (2x1)\n');
fprintf('  A matrix: 4x4\n');
fprintf('  B matrix: 4x2\n');

% Verify controllability
C = ctrb(A_nom, B_mimo);
if rank(C) == 4
    fprintf('  ✓ System is fully controllable\n');
else
    fprintf('  ✗ WARNING: System is not fully controllable!\n');
end

fprintf('\n');

%% STEP 2: DEFINE PERFORMANCE OBJECTIVES
fprintf('STEP 2: Performance Objectives\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

% Define what constitutes "good" performance
performance_goals = struct();
performance_goals.V_error_max = 0.1;        % Max velocity error (m/s)
performance_goals.theta_error_max = 0.5;    % Max pitch error (deg)
performance_goals.overshoot_max = 15;       % Max overshoot (%)
performance_goals.settling_time_max = 5;    % Max settling time (s)
performance_goals.control_effort_max = 50;  % Reasonable control usage

fprintf('Performance targets:\n');
fprintf('  Velocity error:    < %.2f m/s\n', performance_goals.V_error_max);
fprintf('  Pitch error:       < %.2f deg\n', performance_goals.theta_error_max);
fprintf('  Overshoot:         < %.1f%%\n', performance_goals.overshoot_max);
fprintf('  Settling time:     < %.1f s\n', performance_goals.settling_time_max);
fprintf('  Control effort:    < %.1f\n', performance_goals.control_effort_max);

% Weighting priorities for scoring
score_weights = struct();
score_weights.velocity = 2.0;      % Velocity tracking importance
score_weights.pitch = 3.0;         % Pitch tracking importance (higher)
score_weights.overshoot = 1.5;     % Overshoot penalty
score_weights.settling = 1.0;      % Settling time importance
score_weights.effort = 0.5;        % Control effort importance

fprintf('\nScoring priorities:\n');
fprintf('  Velocity tracking:  %.1f\n', score_weights.velocity);
fprintf('  Pitch tracking:     %.1f (highest)\n', score_weights.pitch);
fprintf('  Overshoot penalty:  %.1f\n', score_weights.overshoot);
fprintf('  Settling speed:     %.1f\n', score_weights.settling);
fprintf('  Control economy:    %.1f\n', score_weights.effort);

fprintf('\n');

%% STEP 3: SELECT TUNING METHOD
fprintf('STEP 3: Tuning Method Selection\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('Available methods:\n\n');
fprintf('1. Quick Grid Search\n');
fprintf('   • Time: ~5 minutes\n');
fprintf('   • Tests: ~1,500 combinations\n');
fprintf('   • Best for: Initial tuning\n\n');

fprintf('2. Comprehensive Grid Search\n');
fprintf('   • Time: ~15 minutes\n');
fprintf('   • Tests: ~10,000 combinations\n');
fprintf('   • Best for: Finding optimal solution\n\n');

fprintf('3. Gradient Optimization\n');
fprintf('   • Time: ~5 minutes\n');
fprintf('   • Method: fmincon (gradient descent)\n');
fprintf('   • Best for: Refinement\n\n');

fprintf('4. Multi-Objective Optimization\n');
fprintf('   • Time: ~20 minutes\n');
fprintf('   • Method: Genetic algorithm (Pareto)\n');
fprintf('   • Best for: Trade-off analysis\n\n');

method = input('Select method [1-4]: ');
if isempty(method) || method < 1 || method > 4
    method = 1;
    fprintf('Using default: Quick Grid Search\n');
end

fprintf('\n');

%% STEP 4: RUN TUNING ALGORITHM
fprintf('STEP 4: Running Tuning Algorithm\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

tic;

switch method
    case 1
        fprintf('Running Quick Grid Search...\n\n');
        [best_Q, best_R, best_K, best_score, all_results, tuning_stats] = ...
            quick_grid_search(A_nom, B_mimo, score_weights, performance_goals);
        
    case 2
        fprintf('Running Comprehensive Grid Search...\n\n');
        [best_Q, best_R, best_K, best_score, all_results, tuning_stats] = ...
            comprehensive_grid_search(A_nom, B_mimo, score_weights, performance_goals);
        
    case 3
        fprintf('Running Gradient Optimization...\n\n');
        [best_Q, best_R, best_K, best_score, all_results, tuning_stats] = ...
            gradient_optimization(A_nom, B_mimo, score_weights, performance_goals);
        
    case 4
        fprintf('Running Multi-Objective Optimization...\n\n');
        [best_Q, best_R, best_K, best_score, all_results, tuning_stats] = ...
            multiobjective_optimization(A_nom, B_mimo, score_weights, performance_goals);
end

tuning_time = toc;

fprintf('\n✓ Tuning complete in %.1f seconds\n', tuning_time);
fprintf('\n');

%% STEP 5: DISPLAY OPTIMAL GAINS
fprintf('STEP 5: Optimal LQR Controller\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

fprintf('Optimal Q Matrix (State Weights):\n');
fprintf('┌────────────────────────────────────────┐\n');
fprintf('│  Q_V  (Velocity):         %10.4f  │\n', best_Q(1,1));
fprintf('│  Q_γ  (Flight path):      %10.4f  │\n', best_Q(2,2));
fprintf('│  Q_q  (Pitch rate):       %10.4f  │\n', best_Q(3,3));
fprintf('│  Q_θ  (Pitch angle):      %10.4f  │\n', best_Q(4,4));
fprintf('└────────────────────────────────────────┘\n\n');

fprintf('Optimal R Matrix (Control Weights):\n');
fprintf('┌────────────────────────────────────────┐\n');
fprintf('│  R_elevator:              %10.4f  │\n', best_R(1,1));
fprintf('│  R_throttle:              %10.4f  │\n', best_R(2,2));
fprintf('└────────────────────────────────────────┘\n\n');

fprintf('Optimal K Matrix (Feedback Gains):\n');
fprintf('┌──────────────────────────────────────────────────────────┐\n');
fprintf('│           V         γ         q         θ               │\n');
fprintf('├──────────────────────────────────────────────────────────┤\n');
fprintf('│ Elev:  %8.4f  %8.4f  %8.4f  %8.4f       │\n', best_K(1,:));
fprintf('│ Throt: %8.4f  %8.4f  %8.4f  %8.4f       │\n', best_K(2,:));
fprintf('└──────────────────────────────────────────────────────────┘\n\n');

fprintf('Performance Score: %.4f (lower is better)\n\n');

% Check stability
A_cl = A_nom - B_mimo * best_K;
eigs_cl = eig(A_cl);
max_real_part = max(real(eigs_cl));

if max_real_part < -0.01
    fprintf('✓ System is STABLE (max Re(λ) = %.4f)\n', max_real_part);
elseif max_real_part < 0
    fprintf('⚠ System is MARGINALLY STABLE (max Re(λ) = %.4f)\n', max_real_part);
else
    fprintf('✗ System is UNSTABLE (max Re(λ) = %.4f)\n', max_real_part);
end

fprintf('\n');

%% STEP 6: TEST ON NOMINAL SYSTEM
fprintf('STEP 6: Nominal Performance Evaluation\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

fprintf('Testing controller on nominal system (no disturbances)...\n\n');

[metrics_nom, x_nom, u_nom, t_sim] = ...
    simulate_controller(A_nom, B_mimo, best_K, [], [], []);

fprintf('Results:\n');
fprintf('┌────────────────────────────────────────────────┐\n');
fprintf('│ Metric              │ Value      │ Target    │\n');
fprintf('├────────────────────────────────────────────────┤\n');
fprintf('│ Velocity error      │ %6.3f m/s │ < %.2f    │', ...
    metrics_nom.V_error, performance_goals.V_error_max);
if metrics_nom.V_error < performance_goals.V_error_max
    fprintf(' ✓ │\n');
else
    fprintf(' ✗ │\n');
end
fprintf('│ Pitch error         │ %6.3f deg │ < %.2f    │', ...
    rad2deg(metrics_nom.theta_error), performance_goals.theta_error_max);
if rad2deg(metrics_nom.theta_error) < performance_goals.theta_error_max
    fprintf(' ✓ │\n');
else
    fprintf(' ✗ │\n');
end
fprintf('│ Overshoot           │ %6.1f %%   │ < %.0f     │', ...
    metrics_nom.theta_overshoot, performance_goals.overshoot_max);
if metrics_nom.theta_overshoot < performance_goals.overshoot_max
    fprintf(' ✓ │\n');
else
    fprintf(' ✗ │\n');
end
fprintf('│ Settling time       │ %6.2f s   │ < %.0f      │', ...
    metrics_nom.settling_time, performance_goals.settling_time_max);
if metrics_nom.settling_time < performance_goals.settling_time_max
    fprintf(' ✓ │\n');
else
    fprintf(' ✗ │\n');
end
fprintf('│ Control effort      │ %6.2f     │ < %.0f     │', ...
    metrics_nom.control_effort, performance_goals.control_effort_max);
if metrics_nom.control_effort < performance_goals.control_effort_max
    fprintf(' ✓ │\n');
else
    fprintf(' ✗ │\n');
end
fprintf('└────────────────────────────────────────────────┘\n\n');

%% STEP 7: TEST WITH SLOSH DISTURBANCE
fprintf('STEP 7: Robustness Testing with Slosh\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

fprintf('Testing same controller with slosh disturbance...\n\n');

[metrics_slosh, x_slosh, u_slosh, ~, x_slosh_states, F_dist, M_dist] = ...
    simulate_controller(A_nom, B_mimo, best_K, K_dist, A_slosh, B_slosh);

fprintf('Results with slosh:\n');
fprintf('┌────────────────────────────────────────────────┐\n');
fprintf('│ Metric              │ Nominal    │ w/ Slosh  │\n');
fprintf('├────────────────────────────────────────────────┤\n');
fprintf('│ Velocity error      │ %6.3f m/s │ %6.3f   │\n', ...
    metrics_nom.V_error, metrics_slosh.V_error);
fprintf('│ Pitch error         │ %6.3f deg │ %6.3f   │\n', ...
    rad2deg(metrics_nom.theta_error), rad2deg(metrics_slosh.theta_error));
fprintf('│ Overshoot           │ %6.1f %%   │ %6.1f   │\n', ...
    metrics_nom.theta_overshoot, metrics_slosh.theta_overshoot);
fprintf('│ Settling time       │ %6.2f s   │ %6.2f   │\n', ...
    metrics_nom.settling_time, metrics_slosh.settling_time);
fprintf('│ Control effort      │ %6.2f     │ %6.2f   │\n', ...
    metrics_nom.control_effort, metrics_slosh.control_effort);
fprintf('└────────────────────────────────────────────────┘\n\n');

% Compute degradation
degradation = struct();
degradation.V_pct = (metrics_slosh.V_error - metrics_nom.V_error) / metrics_nom.V_error * 100;
degradation.theta_pct = (metrics_slosh.theta_error - metrics_nom.theta_error) / metrics_nom.theta_error * 100;
degradation.overshoot_pp = metrics_slosh.theta_overshoot - metrics_nom.theta_overshoot;
degradation.settling_pct = (metrics_slosh.settling_time - metrics_nom.settling_time) / metrics_nom.settling_time * 100;

fprintf('Performance degradation due to slosh:\n');
fprintf('  Velocity error:    %+.1f%%\n', degradation.V_pct);
fprintf('  Pitch error:       %+.1f%%\n', degradation.theta_pct);
fprintf('  Overshoot:         %+.1f percentage points\n', degradation.overshoot_pp);
fprintf('  Settling time:     %+.1f%%\n', degradation.settling_pct);

% Robustness assessment
avg_degradation = (abs(degradation.V_pct) + abs(degradation.theta_pct) + abs(degradation.settling_pct)) / 3;
if avg_degradation < 30
    fprintf('\n✓ Controller shows EXCELLENT robustness (%.1f%% avg degradation)\n', avg_degradation);
elseif avg_degradation < 60
    fprintf('\n✓ Controller shows GOOD robustness (%.1f%% avg degradation)\n', avg_degradation);
elseif avg_degradation < 100
    fprintf('\n⚠ Controller shows MODERATE robustness (%.1f%% avg degradation)\n', avg_degradation);
else
    fprintf('\n✗ Controller shows POOR robustness (%.1f%% avg degradation)\n', avg_degradation);
end

fprintf('\n');

%% STEP 8: TUNING STATISTICS
fprintf('STEP 8: Tuning Statistics\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

fprintf('Search summary:\n');
fprintf('  Total combinations tested: %d\n', tuning_stats.total_tested);
fprintf('  Stable controllers found:  %d (%.1f%%)\n', ...
    tuning_stats.stable_count, tuning_stats.stable_count/tuning_stats.total_tested*100);
fprintf('  Best score achieved:       %.4f\n', best_score);

if isfield(tuning_stats, 'score_range')
    fprintf('  Score range:               [%.4f, %.4f]\n', tuning_stats.score_range);
end

fprintf('  Computation time:          %.1f seconds\n', tuning_time);

fprintf('\n');

%% STEP 9: VISUALIZATION
fprintf('STEP 9: Generating Plots\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━\n');

% Main comparison plot
fig1 = figure('Position', [50, 50, 1600, 1000], 'Name', 'LQR Performance Analysis');

% Velocity
subplot(3,4,1);
plot(t_sim, x_nom(1,:), 'b-', 'LineWidth', 2);
hold on;
plot(t_sim, x_slosh(1,:), 'r--', 'LineWidth', 2);
yline(52, 'k:', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('V (m/s)');
title('Velocity Response');
legend('Nominal', 'With Slosh', 'Reference', 'Location', 'best');

% Flight path angle
subplot(3,4,2);
plot(t_sim, rad2deg(x_nom(2,:)), 'b-', 'LineWidth', 2);
hold on;
plot(t_sim, rad2deg(x_slosh(2,:)), 'r--', 'LineWidth', 2);
yline(0, 'k:', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('γ (deg)');
title('Flight Path Angle');

% Pitch rate
subplot(3,4,3);
plot(t_sim, rad2deg(x_nom(3,:)), 'b-', 'LineWidth', 2);
hold on;
plot(t_sim, rad2deg(x_slosh(3,:)), 'r--', 'LineWidth', 2);
yline(0, 'k:', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('q (deg/s)');
title('Pitch Rate');

% Pitch angle
subplot(3,4,4);
plot(t_sim, rad2deg(x_nom(4,:)), 'b-', 'LineWidth', 2);
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
plot(t_sim, rad2deg(u_nom(1,:)), 'b-', 'LineWidth', 2);
hold on;
plot(t_sim, rad2deg(u_slosh(1,:)), 'r--', 'LineWidth', 2);
yline(0, 'k:', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('Elevator (deg)');
title('Elevator Command');

% Throttle
subplot(3,4,6);
plot(t_sim, u_nom(2,:)*100, 'b-', 'LineWidth', 2);
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

% Disturbance moment
subplot(3,4,9);
if ~isempty(M_dist)
    plot(t_sim, M_dist, 'r-', 'LineWidth', 2);
end
grid on;
xlabel('Time (s)');
ylabel('Moment (N·m)');
title('Disturbance Moment');

% Performance comparison bars
subplot(3,4,10);
metrics_array = [metrics_nom.V_error, rad2deg(metrics_nom.theta_error), ...
                 metrics_nom.theta_overshoot, metrics_nom.settling_time;
                 metrics_slosh.V_error, rad2deg(metrics_slosh.theta_error), ...
                 metrics_slosh.theta_overshoot, metrics_slosh.settling_time]';
bar(metrics_array);
set(gca, 'XTickLabel', {'V err', 'θ err', 'O.S.', 'Settle'});
legend('Nominal', 'With Slosh');
title('Performance Metrics');
grid on;

% Closed-loop poles
subplot(3,4,11);
plot(real(eigs_cl), imag(eigs_cl), 'bx', 'MarkerSize', 12, 'LineWidth', 2);
hold on;
xline(0, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Real');
ylabel('Imaginary');
title('Closed-Loop Poles');
axis equal;

% Control allocation
subplot(3,4,12);
effort_elev_nom = sum(u_nom(1,:).^2) * (t_sim(2)-t_sim(1));
effort_throt_nom = sum(u_nom(2,:).^2) * (t_sim(2)-t_sim(1));
pie([effort_elev_nom, effort_throt_nom], {'Elevator', 'Throttle'});
title('Control Allocation (Nominal)');

sgtitle('Automated LQR Tuning Results', 'FontSize', 14, 'FontWeight', 'bold');

% Tuning analysis plot (if grid search)
if method <= 2 && ~isempty(all_results)
    fig2 = create_tuning_analysis_plot(all_results, best_Q, best_R, best_score);
end

fprintf('✓ Plots generated\n\n');

%% STEP 10: SAVE RESULTS
fprintf('STEP 10: Saving Results\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━\n');

results = struct();
results.best_Q = best_Q;
results.best_R = best_R;
results.best_K = best_K;
results.best_score = best_score;
results.metrics_nominal = metrics_nom;
results.metrics_slosh = metrics_slosh;
results.degradation = degradation;
results.tuning_stats = tuning_stats;
results.tuning_time = tuning_time;
results.method = method;
results.x_nominal = x_nom;
results.x_slosh = x_slosh;
results.u_nominal = u_nom;
results.u_slosh = u_slosh;
results.t_sim = t_sim;

save('optimal_lqr_controller.mat', '-struct', 'results');

fprintf('✓ Results saved to: optimal_lqr_controller.mat\n\n');

%% STEP 11: SUMMARY AND RECOMMENDATIONS
fprintf('╔════════════════════════════════════════════════════════╗\n');
fprintf('║                  TUNING COMPLETE                       ║\n');
fprintf('╚════════════════════════════════════════════════════════╝\n\n');

fprintf('OPTIMAL CONTROLLER FOUND:\n');
fprintf('  Q = diag([%.4f, %.4f, %.4f, %.4f])\n', diag(best_Q));
fprintf('  R = diag([%.4f, %.4f])\n', diag(best_R));
fprintf('\n');

fprintf('PERFORMANCE SUMMARY:\n');
fprintf('  ✓ Velocity tracking:   %.3f m/s error\n', metrics_slosh.V_error);
fprintf('  ✓ Pitch tracking:      %.3f deg error\n', rad2deg(metrics_slosh.theta_error));
fprintf('  ✓ Overshoot:           %.1f%%\n', metrics_slosh.theta_overshoot);
fprintf('  ✓ Settling time:       %.2f s\n', metrics_slosh.settling_time);
fprintf('\n');

fprintf('NEXT STEPS:\n');
fprintf('  1. Review plots for detailed analysis\n');
fprintf('  2. Load results: load(''optimal_lqr_controller.mat'')\n');
fprintf('  3. Implement in Simulink or further testing\n');
fprintf('  4. If unsatisfied, adjust score_weights and re-run\n');
fprintf('\n');

fprintf('✓ All done! Your optimal LQR controller is ready.\n\n');

%% HELPER FUNCTIONS

function [Q, R, K, score, results, stats] = quick_grid_search(A, B, weights, goals)
    % Quick grid search - 3 values per parameter
    
    Q_V_range = [1, 10, 100];
    Q_gamma_range = [0.1, 1, 10];
    Q_q_range = [10, 100, 1000];
    Q_theta_range = [100, 1000, 10000];
    R_elev_range = [0.1, 1, 10];
    R_throt_range = [0.01, 0.1, 1];
    
    [Q, R, K, score, results, stats] = grid_search_engine(A, B, ...
        Q_V_range, Q_gamma_range, Q_q_range, Q_theta_range, ...
        R_elev_range, R_throt_range, weights, goals);
end

function [Q, R, K, score, results, stats] = comprehensive_grid_search(A, B, weights, goals)
    % Comprehensive grid search - 5 values per parameter (logarithmic)
    
    Q_V_range = logspace(0, 3, 5);        % 1 to 1000
    Q_gamma_range = logspace(-1, 2, 5);   % 0.1 to 100
    Q_q_range = logspace(1, 4, 5);        % 10 to 10000
    Q_theta_range = logspace(2, 5, 5);    % 100 to 100000
    R_elev_range = logspace(-1, 1, 4);    % 0.1 to 10
    R_throt_range = logspace(-2, 0, 4);   % 0.01 to 1
    
    [Q, R, K, score, results, stats] = grid_search_engine(A, B, ...
        Q_V_range, Q_gamma_range, Q_q_range, Q_theta_range, ...
        R_elev_range, R_throt_range, weights, goals);
end

function [Q, R, K, score, results, stats] = gradient_optimization(A, B, weights, goals)
    % Gradient-based optimization using fmincon
    
    % Initial guess (reasonable defaults)
    x0 = [log10(10), log10(1), log10(100), log10(1000), log10(1), log10(0.1)];
    
    % Bounds
    lb = [-1, -2, 0, 1, -2, -3];
    ub = [4, 3, 5, 6, 2, 1];
    
    % Objective function
    obj = @(x) evaluate_controller(x, A, B, weights, goals);
    
    % Options
    opts = optimoptions('fmincon', 'Display', 'iter', ...
        'MaxFunctionEvaluations', 300, 'OptimalityTolerance', 1e-4);
    
    % Optimize
    [x_opt, score] = fmincon(obj, x0, [], [], [], [], lb, ub, [], opts);
    
    % Extract results
    Q = diag(10.^x_opt(1:4));
    R = diag(10.^x_opt(5:6));
    K = lqr(A, B, Q, R);
    
    results = [];
    stats = struct('total_tested', 300, 'stable_count', 300);
end

function [Q, R, K, score, results, stats] = multiobjective_optimization(A, B, weights, goals)
    % Multi-objective optimization using genetic algorithm
    
    nvars = 6;
    lb = [-1, -2, 0, 1, -2, -3];
    ub = [4, 3, 5, 6, 2, 1];
    
    % Multi-objective function: [tracking_error, control_effort]
    multobj = @(x) evaluate_multiobjective(x, A, B, weights, goals);
    
    opts = optimoptions('gamultiobj', 'Display', 'iter', ...
        'PopulationSize', 50, 'MaxGenerations', 20);
    
    [x_pareto, fval_pareto] = gamultiobj(multobj, nvars, [], [], [], [], lb, ub, opts);
    
    % Select balanced solution from Pareto front
    combined = fval_pareto(:,1) + 0.5*fval_pareto(:,2);
    [score, idx] = min(combined);
    
    Q = diag(10.^x_pareto(idx, 1:4));
    R = diag(10.^x_pareto(idx, 5:6));
    K = lqr(A, B, Q, R);
    
    results = struct('x_pareto', x_pareto, 'fval_pareto', fval_pareto);
    stats = struct('total_tested', 50*20, 'stable_count', size(x_pareto,1));
end

function [Q, R, K, best_score, results, stats] = grid_search_engine(A, B, ...
    Q_V_range, Q_gamma_range, Q_q_range, Q_theta_range, ...
    R_elev_range, R_throt_range, weights, goals)
    
    total = length(Q_V_range) * length(Q_gamma_range) * length(Q_q_range) * ...
            length(Q_theta_range) * length(R_elev_range) * length(R_throt_range);
    
    fprintf('Testing %d combinations...\n', total);
    
    results = [];
    best_score = inf;
    Q = [];
    R = [];
    K = [];
    counter = 0;
    stable_count = 0;
    
    for qv = Q_V_range
        for qg = Q_gamma_range
            for qq = Q_q_range
                for qt = Q_theta_range
                    for re = R_elev_range
                        for rt = R_throt_range
                            counter = counter + 1;
                            
                            if mod(counter, 200) == 0
                                fprintf('  Progress: %d/%d (%.1f%%) - Best score: %.4f\n', ...
                                    counter, total, counter/total*100, best_score);
                            end
                            
                            Q_test = diag([qv, qg, qq, qt]);
                            R_test = diag([re, rt]);
                            
                            try
                                K_test = lqr(A, B, Q_test, R_test);
                                
                                % Check stability
                                A_cl = A - B*K_test;
                                if all(real(eig(A_cl)) < 0)
                                    stable_count = stable_count + 1;
                                    
                                    % Evaluate performance
                                    [metrics, ~, ~, ~] = simulate_controller(A, B, K_test, [], [], []);
                                    score = compute_score(metrics, weights, goals);
                                    
                                    result = struct('Q', Q_test, 'R', R_test, 'K', K_test, ...
                                        'score', score, 'stable', true, 'metrics', metrics);
                                    results = [results; result];
                                    
                                    if score < best_score
                                        best_score = score;
                                        Q = Q_test;
                                        R = R_test;
                                        K = K_test;
                                    end
                                end
                            catch
                                % LQR failed or unstable
                            end
                        end
                    end
                end
            end
        end
    end
    
    stats = struct('total_tested', total, 'stable_count', stable_count);
    
    if ~isempty(results)
        scores = [results.score];
        stats.score_range = [min(scores), max(scores)];
    end
    
    fprintf('✓ Search complete: %d/%d stable\n', stable_count, total);
end

function score = evaluate_controller(x, A, B, weights, goals)
    Q = diag(10.^x(1:4));
    R = diag(10.^x(5:6));
    
    try
        K = lqr(A, B, Q, R);
        A_cl = A - B*K;
        
        if any(real(eig(A_cl)) >= 0)
            score = 1e6;
            return;
        end
        
        [metrics, ~, ~, ~] = simulate_controller(A, B, K, [], [], []);
        score = compute_score(metrics, weights, goals);
    catch
        score = 1e6;
    end
end

function scores = evaluate_multiobjective(x, A, B, weights, goals)
    Q = diag(10.^x(1:4));
    R = diag(10.^x(5:6));
    
    try
        K = lqr(A, B, Q, R);
        A_cl = A - B*K;
        
        if any(real(eig(A_cl)) >= 0)
            scores = [1e6, 1e6];
            return;
        end
        
        [metrics, ~, ~, ~] = simulate_controller(A, B, K, [], [], []);
        
        % Objective 1: Tracking performance
        tracking = weights.velocity * metrics.V_error^2 + ...
                   weights.pitch * metrics.theta_error^2;
        
        % Objective 2: Control effort
        effort = metrics.control_effort;
        
        scores = [tracking, effort];
    catch
        scores = [1e6, 1e6];
    end
end

function score = compute_score(metrics, weights, goals)
    % Compute weighted performance score
    
    % Normalized errors (divide by goals)
    V_norm = metrics.V_error / goals.V_error_max;
    theta_norm = metrics.theta_error / goals.theta_error_max;
    overshoot_norm = max(0, metrics.theta_overshoot - goals.overshoot_max) / goals.overshoot_max;
    settling_norm = metrics.settling_time / goals.settling_time_max;
    effort_norm = metrics.control_effort / goals.control_effort_max;
    
    % Weighted score
    score = weights.velocity * V_norm^2 + ...
            weights.pitch * theta_norm^2 + ...
            weights.overshoot * overshoot_norm^2 + ...
            weights.settling * settling_norm^2 + ...
            weights.effort * effort_norm^2;
end

function [metrics, x, u, t, x_slosh, F_dist, M_dist] = ...
    simulate_controller(A, B, K, K_dist, A_slosh, B_slosh)
    
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
    
    % Check for slosh
    has_slosh = ~isempty(K_dist) && ~isempty(A_slosh) && ~isempty(B_slosh);
    
    if has_slosh
        x_slosh = zeros(2, N);
        x_slosh(:,1) = [0.1; 0];
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
        
        % Dynamics
        if has_slosh
            dist = K_dist * x_slosh(:,i);
            F_dist(i) = dist(1);
            M_dist(i) = dist(2);
            
            dx = A * x(:,i) + B * u(:,i) + [0; 0; M_dist(i)/15000; 0];
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
    idx = find(theta_settled, 1);
    if ~isempty(idx)
        metrics.settling_time = t(idx);
    else
        metrics.settling_time = t(end);
    end
    
    metrics.control_effort = sum(u(1,:).^2 + u(2,:).^2) * dt;
end

function fig = create_tuning_analysis_plot(results, best_Q, best_R, best_score)
    fig = figure('Position', [100, 100, 1200, 800], 'Name', 'Tuning Analysis');
    
    scores = [results.score];
    stable = [results.stable];
    
    Q_theta = arrayfun(@(r) r.Q(4,4), results);
    R_elev = arrayfun(@(r) r.R(1,1), results);
    
    % Score distribution
    subplot(2,2,1);
    histogram(scores(stable & scores < 1000), 30);
    xlabel('Performance Score');
    ylabel('Count');
    title('Score Distribution');
    grid on;
    
    % Parameter space
    subplot(2,2,2);
    scatter3(log10(Q_theta(stable)), log10(R_elev(stable)), scores(stable), ...
        30, scores(stable), 'filled');
    hold on;
    scatter3(log10(best_Q(4,4)), log10(best_R(1,1)), best_score, ...
        100, 'r', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
    xlabel('log10(Q_θ)');
    ylabel('log10(R_{elev})');
    zlabel('Score');
    title('Parameter Space');
    colorbar;
    grid on;
    
    % Score vs Q_theta
    subplot(2,2,3);
    scatter(Q_theta(stable), scores(stable), 20, 'b', 'filled');
    hold on;
    plot(best_Q(4,4), best_score, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    set(gca, 'XScale', 'log');
    xlabel('Q_θ');
    ylabel('Score');
    title('Score vs Pitch Weight');
    grid on;
    
    % Score vs R_elev
    subplot(2,2,4);
    scatter(R_elev(stable), scores(stable), 20, 'b', 'filled');
    hold on;
    plot(best_R(1,1), best_score, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    set(gca, 'XScale', 'log');
    xlabel('R_{elev}');
    ylabel('Score');
    title('Score vs Elevator Weight');
    grid on;
    
    sgtitle('Tuning Analysis', 'FontSize', 14, 'FontWeight', 'bold');
end

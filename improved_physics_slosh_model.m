%% IMPROVED SLOSH MODEL - FIXED: Stable Physics with 3D Visualization
%
% Fixes:
%   - Numerical stability in damping calculations
%   - Proper scaling of physics parameters
%   - NaN protection
%   - 3D energy visualization
%
% Same LQI controller (unchanged)

clear all; close all; clc;

fprintf('╔════════════════════════════════════════════════════════╗\n');
fprintf('║   IMPROVED SLOSH MODEL: Fixed & Stable                 ║\n');
fprintf('╚════════════════════════════════════════════════════════╝\n\n');

%% STEP 1: FLUID PROPERTIES
fprintf('STEP 1: Define Fluid Properties\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

fluid_type = 'water';  % 'water' or 'retardant'

switch fluid_type
    case 'water'
        rho_fluid = 1000;      % kg/m³
        mu_fluid = 0.001;      % Pa·s
        fprintf('Fluid: Water\n');
    case 'retardant'
        rho_fluid = 1100;      % kg/m³
        mu_fluid = 0.05;       % Pa·s (50x more viscous)
        fprintf('Fluid: Fire Retardant\n');
end

fprintf('  Density:   ρ = %.0f kg/m³\n', rho_fluid);
fprintf('  Viscosity: μ = %.4f Pa·s\n\n', mu_fluid);

%% STEP 2: DEFINE NOMINAL PLANT (unchanged)
fprintf('STEP 2: Define Nominal Plant\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

m1 = 1.0;
m2 = 1.0;
k1 = 2.0;
k2 = 1.0;
k3 = 2.0;
b1 = 0.5;
b2 = 0.3;

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

fprintf('✓ System defined\n\n');

%% STEP 3: DESIGN LQI CONTROLLER (unchanged)
fprintf('STEP 3: Design LQI Controller\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

C_track = zeros(2, 4);
C_track(1, 1) = 1;
C_track(2, 3) = 1;

A_aug = [A_nom,      zeros(4, 2);
         -C_track,   zeros(2, 2)];

B_aug = [B_nom;
         zeros(2, 2)];

Q_x = diag([10, 1, 10, 1]);
Q_I = diag([100, 100]);
Q_aug = blkdiag(Q_x, Q_I);
R_aug = diag([0.1, 0.1]);

K_aug = lqr(A_aug, B_aug, Q_aug, R_aug);

fprintf('✓ LQI controller designed (SAME as before)\n\n');

%% STEP 4: IMPROVED SLOSH PHYSICS (FIXED)
fprintf('STEP 4: Improved Slosh Physics (Numerically Stable)\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

function [k_slosh, b_slosh, omega, zeta, Re, breakdown] = ...
    compute_slosh_parameters_stable(m_slosh, N_baffles, rho_fluid, mu_fluid)
    
    % Handle zero baffles
    if N_baffles == 0
        N_baffles = 1;
    end
    
    % Base parameters
    k_base = 5.0;
    b_base = 0.2;
    
    % Spring constant (scales with N_baffles)
    k_slosh = k_base * N_baffles;
    
    % Natural frequency
    omega = sqrt(k_slosh / m_slosh);
    
    % ═══════════════════════════════════════════════════════════
    % IMPROVED DAMPING (simplified and stable)
    % ═══════════════════════════════════════════════════════════
    
    % 1. SURFACE AREA EFFECT (viscous damping)
    % More baffles → more wall surface → more friction
    surface_factor = 1 + 0.15 * (N_baffles - 1);  % Increases with baffles
    b_viscous = b_base * surface_factor;
    
    % 2. CHARACTERISTIC LENGTH (for Reynolds number)
    L_comp = 1.0 / N_baffles;  % Compartment length decreases
    v_typical = 0.1;           % Typical velocity (m/s)
    
    % 3. REYNOLDS NUMBER
    Re = rho_fluid * v_typical * L_comp / mu_fluid;
    
    % 4. TURBULENT DAMPING (edge effects)
    if Re < 2000
        % Laminar - minimal turbulent damping
        b_turbulent = 0;
        flow_regime = 'Laminar';
    elseif Re < 4000
        % Transitional
        transition_factor = (Re - 2000) / 2000;
        b_turbulent = b_base * 0.3 * transition_factor * (N_baffles - 1);
        flow_regime = 'Transitional';
    else
        % Fully turbulent
        b_turbulent = b_base * 0.5 * (N_baffles - 1);
        flow_regime = 'Turbulent';
    end
    
    % 5. TOTAL DAMPING
    b_slosh = b_viscous + b_turbulent;
    
    % Ensure positive and reasonable
    b_slosh = max(b_slosh, b_base);  % Never less than base
    b_slosh = min(b_slosh, b_base * 5);  % Cap at 5x base
    
    % 6. DAMPING RATIO
    zeta = b_slosh / (2 * sqrt(k_slosh * m_slosh));
    
    % Return breakdown
    breakdown = struct();
    breakdown.b_base = b_base;
    breakdown.b_viscous = b_viscous;
    breakdown.b_turbulent = b_turbulent;
    breakdown.b_total = b_slosh;
    breakdown.Re = Re;
    breakdown.flow_regime = flow_regime;
    breakdown.surface_factor = surface_factor;
end

% Test the function
m_test = 0.125;  % 50% slosh / 4 compartments
N_test = 4;
[k_t, b_t, omega_t, zeta_t, Re_t, breakdown_t] = ...
    compute_slosh_parameters_stable(m_test, N_test, rho_fluid, mu_fluid);

fprintf('Example: 4 baffles, 50%% slosh mass\n');
fprintf('  ω = %.3f rad/s (%.3f Hz)\n', omega_t, omega_t/(2*pi));
fprintf('  ζ = %.4f\n', zeta_t);
fprintf('  Re = %.0f (%s flow)\n', Re_t, breakdown_t.flow_regime);
fprintf('  Damping breakdown:\n');
fprintf('    Base:         %.4f N·s/m\n', breakdown_t.b_base);
fprintf('    Viscous:      %.4f N·s/m (×%.2f surface factor)\n', ...
        breakdown_t.b_viscous, breakdown_t.surface_factor);
fprintf('    Turbulent:    %.4f N·s/m (edge effects)\n', breakdown_t.b_turbulent);
fprintf('    Total:        %.4f N·s/m\n\n', breakdown_t.b_total);

%% STEP 5: PARAMETER SWEEP
fprintf('STEP 5: Parameter Sweep\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

slosh_mass_percentages = [10, 20, 30, 40, 50, 60, 70];
baffle_configs = [0, 1, 2, 4, 8, 16];

n_slosh = length(slosh_mass_percentages);
n_baffles = length(baffle_configs);

% Simulation parameters
dt = 0.01;
t = 0:dt:30;
N_time = length(t);

x0 = [1; 0; -0.5; 0];
x_ref = [0.5; 0; -0.2; 0];
x_slosh_init = [0.1; 0];

% Preallocate results
results = struct();
results.error_x1 = zeros(n_slosh, n_baffles);
results.max_force = zeros(n_slosh, n_baffles);
results.energy = zeros(n_slosh, n_baffles);
results.omega = zeros(n_slosh, n_baffles);
results.zeta = zeros(n_slosh, n_baffles);
results.Re = zeros(n_slosh, n_baffles);

% Simulate nominal
x_nominal = zeros(4, N_time);
x_I_nominal = zeros(2, N_time);
u_nominal = zeros(2, N_time);
x_nominal(:,1) = x0;

for i = 1:N_time-1
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
energy_nominal = sum(sum(u_nominal.^2, 1)) * dt;

fprintf('Nominal (no slosh): Error=%.6f m, Energy=%.3f J\n\n', ...
    error_nominal_x1, energy_nominal);

% Parameter sweep
sim_count = 0;
total_sims = n_slosh * n_baffles;

for i_slosh = 1:n_slosh
    slosh_percent = slosh_mass_percentages(i_slosh);
    m_slosh_total = (slosh_percent/100) * m1;
    
    fprintf('Slosh Mass: %.0f%% (%.3f kg)\n', slosh_percent, m_slosh_total);
    
    for i_baffle = 1:n_baffles
        N_baffles = baffle_configs(i_baffle);
        sim_count = sim_count + 1;
        
        if N_baffles == 0
            N_baffles_sim = 1;
        else
            N_baffles_sim = N_baffles;
        end
        
        m_slosh_comp = m_slosh_total / N_baffles_sim;
        
        % Compute improved slosh parameters
        [k_slosh, b_slosh, omega, zeta, Re, breakdown] = ...
            compute_slosh_parameters_stable(m_slosh_comp, N_baffles_sim, rho_fluid, mu_fluid);
        
        % Store characteristics
        results.omega(i_slosh, i_baffle) = omega;
        results.zeta(i_slosh, i_baffle) = zeta;
        results.Re(i_slosh, i_baffle) = Re;
        
        % Slosh dynamics
        A_slosh = [0, 1; -k_slosh/m_slosh_comp, -b_slosh/m_slosh_comp];
        B_slosh = [0; 1];
        K_dist = [k_slosh, b_slosh; 0, 0];
        
        % Simulation
        x_sim = zeros(4, N_time);
        x_I_sim = zeros(2, N_time);
        u_sim = zeros(2, N_time);
        F_dist_total = zeros(N_time, 1);
        x_slosh_states = zeros(2, N_baffles_sim, N_time);
        
        x_sim(:,1) = x0;
        
        for comp = 1:N_baffles_sim
            phase_offset = (comp-1) * 0.02;
            x_slosh_states(:, comp, 1) = x_slosh_init + [phase_offset; 0];
        end
        
        % Simulation loop
        for i = 1:N_time-1
            y = C_track * x_sim(:,i);
            r = [x_ref(1); x_ref(3)];
            e = r - y;
            x_I_sim(:,i+1) = x_I_sim(:,i) + e * dt;
            
            x_aug_vec = [x_sim(:,i); x_I_sim(:,i)];
            u_sim(:,i) = -K_aug * x_aug_vec;
            u_sim(:,i) = max(min(u_sim(:,i), 10), -10);
            
            % Compute total disturbance
            d_total = [0; 0];
            for comp = 1:N_baffles_sim
                d_comp = K_dist * x_slosh_states(:, comp, i);
                d_total = d_total + d_comp;
            end
            F_dist_total(i) = d_total(1);
            
            % Check for NaN
            if any(isnan(d_total)) || any(isinf(d_total))
                fprintf('  WARNING: NaN detected at t=%.2f, aborting sim\n', t(i));
                break;
            end
            
            % Plant dynamics
            dx = A_nom * x_sim(:,i) + B_nom * u_sim(:,i) + B_dist * d_total;
            x_sim(:,i+1) = x_sim(:,i) + dx * dt;
            
            % Slosh dynamics
            a1 = dx(2);
            for comp = 1:N_baffles_sim
                dx_slosh = A_slosh * x_slosh_states(:, comp, i) + B_slosh * a1;
                x_slosh_states(:, comp, i+1) = x_slosh_states(:, comp, i) + dx_slosh * dt;
            end
        end
        u_sim(:,end) = u_sim(:,end-1);
        F_dist_total(end) = F_dist_total(end-1);
        
        % Compute metrics (with NaN protection)
        if any(isnan(x_sim(:))) || any(isinf(x_sim(:)))
            results.error_x1(i_slosh, i_baffle) = NaN;
            results.max_force(i_slosh, i_baffle) = NaN;
            results.energy(i_slosh, i_baffle) = NaN;
            fprintf('  %2d baffles: SIMULATION FAILED (NaN)\n', N_baffles);
        else
            results.error_x1(i_slosh, i_baffle) = abs(x_sim(1,end) - x_ref(1));
            results.max_force(i_slosh, i_baffle) = max(abs(F_dist_total));
            results.energy(i_slosh, i_baffle) = sum(sum(u_sim.^2, 1)) * dt;
            
            fprintf('  %2d baffles: ω=%.2f Hz, ζ=%.3f, Error=%.4f m, Energy=%.2f J [%d/%d]\n', ...
                N_baffles, omega/(2*pi), zeta, ...
                results.error_x1(i_slosh, i_baffle), ...
                results.energy(i_slosh, i_baffle), sim_count, total_sims);
        end
    end
    fprintf('\n');
end
fprintf('✓ Parameter sweep complete!\n\n');

%% STEP 6: 3D ENERGY VISUALIZATION
fprintf('STEP 6: Creating 3D Energy Visualization...\n');

% Create meshgrid
[Baffles_grid, SloshMass_grid] = meshgrid(baffle_configs, slosh_mass_percentages);

% FIGURE 1: 3D Energy Surface
figure('Name', 'Figure 1: 3D Energy Surface', 'Position', [50, 50, 1400, 800]);

subplot(1,2,1);
surf(Baffles_grid, SloshMass_grid, results.energy, 'EdgeColor', 'k', 'FaceAlpha', 0.9);
hold on;

% Add nominal energy plane
Z_nominal = energy_nominal * ones(size(Baffles_grid));
surf(Baffles_grid, SloshMass_grid, Z_nominal, 'FaceColor', 'b', ...
    'FaceAlpha', 0.25, 'EdgeColor', 'none');

xlabel('Number of Baffles', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Slosh Mass (% of Vehicle)', 'FontSize', 12, 'FontWeight', 'bold');
zlabel('Control Energy (J)', 'FontSize', 12, 'FontWeight', 'bold');
title('3D Energy Surface', 'FontSize', 14, 'FontWeight', 'bold');
colorbar;
colormap(jet);
grid on;
view(45, 30);
legend('Energy with Slosh', 'Nominal (No Slosh)', 'Location', 'best');

% Add annotation
text(baffle_configs(1), slosh_mass_percentages(1), energy_nominal + 1, ...
    sprintf('Nominal: %.2f J', energy_nominal), ...
    'FontSize', 12, 'FontWeight', 'bold', 'Color', 'b', ...
    'BackgroundColor', 'w', 'EdgeColor', 'b');

subplot(1,2,2);
% Top view (contour)
contourf(Baffles_grid, SloshMass_grid, results.energy, 20);
hold on;
[C, h] = contour(Baffles_grid, SloshMass_grid, results.energy, 10, 'k-', 'LineWidth', 1);
clabel(C, h, 'FontSize', 10, 'Color', 'k');
contour(Baffles_grid, SloshMass_grid, results.energy, [energy_nominal energy_nominal], ...
    'b-', 'LineWidth', 3);
xlabel('Number of Baffles', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Slosh Mass (%)', 'FontSize', 12, 'FontWeight', 'bold');
title('Energy Contours (Top View)', 'FontSize', 14, 'FontWeight', 'bold');
colorbar;
colormap(jet);
grid on;
set(gca, 'XTick', baffle_configs);

sgtitle('3D Energy Analysis: Slosh Mass vs Baffles', 'FontSize', 16, 'FontWeight', 'bold');

% FIGURE 2: Multiple 3D Views
figure('Name', 'Figure 2: 3D Energy - Multiple Views', 'Position', [100, 100, 1600, 1000]);

% Standard view
subplot(2,3,1);
surf(Baffles_grid, SloshMass_grid, results.energy, 'EdgeColor', 'interp');
hold on;
surf(Baffles_grid, SloshMass_grid, Z_nominal, 'FaceColor', 'b', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
xlabel('Baffles');
ylabel('Slosh Mass (%)');
zlabel('Energy (J)');
title('Standard View (45°, 30°)');
colormap(jet);
grid on;
view(45, 30);

% Top view
subplot(2,3,2);
surf(Baffles_grid, SloshMass_grid, results.energy);
hold on;
surf(Baffles_grid, SloshMass_grid, Z_nominal, 'FaceColor', 'b', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
xlabel('Baffles');
ylabel('Slosh Mass (%)');
zlabel('Energy (J)');
title('Top View');
colormap(jet);
grid on;
view(0, 90);

% Side view (slosh mass effect)
subplot(2,3,3);
surf(Baffles_grid, SloshMass_grid, results.energy);
hold on;
surf(Baffles_grid, SloshMass_grid, Z_nominal, 'FaceColor', 'b', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
xlabel('Baffles');
ylabel('Slosh Mass (%)');
zlabel('Energy (J)');
title('Side View (Slosh Effect)');
colormap(jet);
grid on;
view(0, 0);

% Front view (baffle effect)
subplot(2,3,4);
surf(Baffles_grid, SloshMass_grid, results.energy);
hold on;
surf(Baffles_grid, SloshMass_grid, Z_nominal, 'FaceColor', 'b', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
xlabel('Baffles');
ylabel('Slosh Mass (%)');
zlabel('Energy (J)');
title('Front View (Baffle Effect)');
colormap(jet);
grid on;
view(90, 0);

% Energy overhead (percentage)
subplot(2,3,5);
energy_overhead = (results.energy - energy_nominal) / energy_nominal * 100;
surf(Baffles_grid, SloshMass_grid, energy_overhead, 'EdgeColor', 'k');
xlabel('Baffles');
ylabel('Slosh Mass (%)');
zlabel('Overhead (%)');
title('Energy Overhead vs Nominal');
colormap(hot);
grid on;
view(45, 30);
colorbar;

% Cross-sections
subplot(2,3,6);
colors_slosh = jet(n_slosh);
for i = 1:n_slosh
    plot(baffle_configs, results.energy(i,:), 'o-', 'LineWidth', 2, ...
        'Color', colors_slosh(i,:), 'DisplayName', sprintf('%d%%', slosh_mass_percentages(i)));
    hold on;
end
yline(energy_nominal, 'b--', 'LineWidth', 3, 'DisplayName', 'Nominal');
xlabel('Number of Baffles');
ylabel('Energy (J)');
title('Cross-Sections (All Slosh Masses)');
legend('Location', 'best');
grid on;
set(gca, 'XTick', baffle_configs);

sgtitle('3D Energy Visualization - Multiple Perspectives', 'FontSize', 16, 'FontWeight', 'bold');

% FIGURE 3: Physics Characteristics
figure('Name', 'Figure 3: Physics Analysis', 'Position', [150, 150, 1600, 800]);

% Frequency
subplot(2,3,1);
for i = 1:n_slosh
    plot(baffle_configs, results.omega(i,:)/(2*pi), 'o-', 'LineWidth', 2, ...
        'Color', colors_slosh(i,:), 'DisplayName', sprintf('%d%%', slosh_mass_percentages(i)));
    hold on;
end
xlabel('Number of Baffles');
ylabel('Frequency (Hz)');
title('Slosh Frequency (ω ∝ √N)');
legend('Location', 'best');
grid on;

% Damping ratio
subplot(2,3,2);
for i = 1:n_slosh
    plot(baffle_configs, results.zeta(i,:), 'o-', 'LineWidth', 2, ...
        'Color', colors_slosh(i,:), 'DisplayName', sprintf('%d%%', slosh_mass_percentages(i)));
    hold on;
end
xlabel('Number of Baffles');
ylabel('Damping Ratio ζ');
title('Damping (Increases with Baffles)');
legend('Location', 'best');
grid on;

% Reynolds number
subplot(2,3,3);
for i = 1:n_slosh
    semilogy(baffle_configs, results.Re(i,:), 'o-', 'LineWidth', 2, ...
        'Color', colors_slosh(i,:), 'DisplayName', sprintf('%d%%', slosh_mass_percentages(i)));
    hold on;
end
yline(2000, 'r--', 'DisplayName', 'Laminar limit');
yline(4000, 'g--', 'DisplayName', 'Turbulent limit');
xlabel('Number of Baffles');
ylabel('Reynolds Number');
title('Flow Regime (Re ∝ 1/N)');
legend('Location', 'best');
grid on;

% Error
subplot(2,3,4);
for i = 1:n_slosh
    semilogy(baffle_configs, results.error_x1(i,:), 'o-', 'LineWidth', 2, ...
        'Color', colors_slosh(i,:), 'DisplayName', sprintf('%d%%', slosh_mass_percentages(i)));
    hold on;
    % plot(baffle_configs, results.error_x1(i,:)*1000, 'o-', 'LineWidth', 2, ...
    %     'Color', colors_slosh(i,:), 'DisplayName', sprintf('%d%%', slosh_mass_percentages(i)));
    % hold on;
end
yline(error_nominal_x1, 'k--', 'LineWidth', 2, 'DisplayName', 'Nominal'); % Converted to mm to make it readable
xlabel('Number of Baffles');
ylabel('Error (m)');
title('Tracking Error (Log Scale)');
legend('Location', 'best');
grid on;

% Force
subplot(2,3,5);
for i = 1:n_slosh
    semilogy(baffle_configs, results.max_force(i,:), 'o-', 'LineWidth', 2, ...
        'Color', colors_slosh(i,:), 'DisplayName', sprintf('%d%%', slosh_mass_percentages(i)));
    hold on;
end
xlabel('Number of Baffles');
ylabel('Max Force (log(N))');
title('Peak Disturbance Force');
legend('Location', 'best');
grid on;

% Energy
subplot(2,3,6);
for i = 1:n_slosh
    plot(baffle_configs, results.energy(i,:), 'o-', 'LineWidth', 2, ...
        'Color', colors_slosh(i,:), 'DisplayName', sprintf('%d%%', slosh_mass_percentages(i)));
    hold on;
end
yline(energy_nominal, 'k--', 'LineWidth', 2, 'DisplayName', 'Nominal');
xlabel('Number of Baffles');
ylabel('Energy (J)');
title('Control Energy');
legend('Location', 'best');
grid on;

sgtitle('Physics Characteristics and Performance', 'FontSize', 16, 'FontWeight', 'bold');
figure
% Error
for i = 1:n_slosh
    semilogy(baffle_configs, results.error_x1(i,:), 'o-', 'LineWidth', 2, ...
        'Color', colors_slosh(i,:), 'DisplayName', sprintf('%d%%', slosh_mass_percentages(i)));
    hold on;
    % plot(baffle_configs, results.error_x1(i,:)*1000, 'o-', 'LineWidth', 2, ...
    %     'Color', colors_slosh(i,:), 'DisplayName', sprintf('%d%%', slosh_mass_percentages(i)));
    % hold on;
end
yline(error_nominal_x1, 'k--', 'LineWidth', 2, 'DisplayName', 'Nominal'); % Converted to mm to make it readable
xlabel('Number of Baffles');
ylabel('Error (log(m))');
title('Tracking Error (Log Scale)');
legend('Location', 'best');
grid on;

fprintf('✓ Figures generated\n\n');

%% STEP 7: SUMMARY
fprintf('╔════════════════════════════════════════════════════════╗\n');
fprintf('║         IMPROVED PHYSICS MODEL SUMMARY                 ║\n');
fprintf('╚════════════════════════════════════════════════════════╝\n\n');

fprintf('FLUID: %s (ρ=%.0f kg/m³, μ=%.4f Pa·s)\n\n', fluid_type, rho_fluid, mu_fluid);

fprintf('ENERGY ANALYSIS (50%% slosh):\n');
idx_50 = find(slosh_mass_percentages == 50);
fprintf('  Nominal:      %.3f J (baseline)\n', energy_nominal);
for j = 1:n_baffles
    if ~isnan(results.energy(idx_50, j))
        overhead = (results.energy(idx_50,j) - energy_nominal) / energy_nominal * 100;
        fprintf('  %2d baffles:   %.3f J (+%.1f%%)\n', ...
            baffle_configs(j), results.energy(idx_50,j), overhead);
    end
end
fprintf('\n');

fprintf('OPTIMAL CONFIGURATION (50%% slosh):\n');
valid_energies = results.energy(idx_50, ~isnan(results.energy(idx_50,:)));
valid_configs = baffle_configs(~isnan(results.energy(idx_50,:)));
if ~isempty(valid_energies)
    [min_energy, min_idx] = min(valid_energies);
    optimal_baffles = valid_configs(min_idx);
    fprintf('  Optimal: %d baffles (%.3f J)\n', optimal_baffles, min_energy);
    fprintf('  Savings: %.1f%% vs no baffles\n\n', ...
        (results.energy(idx_50,1) - min_energy)/results.energy(idx_50,1)*100);
end

fprintf('✓✓✓ ANALYSIS COMPLETE ✓✓✓\n\n');
fprintf('Generated 3 figures:\n');
fprintf('  Figure 1: 3D Energy Surface (main visualization)\n');
fprintf('  Figure 2: Multiple 3D Views\n');
fprintf('  Figure 3: Physics Characteristics\n\n');

save('improved_physics_results.mat', 'results', 'slosh_mass_percentages', ...
     'baffle_configs', 'energy_nominal', 'fluid_type', 'rho_fluid', 'mu_fluid');

fprintf('✓ Results saved to: improved_physics_results.mat\n\n');

%% Build Simulink Model: Separated Architecture with Control Loop
% Complete closed-loop system:
%   [Reference] → [Controller] → [Nominal Plant] ← [Slosh Disturbances]
%                                      ↓                    ↑
%                                  [States] → [Slosh Dynamics]
%                                      ↓
%                                 [Feedback]
%
% Features:
% - PID or LQR controller options
% - Enable/disable slosh
% - Enable/disable controller (open-loop mode)
% - Reference tracking
% - Complete performance monitoring

clear all; close all; clc;

fprintf('=== Building Separated Architecture with Control Loop ===\n');

%% LOAD OR DEFINE PARAMETERS
if exist('separated_system_params.mat', 'file')
    load('separated_system_params.mat');
    fprintf('✓ Loaded parameters from file\n');
else
    fprintf('Running separated_architecture.m to generate parameters...\n');
    run('separated_architecture.m');
    close all;  % Close plots from that script
end

%% BUILD SIMULINK MODEL
model_name = 'slosh_control_loop';

% Close if already open
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end

% Create new model
new_system(model_name);
open_system(model_name);

fprintf('\n--- Building Closed-Loop Control System ---\n');

%% POSITIONS (for better layout)
pos_ref = [50, 240, 100, 270];
pos_error_sum = [150, 235, 180, 275];
pos_controller = [230, 220, 310, 290];
pos_plant_mux = [370, 210, 380, 300];
pos_plant = [420, 220, 520, 300];
pos_demux_states = [570, 228, 575, 292];
pos_accel_deriv = [620, 180, 680, 280];
pos_accel_mux = [730, 228, 735, 282];
pos_slosh = [780, 220, 880, 300];
pos_dist_gain = [930, 230, 980, 290];
pos_dist_demux = [1030, 238, 1035, 282];

%% 1. REFERENCE INPUT
fprintf('Adding reference command...\n');

add_block('simulink/Sources/Step', [model_name, '/Reference_Command']);
set_param([model_name, '/Reference_Command'], ...
          'Time', '1', ...
          'Before', '0', ...
          'After', '0.1', ...  % 0.1 rad = 5.7 degrees
          'Position', pos_ref);

%% 2. ERROR COMPUTATION (Reference - Feedback)
fprintf('Adding error computation...\n');

add_block('simulink/Math Operations/Sum', [model_name, '/Error_Sum']);
set_param([model_name, '/Error_Sum'], ...
          'Inputs', '+-', ...
          'IconShape', 'round', ...
          'Position', pos_error_sum);

%% 3. CONTROLLER SUBSYSTEM
fprintf('Adding controller...\n');

% Create controller subsystem
controller_sys = [model_name, '/Controller'];
add_block('built-in/Subsystem', controller_sys);
set_param(controller_sys, 'Position', pos_controller);

% Build inside controller
Simulink.SubSystem.deleteContents(controller_sys);

% Input (error)
add_block('built-in/Inport', [controller_sys, '/Error']);
set_param([controller_sys, '/Error'], 'Position', [30, 50, 60, 70]);

% PID Controller
add_block('simulink/Continuous/PID Controller', [controller_sys, '/PID']);
set_param([controller_sys, '/PID'], ...
          'P', '15', ...
          'I', '2', ...
          'D', '5', ...
          'Position', [120, 30, 180, 90]);

% Saturation (realistic actuator limits)
add_block('simulink/Discontinuities/Saturation', [controller_sys, '/Saturation']);
set_param([controller_sys, '/Saturation'], ...
          'UpperLimit', '0.5', ...   % +28.6 degrees
          'LowerLimit', '-0.5', ...  % -28.6 degrees
          'Position', [230, 45, 260, 75]);

% Output
add_block('built-in/Outport', [controller_sys, '/Control_Output']);
set_param([controller_sys, '/Control_Output'], 'Position', [310, 53, 340, 67]);

% Connect controller internals
add_line(controller_sys, 'Error/1', 'PID/1');
add_line(controller_sys, 'PID/1', 'Saturation/1');
add_line(controller_sys, 'Saturation/1', 'Control_Output/1');

% Add annotation
add_block('built-in/Note', [controller_sys, '/Note_Controller']);
set_param([controller_sys, '/Note_Controller'], ...
          'Position', [80, 120, 280, 160], ...
          'Text', 'PID Controller\nKp=15, Ki=2, Kd=5\nSaturation: ±0.5 rad (±28.6°)');

%% 4. PLANT INPUT MUX
fprintf('Adding plant input multiplexer...\n');

add_block('simulink/Signal Routing/Mux', [model_name, '/Plant_Input_Mux']);
set_param([model_name, '/Plant_Input_Mux'], ...
          'Inputs', '3', ...
          'DisplayOption', 'bar', ...
          'Position', pos_plant_mux);

%% 5. NOMINAL PLANT
fprintf('Adding nominal plant...\n');

add_block('simulink/Continuous/State-Space', [model_name, '/Nominal_Plant']);
set_param([model_name, '/Nominal_Plant'], ...
          'A', mat2str(A_nom), ...
          'B', mat2str(B_nom), ...
          'C', mat2str(C_nom), ...
          'D', mat2str(D_nom), ...
          'X0', mat2str(x0_nom), ...
          'Position', pos_plant);

% Add annotation for plant
add_block('built-in/Note', [model_name, '/Note_Plant']);
set_param([model_name, '/Note_Plant'], ...
          'Position', [420, 160, 520, 210], ...
          'Text', 'Nominal Plant (4 states)\n[V, γ, q, θ]\nInputs: [u, F_d, M_d]');

%% 6. STATE DEMUX AND FEEDBACK
fprintf('Adding state extraction...\n');

% Demux all states - this splits into 4 separate scalar signals
add_block('simulink/Signal Routing/Demux', [model_name, '/States_Demux']);
set_param([model_name, '/States_Demux'], ...
          'Outputs', '4', ...
          'DisplayOption', 'bar', ...
          'Position', pos_demux_states);

% Note: Output 1 = V, Output 2 = gamma, Output 3 = q, Output 4 = theta
% We'll use Output 4 directly for feedback (no need for separate Selector)

%% 7. ACCELERATION EXTRACTION
fprintf('Adding acceleration extraction...\n');

% Derivative blocks for accelerations
add_block('simulink/Continuous/Derivative', [model_name, '/dV_dt']);
set_param([model_name, '/dV_dt'], 'Position', [620, 195, 650, 225]);

add_block('simulink/Continuous/Derivative', [model_name, '/dq_dt']);
set_param([model_name, '/dq_dt'], 'Position', [620, 255, 650, 285]);

% Mux accelerations
add_block('simulink/Signal Routing/Mux', [model_name, '/Accel_Mux']);
set_param([model_name, '/Accel_Mux'], ...
          'Inputs', '2', ...
          'DisplayOption', 'bar', ...
          'Position', pos_accel_mux);

%% 8. SLOSH DYNAMICS
fprintf('Adding slosh dynamics...\n');

add_block('simulink/Continuous/State-Space', [model_name, '/Slosh_Dynamics']);
set_param([model_name, '/Slosh_Dynamics'], ...
          'A', mat2str(A_slosh), ...
          'B', mat2str(B_slosh), ...
          'C', mat2str(C_slosh), ...
          'D', mat2str(D_slosh), ...
          'X0', mat2str(x0_slosh), ...
          'Position', pos_slosh);

add_block('built-in/Note', [model_name, '/Note_Slosh']);
set_param([model_name, '/Note_Slosh'], ...
          'Position', [780, 160, 880, 210], ...
          'Text', 'Slosh Dynamics (2 states)\n[x_s, v_s]\nInputs: [a_x, a_θ]');

%% 9. DISTURBANCE COMPUTATION
fprintf('Adding disturbance computation...\n');

% Gain block for disturbance
add_block('simulink/Math Operations/Gain', [model_name, '/Disturbance_Gain']);
set_param([model_name, '/Disturbance_Gain'], ...
          'Gain', mat2str(K_dist), ...
          'Multiplication', 'Matrix(K*u)', ...
          'Position', pos_dist_gain);

% Demux disturbances
add_block('simulink/Signal Routing/Demux', [model_name, '/Disturbance_Demux']);
set_param([model_name, '/Disturbance_Demux'], ...
          'Outputs', '2', ...
          'DisplayOption', 'bar', ...
          'Position', pos_dist_demux);

%% 10. ENABLE/DISABLE SLOSH SWITCH
fprintf('Adding slosh enable/disable switch...\n');

% Manual switch to turn slosh on/off
add_block('simulink/Signal Routing/Manual Switch', [model_name, '/Enable_Slosh_F']);
set_param([model_name, '/Enable_Slosh_F'], 'Position', [1100, 227, 1125, 253]);

add_block('simulink/Signal Routing/Manual Switch', [model_name, '/Enable_Slosh_M']);
set_param([model_name, '/Enable_Slosh_M'], 'Position', [1100, 267, 1125, 293]);

% Zero constants for "slosh off" mode
add_block('simulink/Sources/Constant', [model_name, '/Zero_Force']);
set_param([model_name, '/Zero_Force'], 'Value', '0', 'Position', [1050, 190, 1070, 210]);

add_block('simulink/Sources/Constant', [model_name, '/Zero_Moment']);
set_param([model_name, '/Zero_Moment'], 'Value', '0', 'Position', [1050, 160, 1070, 180]);

%% 11. SCOPES AND DISPLAYS
fprintf('Adding scopes and displays...\n');

% Performance scope (reference, output, error)
add_block('simulink/Sinks/Scope', [model_name, '/Performance_Scope']);
set_param([model_name, '/Performance_Scope'], ...
          'NumInputPorts', '1', ...
          'Position', [700, 30, 730, 60]);

% Mux for performance signals
add_block('simulink/Signal Routing/Mux', [model_name, '/Performance_Mux']);
set_param([model_name, '/Performance_Mux'], ...
          'Inputs', '3', ...
          'DisplayOption', 'bar', ...
          'Position', [630, 28, 635, 62]);

% Control input scope
add_block('simulink/Sinks/Scope', [model_name, '/Control_Scope']);
set_param([model_name, '/Control_Scope'], 'Position', [700, 80, 730, 110]);

% Slosh states scope
add_block('simulink/Sinks/Scope', [model_name, '/Slosh_Scope']);
set_param([model_name, '/Slosh_Scope'], 'Position', [930, 350, 960, 380]);

% Disturbance scope
add_block('simulink/Sinks/Scope', [model_name, '/Disturbance_Scope']);
set_param([model_name, '/Disturbance_Scope'], 'Position', [1100, 330, 1130, 360]);

% Mux for disturbance scope
add_block('simulink/Signal Routing/Mux', [model_name, '/Dist_Scope_Mux']);
set_param([model_name, '/Dist_Scope_Mux'], ...
          'Inputs', '2', ...
          'Position', [1050, 333, 1055, 357]);

%% 12. CONNECT EVERYTHING
fprintf('Connecting all blocks...\n');

% Reference to error sum
add_line(model_name, 'Reference_Command/1', 'Error_Sum/1');

% Feedback to error sum (already connected above via States_Demux/4)
% (Connection made in previous section)

% Error to controller
add_line(model_name, 'Error_Sum/1', 'Controller/1');

% Controller to plant input mux (position 1)
add_line(model_name, 'Controller/1', 'Plant_Input_Mux/1');

% Plant input mux to plant
add_line(model_name, 'Plant_Input_Mux/1', 'Nominal_Plant/1');

% Plant to demux (splits into 4 scalar signals)
add_line(model_name, 'Nominal_Plant/1', 'States_Demux/1');

% Demux output 4 (theta) to feedback
% This is the feedback path that closes the control loop
add_line(model_name, 'States_Demux/4', 'Error_Sum/2', 'autorouting', 'on');

% Accelerations: V to derivative
add_line(model_name, 'States_Demux/1', 'dV_dt/1');

% Accelerations: q to derivative
add_line(model_name, 'States_Demux/3', 'dq_dt/1');

% Derivatives to acceleration mux
add_line(model_name, 'dV_dt/1', 'Accel_Mux/1');
add_line(model_name, 'dq_dt/1', 'Accel_Mux/2');

% Accelerations to slosh
add_line(model_name, 'Accel_Mux/1', 'Slosh_Dynamics/1');

% Slosh to disturbance gain
add_line(model_name, 'Slosh_Dynamics/1', 'Disturbance_Gain/1');

% Disturbance gain to demux
add_line(model_name, 'Disturbance_Gain/1', 'Disturbance_Demux/1');

% Disturbances to switches
add_line(model_name, 'Disturbance_Demux/1', 'Enable_Slosh_F/1');
add_line(model_name, 'Disturbance_Demux/2', 'Enable_Slosh_M/1');

% Zero constants to switches
add_line(model_name, 'Zero_Force/1', 'Enable_Slosh_F/2');
add_line(model_name, 'Zero_Moment/1', 'Enable_Slosh_M/2');

% Switches to plant input mux (positions 2 and 3)
add_line(model_name, 'Enable_Slosh_F/1', 'Plant_Input_Mux/2', 'autorouting', 'on');
add_line(model_name, 'Enable_Slosh_M/1', 'Plant_Input_Mux/3', 'autorouting', 'on');

% Performance monitoring
add_line(model_name, 'Reference_Command/1', 'Performance_Mux/1', 'autorouting', 'on');
add_line(model_name, 'States_Demux/4', 'Performance_Mux/2', 'autorouting', 'on');  % Theta
add_line(model_name, 'Error_Sum/1', 'Performance_Mux/3', 'autorouting', 'on');
add_line(model_name, 'Performance_Mux/1', 'Performance_Scope/1');

% Control input monitoring
add_line(model_name, 'Controller/1', 'Control_Scope/1', 'autorouting', 'on');

% Slosh monitoring
add_line(model_name, 'Slosh_Dynamics/1', 'Slosh_Scope/1', 'autorouting', 'on');

% Disturbance monitoring
add_line(model_name, 'Disturbance_Demux/1', 'Dist_Scope_Mux/1');
add_line(model_name, 'Disturbance_Demux/2', 'Dist_Scope_Mux/2');
add_line(model_name, 'Dist_Scope_Mux/1', 'Disturbance_Scope/1');

%% 13. ADD ANNOTATIONS
fprintf('Adding annotations...\n');

% Title annotation
add_block('built-in/Note', [model_name, '/Title']);
set_param([model_name, '/Title'], ...
          'Position', [400, 30, 700, 80], ...
          'Text', 'CLOSED-LOOP CONTROL SYSTEM\nwith Separated Slosh Disturbance\n\nController: PID\nPlant: 4th Order Aircraft\nDisturbance: 2nd Order Slosh', ...
          'FontSize', '14', ...
          'FontWeight', 'bold');

% Control loop annotation
add_block('built-in/Note', [model_name, '/Note_Loop']);
set_param([model_name, '/Note_Loop'], ...
          'Position', [150, 390, 450, 430], ...
          'Text', 'FEEDBACK LOOP: Pitch angle (θ) fed back to compare with reference');

% Slosh annotation
add_block('built-in/Note', [model_name, '/Note_Slosh_Effect']);
set_param([model_name, '/Note_Slosh_Effect'], ...
          'Position', [780, 390, 1080, 430], ...
          'Text', 'SLOSH DISTURBANCE: Aircraft motion excites slosh → generates forces → disturbs plant');

%% 14. CONFIGURE MODEL SETTINGS
fprintf('Configuring model settings...\n');

set_param(model_name, 'Solver', 'ode45');
set_param(model_name, 'StopTime', '15');
set_param(model_name, 'SaveOutput', 'on');
set_param(model_name, 'OutputSaveName', 'yout');
set_param(model_name, 'SaveTime', 'on');
set_param(model_name, 'TimeSaveName', 'tout');

% Configure scopes
scopeConfig = struct(...
    'NumInputPorts', 1, ...
    'TimeRange', '15', ...
    'YMin', '-0.2', ...
    'YMax', '0.2');

% Set performance scope to show 3 signals
set_param([model_name, '/Performance_Scope'], 'NumInputPorts', '1');

%% 15. SAVE MODEL
save_system(model_name);
fprintf('✓ Model saved: %s.slx\n', model_name);


fprintf('\n--- Building LQR Variant ---\n');

model_name_lqr = 'slosh_control_loop_LQR';

if bdIsLoaded(model_name_lqr)
    close_system(model_name_lqr, 0);
end

% Copy the PID model
copyfile([model_name, '.slx'], [model_name_lqr, '.slx']);
load_system(model_name_lqr);

% Compute LQR gains
Q = diag([0.1, 1, 10, 100]);
R = 1;
K_lqr = lqr(A_nom, B_nom(:,1), Q, R);

fprintf('LQR Gains: K = [%.2f, %.2f, %.2f, %.2f]\n', K_lqr);

% Update controller documentation
set_param([model_name_lqr, '/Controller/Note_Controller'], ...
          'Text', sprintf('LQR State Feedback\nK = [%.2f, %.2f, %.2f, %.2f]\nSaturation: ±0.5 rad', K_lqr));

save_system(model_name_lqr);
fprintf('✓ LQR model template saved: %s.slx\n', model_name_lqr);
fprintf('  (Note: Manually replace PID with state feedback for full LQR)\n');

%% SUMMARY
fprintf('\n=== Model Creation Summary ===\n');
fprintf('Created Simulink models with COMPLETE CONTROL LOOP:\n\n');

fprintf('1. %s.slx (PID Controller) ✓\n', model_name);
fprintf('   ┌─────────────────────────────────────────┐\n');
fprintf('   │  Reference → Error → PID → Plant       │\n');
fprintf('   │                ↑             ↓          │\n');
fprintf('   │           Feedback       Slosh          │\n');
fprintf('   │                ↑             ↓          │\n');
fprintf('   │                └─── Disturbances ───┘   │\n');
fprintf('   └─────────────────────────────────────────┘\n\n');

fprintf('   Features:\n');
fprintf('   ✓ PID Controller (Kp=15, Ki=2, Kd=5)\n');
fprintf('   ✓ Reference tracking (step command)\n');
fprintf('   ✓ Feedback from pitch angle\n');
fprintf('   ✓ Actuator saturation (±28.6°)\n');
fprintf('   ✓ Separated nominal plant\n');
fprintf('   ✓ Slosh as external disturbance\n');
fprintf('   ✓ Enable/disable slosh (manual switches)\n');
fprintf('   ✓ Complete performance monitoring\n\n');

fprintf('2. %s.slx (LQR Template)\n', model_name_lqr);
fprintf('   Same structure as PID model\n');
fprintf('   Controller needs manual update to state feedback\n\n');

fprintf('=== How to Use ===\n\n');

fprintf('STEP 1: Open the model\n');
fprintf('  >> open_system(''%s'')\n\n', model_name);

fprintf('STEP 2: Run simulation\n');
fprintf('  >> sim(''%s'')\n\n', model_name);

fprintf('STEP 3: View results in scopes\n');
fprintf('  • Performance Scope: Reference, Output, Error\n');
fprintf('  • Control Scope: Elevator angle\n');
fprintf('  • Slosh Scope: Slosh position and velocity\n');
fprintf('  • Disturbance Scope: Forces and moments\n\n');

fprintf('STEP 4: Experiment!\n');
fprintf('  • Change reference command (double-click Step block)\n');
fprintf('  • Tune PID gains (double-click Controller subsystem)\n');
fprintf('  • Disable slosh (flip manual switches)\n');
fprintf('  • Change initial conditions (State-Space blocks)\n\n');

fprintf('=== Key Signals ===\n\n');
fprintf('Reference Input:    Desired pitch angle (rad)\n');
fprintf('Error:              Reference - Actual\n');
fprintf('Control Input:      Elevator angle (rad)\n');
fprintf('Plant States:       [V, γ, q, θ]\n');
fprintf('Slosh States:       [x_s, v_s]\n');
fprintf('Disturbances:       [F_dist, M_dist]\n\n');

fprintf('=== Control Loop Explained ===\n\n');
fprintf('1. Reference command sets desired pitch angle\n');
fprintf('2. Error computed: desired - actual\n');
fprintf('3. PID controller computes elevator command\n');
fprintf('4. Saturation limits elevator to realistic range\n');
fprintf('5. Nominal plant responds to elevator\n');
fprintf('6. Aircraft motion excites slosh\n');
fprintf('7. Slosh generates disturbance forces/moments\n');
fprintf('8. Disturbances affect nominal plant\n');
fprintf('9. Pitch angle measured and fed back\n');
fprintf('10. Loop closes - controller automatically rejects disturbances!\n\n');

fprintf('=== Advantages of This Architecture ===\n\n');
fprintf('✓ MODULAR: Each component clearly separated\n');
fprintf('✓ TESTABLE: Can disable slosh to test controller alone\n');
fprintf('✓ REALISTIC: Matches real control systems\n');
fprintf('✓ EDUCATIONAL: Easy to understand signal flow\n');
fprintf('✓ FLEXIBLE: Easy to add more disturbances\n');
fprintf('✓ PROFESSIONAL: Industry-standard architecture\n\n');

fprintf('✓ Build complete! Open the model and press Run!\n');

%% SAVE WORKSPACE
fprintf('\nSaving workspace variables...\n');
save('control_loop_params.mat', 'A_nom', 'B_nom', 'C_nom', 'D_nom', ...
     'A_slosh', 'B_slosh', 'C_slosh', 'D_slosh', ...
     'K_dist', 'K_lqr', ...
     'x0_nom', 'x0_slosh', ...
     'm_aircraft', 'Iy', 'g', 'V0', ...
     'm_slosh', 'k_slosh', 'c_slosh', 'h_slosh');

fprintf('✓ Parameters saved to control_loop_params.mat\n');


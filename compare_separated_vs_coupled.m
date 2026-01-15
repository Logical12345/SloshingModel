% Compare Separated vs Coupled Architecture

clear all; close all; clc;

fprintf('Running both architectures...\n');

% Run separated architecture
sim('slosh_separated_architecture');
t_sep = yout.time;
y_sep = yout.signals.values;

% Run coupled architecture
sim('slosh_model_monolithic');
t_coup = yout.time;
y_coup = yout.signals.values;

% Compare results
figure;
subplot(2,1,1);
plot(t_sep, y_sep(:,3), 'b-', 'LineWidth', 2);
hold on;
plot(t_coup, y_coup(:,3), 'r--', 'LineWidth', 2);
legend('Separated', 'Coupled');
title('Pitch Rate Comparison');
grid on;

fprintf('Max difference: %.4e\n', max(abs(y_sep(:,3) - y_coup(:,3))));

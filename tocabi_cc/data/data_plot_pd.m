clc; clear; close all;

%% ===== Load log files =====
q_des   = load('joint_desired_log.txt');      % desired joint position
q       = load('joint_position_log.txt');     % actual joint position
qdot    = load('joint_velocity_log.txt');     % raw velocity
tau = load('torque_sum_log.txt');             % torque sum

%% ===== Time axis (control loop 기준) =====
N = length(q);
dt = 0.0005; 
t = (0:N-1) * dt;

%% ===== Plot =====
figure('Color','w','Position',[100 100 1400 400]);

% (1) Joint Desired vs Position
subplot(1,3,1);
plot(t, q_des, 'LineWidth', 2); hold on;
plot(t, q, 'LineWidth', 2);
grid on;
title('Joint Position Tracking');
xlabel('Time [s]');
ylabel('Position [rad]');
legend('q_{desired}', 'q', 'Location','best');

% (2) Joint Velocity
subplot(1,3,2);
plot(t, qdot, 'LineWidth', 1.5); hold on;
grid on;
title('Joint Velocity');
xlabel('Time [s]');
ylabel('Velocity [rad/s]');

% (3) Torque Sum
subplot(1,3,3);
plot(t, tau, 'LineWidth', 2);
grid on;
title('Torque Command (Torque Sum)');
xlabel('Time [s]');
ylabel('Torque [Nm]');

sgtitle('Low-Level Joint Control Debug Plot');
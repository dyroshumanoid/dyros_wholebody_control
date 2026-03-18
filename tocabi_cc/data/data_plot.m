clc; clear; close all;

%% ===== Load log files =====
base_dir = fileparts(mfilename('fullpath'))

min_distance_wo_cbf = importdata(fullfile(base_dir, 'wo_cbf', 'min_distance_log.txt'));
% hand_traj_wo_cbf    = importdata(fullfile(base_dir, 'wo_cbf', 'lhand_traj_log.txt'));

min_distance_issf   = importdata(fullfile(base_dir, 'issf', 'min_distance_log.txt'));
% hand_traj_issf      = importdata(fullfile(base_dir, 'issf', 'lhand_traj_log.txt'));

% min_distance_cbf   = importdata(fullfile(base_dir, 'cbf', 'min_distance_log.txt'));
% hand_traj_cbf      = importdata(fullfile(base_dir, 'cbf', 'lhand_traj_log.txt'));
% 
% min_distance_ecbf   = importdata(fullfile(base_dir, 'ecbf', 'min_distance_log.txt'));
% hand_traj_ecbf      = importdata(fullfile(base_dir, 'ecbf', 'lhand_traj_log.txt'));
%% ===== Time axis (control loop 기준) =====
N = 22000;
dt = 0.0005; 
t = (0:N-1) * dt;

% (4) min dist
figure()
plot(t, min_distance_wo_cbf(1:1:N, 15), 'LineWidth', 2);
hold on
plot(t, min_distance_issf(1:1:N, 15), 'LineWidth', 2);
% plot(t, min_distance_cbf(1:1:N, 3), 'LineWidth', 2);
% plot(t, min_distance_ecbf(1:1:N, 3), 'LineWidth', 2);
grid on;
title('h_{AB }');
xlabel('Time [s]');
ylabel('min dist [s]');
legend()
% 
% figure()
% plot(t, hand_traj_wo_cbf(1:1:N, 1), 'LineWidth', 2);
% hold on
% plot(t, hand_traj_issf(1:1:N, 1), 'LineWidth', 2);
% plot(t, hand_traj_cbf(1:1:N, 1), 'LineWidth', 2);
% plot(t, hand_traj_ecbf(1:1:N, 1), 'LineWidth', 2);
% 

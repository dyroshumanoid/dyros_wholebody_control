clc
clear all
close all

%% dt checker
microsecond_data = double(importdata("cc_calc_time_log.txt"));
hz = 1e6 ./ microsecond_data;
plot(hz)

%% joint pos
qpos = importdata("current_qpos_log.txt")
mpc_qpos = importdata("mpc_qpos_log.txt")

figure()
plot(mpc_qpos(:,1:12))
sgtitle("mpc qpos")
legend()

% figure()
hold on
plot(qpos(:,7:18))
sgtitle("current qpos log qpos")
legend()

%% joint torque
mpc_torque = importdata("mpc_torque_log.txt")

figure()
plot(mpc_torque(:,1:12))
sgtitle("mpc torque")
legend()


%%
clc
clear all
close all


data = readmatrix('torque_sum_log.txt');
torque_sol = data(:, 1:33);

data = readmatrix('torque_idn_log.txt');
torque_id = data(:, 1:33);

% 
data = readmatrix('torque_pd_log.txt');
torque_pd = data(:, 1:33);

figure()
for torque_cnt = 1:1:6
    sgtitle('JOINT TORQUE')
    plot(torque_sol(:,torque_cnt))    
    hold on 
    plot(torque_id(:,torque_cnt))
    plot(torque_pd(:,torque_cnt))
    legend('sum','idn','pd')
end
%%

% data = readmatrix('computation_time_log.txt');
% dt = data(:, 1);
% figure()
% plot(dt(:))
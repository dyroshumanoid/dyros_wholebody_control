clc
clear all
close all
%
clc
clear all
close all

data = readmatrix('dataWM1.txt');
left_foot_pos = data(:, 1:6);
data = readmatrix('dataWM2.txt');
right_foot_pos= data(:, 1:6);
data = readmatrix('dataWM3.txt');
com= data(:, 1:6);

data = readmatrix('dataWM4.txt');
cp= data(:, 1:4);


w = sqrt(9.81 / 0.73)

start_cnt = 1;

figure()
plot(left_foot_pos(:,start_cnt));
hold on
plot(left_foot_pos(:,start_cnt + 3));
plot(right_foot_pos(:,start_cnt));
plot(right_foot_pos(:,start_cnt + 3));
plot(com(:, start_cnt))
plot(com(:, start_cnt + 3))
plot(cp(:, start_cnt))
plot(cp(:, start_cnt + 2))
legend()

start_cnt = 2;
figure()
plot(left_foot_pos(:,start_cnt));
hold on
plot(left_foot_pos(:,start_cnt + 3));
plot(right_foot_pos(:,start_cnt));
plot(right_foot_pos(:,start_cnt + 3));
plot(com(:, start_cnt))
plot(com(:, start_cnt + 3))
plot(cp(:, start_cnt))
plot(cp(:, start_cnt + 2))
legend()

%%
% clc
% clear all
% close all

preview_tick = 1

data = readmatrix('dataWM5.txt');
com= data(:, 1:6);
data = readmatrix('dataWM6.txt');
lf= data(:, 1:6);

figure()
subplot(2,1,1)
plot(com(:,1));
hold on
plot(com(:,4));
plot(lf(:,1));
plot(lf(:,4));
legend('com tra', 'com cur', 'lfoot traj', 'lfoot pos')

subplot(2,1,2)
plot(com(:,2));
hold on
plot(com(:,5));
plot(lf(:,2));
plot(lf(:,5));
legend('com tra', 'com cur', 'lfoot traj', 'lfoot pos')
%%
data = readmatrix('contact_wrench_des.txt');
LF_des = data(:, 1:6);
RF_des = data(:, 7:12);

data = readmatrix('contact_wrench_ft.txt');
LF = -data(:, 1:6);
RF = -data(:, 7:12);

data = readmatrix('contact_wrench_id.txt');
LF_id = data(:, 1:6);
RF_id = data(:, 7:12);

figure()
plot(LF_id(:,3))
hold on
% plot(LF_des(:,3))
plot(RF_id(:,3))
% plot(RF_des(:,3))
legend('lf mea', 'lf des', 'rf mea', 'rf des')
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
sgtitle('TORQUE LEG')
% for cnt = 1:1:6
cnt = 3
% plot(torque_sol(:,cnt))    
hold on 
plot(torque_id(:,cnt))
% plot(torque_pd(:,cnt))
legend()
% end

% figure()
% sgtitle('TORQUE WAIST')
% for cnt = 13:1:15
%     plot(torque_sol(:,cnt))    
%     hold on
% end
% 
% figure()
% sgtitle('TORQUE UPPER')
% for cnt = 16:1:33
%     plot(torque_sol(:,cnt))    
%     hold on 
% end

%%
clc; clear all; close all;

preview_tick = 4

data = load('dataWM7.txt');
zmp_x = data(preview_tick, :);
data = load('dataWM8.txt');
zmp_y = data(preview_tick, :);

figure()
subplot(2,1,1)
plot(zmp_x);
subplot(2,1,2)
plot(zmp_y);
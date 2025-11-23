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

data = readmatrix('joint_pos_log.txt');
qpos = data(:, 1:33);

data = readmatrix('joint_pos_des_log.txt');
qpos_des = data(:, 1:33);

data = readmatrix('joint_vel_log.txt');
qvel = data(:, 1:33);

data = readmatrix('joint_vel_des_log.txt');
qvel_des = data(:, 1:33);

data = readmatrix('joint_acc_des_log.txt');
qacc_des = data(:, 1:39);

data = readmatrix('contact_wrench_log.txt');
contact_wrench = data(:, 1:12);

figure()
sgtitle('JOINT TORQUE')
torque_cnt = 15
plot(torque_sol(:,torque_cnt))    
hold on 
plot(torque_id(:,torque_cnt))
plot(torque_pd(:,torque_cnt))
legend()

pos_cnt = 6 + 15
figure()
sgtitle('JOINT QPOS')
plot(qpos(:,pos_cnt))    
hold on 
plot(qpos_des(:,pos_cnt))
legend()

figure()
sgtitle('JOINT QVEL')
plot(qvel(:,pos_cnt))    
hold on 
plot(qvel_des(:,pos_cnt))
legend()

figure()
sgtitle('JOINT QACC')
hold on 
plot(qacc_des(:,pos_cnt))
legend()

figure()
sgtitle('CONTACT WRENCH')
hold on 
plot(contact_wrench(:,:))
legend()
%%

clc
clear all
close all
data = readmatrix('cp_log.txt');
cp = data(:, 1:4);

data = readmatrix('com_log.txt');
com = data(:, 1:6);

data = readmatrix('zmp_log.txt');
zmp = data(:, 1:2);

figure()
sgtitle('X')
plot(cp(:,1))    
hold on 
plot(cp(:,3)) 
plot(com(:,1))
plot(com(:,4))
plot(zmp(:,1))
legend('cp mea', 'cp des', 'com pos', 'com v', 'zmp')

figure()
sgtitle('Y')
plot(cp(:,2))    
hold on 
plot(cp(:,4)) 
plot(com(:,2))
plot(com(:,5))
plot(zmp(:,2))
legend('cp mea', 'cp des', 'com pos', 'com v', 'zmp')
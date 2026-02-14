clc
clear all
close all

q_des = importdata("joint_desired_log.txt");
q_cur = importdata("joint_position_log.txt");
q_vel = importdata("joint_velocity_log.txt");
torque= importdata("torque_sum_log.txt");

figure()
sgtitle('pos')
plot(q_des);
hold on
plot(q_cur);
legend('q des', 'q cur')

figure()
sgtitle('torque')
plot(torque)

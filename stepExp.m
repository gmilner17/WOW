%% stepExp.m
% Group 11
% EENG 350
% Dr. Sager
% Spring 2022
%
% This script shows results from both the experimental and simulated open
% and closed loop step response experiments for the mini project.
%
% required file: miniProj.slx
%
%% Open loop step response
% import data from experiment
data = importdata('newDataStep.csv');
timeD = ((data(96:end, 1)) - 1021) / 1000;
vel = data(96:end, 2);
figure(1)
plot(timeD, vel, 'r')
hold on

% define parameters for simulated transfer function
k = (9.62) / 4;
sigma = 14.9;
num = k*sigma;
sys = tf(num,[1 sigma]);
step(sys * 4, 'b')
title('Open Loop Step Response');
xlabel('Time');
ylabel('Velocity (rad/s)');
legend('experimental', 'simulated');
%% Closed loop step response
%
% This simulation incorporates a PI controller. This controller can be
% tuned in Simulink to ensure a a rise time of 1 second and <12% overshoot.
%
open_system('miniProj');
%
% run the simulation
%
out=sim('miniProj');

% import experimental data
data2 = importdata('experiment.csv');

% plot the experimental and the simulated closed loop step response
% it can be shown that stiction causes a difference between the simulated
% and the experimental data, but overall matches reasonably to each other.

figure(2)
plot((data2(:, 1)) / 1000, data2(:, 2), 'r');
hold on
plot(out.theta, 'b')
title('Closed Loop Step Response');
xlabel('Time');
ylabel('Position (rad)');
legend('experimental', 'simulated');
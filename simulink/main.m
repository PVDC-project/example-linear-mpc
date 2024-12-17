%% Linear MPC formulation using YALMIP
clear;clc;close all;
addpath('functions')

% define the system dynamics and number of inputs
[A,B,C,D] = plant_model();  % discrete-time LTI system
nu = size(B,2);

% define the controller settings
Ts = 0.1;   % [s] sampling time
N = 10;     % [-] prediction horizon

% simulation
% initial and reference states
x0 = zeros(12,1);  % start at rest
x_ref = [0; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0];  % lift the quad 1 m

Tsim = 2;  % [s] simulation time

simout = sim('simulink_yalmip.slx');

% plot the results
figure

% plot the state(s) of interest
x_log = squeeze(simout.x_log);
subplot(2,1,1)
plot(x_log(3,:))  % third state should follow the reference
hold on
yline(x_ref(3),'r--')  % plot the reference
ylabel('x3')
ylim padded
xlim tight

% plot the control inputs
u_log = squeeze(simout.u_log);
u0 = 10.5916;  % hovering input
umin = [9.6; 9.6; 9.6; 9.6] - u0;
umax = [13; 13; 13; 13] - u0;
subplot(2,1,2)
stairs(u_log')
hold on
yline([umin(1) umax(1)],'r--')  % plot the input limits
legend({'u1','u2','u3','u4'})
ylabel('u')
xlabel('k')
ylim padded
xlim tight

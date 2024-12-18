%% Linear MPC formulation using YALMIP
clear;clc;close all;
addpath('functions')

% define the controller settings
Ts = 0.01;  % [s] sampling time
N = 10;     % [-] prediction horizon

% define the system dynamics and number of inputs
[A,B,C,D] = plant_model(0);  % continuous-time LTI system
[nx,nu] = size(B);

% simulation
x0 = zeros(nx,1);  % start at rest
Tsim = 1;  % [s] simulation time

% create a reference signal
ref_amplitude = 1;
switching_times = Tsim/4 * (1:4);  % switch four times during the simulation
t = 0:Ts:Tsim;
x_ref = ref_amplitude * ones(size(t));
for ii = 1:numel(switching_times)-1
    start_p = find(t > switching_times(ii), 1, 'first');
    end_p = find(t <= switching_times(ii+1),1,'last');
    x_ref(start_p:end_p) = x_ref(start_p:end_p) * (-1)^ii;
end
x_ref_sim = [t' x_ref'];  % format needed for Simulink

simout = sim('simulink_yalmip.slx');

% plot the results
figure

% plot the state(s) of interest
subplot(2,1,1)
x_log = squeeze(simout.x_log);
plot(t,C*x_log)  % plot the output
hold on
plot(t,x_ref,'r--')  % plot the reference
ylabel('$\omega$ [rad/s]','Interpreter','latex')
grid on
legend({'$\omega$','$\omega_{ref}$'},'Location','northeast','Interpreter','latex')
ylim padded
xlim tight
title('Simulation results')

% plot the control inputs
subplot(2,1,2)
u_log = squeeze(simout.u_log);
stairs(t,u_log)
hold on
yline([-6 6],'r--')  % plot the input limits
ylabel('$u_a$ [V]','Interpreter','latex')
xlabel('$t$ [s]','Interpreter','latex')
grid on
legend({'$u_a$','$u_{nom}$'},'Interpreter','latex')
ylim padded
xlim tight

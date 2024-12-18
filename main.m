%% Linear MPC formulation using YALMIP
clear;clc;close all;

%% Discrete-time model
% DC motor parameters
Ra = 2.19;      % [Ohm] armature resistance
La = 4.4e-3;    % [H] armature inductance
Cm = 1.5872;    % [Nm/A] electromechanical constant
Ce = Cm;        % [Vs/rad] back-EMF constant
J = 0.0345;     % [kgm^2] rotor moment of inertia

% continuous-time LTI system
% form: dx/dt = Ac*x + Bc*u
% x = [ia w]', u = ua, y = w
% (ia - armature current)
% (w - motor speed)
% (ua - armature voltage)
Ac = [-Ra/La -Ce/La;
       Cm/J     0];
Bc = [1/La
       0];
C = [0 1];
D = 0;
sysc = ss(Ac,Bc,C,D);

% discrete-time LTI system
% form: x+ = A*x + B*u
Ts = 0.01;              % [s] sampling time
sysd = c2d(sysc,Ts);    % discretization using ZOH
A = sysd.A; B = sysd.B; % discrete-time matrices (C and D stay the same)

% define system dimensions
[nx, nu] = size(B);
ny = size(C,1);

%% MPC parameters
% objective function matrices
Q = diag([0 1e3]);      % we only care about tracking the speed in this case
R = 1;                  % input cost should be positive
[~,QN] = dlqr(A,B,Q,R); % LQR solution as the terminal cost

% input constraints
Unom = 6;  % [V] nominal voltage
umin = -Unom;
umax = Unom;

% state constraints (optional)
xmin = [-10; -10];
xmax = [10; 10];

% prediction horizon
N = 10;  % steps, not seconds

%% Define the optimization problem for YALMIP
u = sdpvar(nu,N);   % input vector
x = sdpvar(nx,N+1); % state vector
x0 = sdpvar(nx,1);  % initial state, will be set online
xr = sdpvar(nx,1);  % output reference, will be set online

% objective
objective = 0;
for k = 1 : N
    objective = objective + (x(:,k)-xr)' * Q * (x(:,k)-xr);     % state cost
    objective = objective + u(:,k)' * R * u(:,k);               % input cost
end
objective = objective + (x(:,N+1)-xr)' * QN * (x(:,N+1)-xr);    % terminal cost

% constraints
constraints = [];
constraints = [constraints, x(:,1) == x0];  % initial state constraint
for k = 1 : N
    constraints = [constraints, x(:,k+1) == A*x(:,k) + B*u(:,k)];   % state dynamics
    constraints = [constraints, umin <= u(:,k) <= umax];            % input constraints
    constraints = [constraints, xmin <= x(:,k+1) <= xmax];          % state constraints
end

% solver options
options = sdpsettings();
options.verbose = 0;  % supress printing

% define input and output data
input_data = {x0,xr};   % the solver requires an initial state and the reference
output_data = u(:,1);   % the solver returns the first optimal input

% define a controller object
controller = optimizer(constraints, objective, options, input_data, output_data);

%% Simulate in closed loop
% simulation setup
Tsim = 1;       % [s] simulation length
nsim = Tsim/Ts; % number of simulation steps

% create a reference signal
ref_amplitude = 1;
switching_times = Tsim/4 * (1:4);  % switch four times during the simulation
t = 0:Ts:Tsim;
xr = ref_amplitude * ones(size(t));
for ii = 1:numel(switching_times)-1
    start_p = find(t > switching_times(ii), 1, 'first');
    end_p = find(t <= switching_times(ii+1),1,'last');
    xr(start_p:end_p) = xr(start_p:end_p) * (-1)^ii;
end

% preallocate logs
x_log = nan(nx,nsim+1);
u_log = nan(nu,nsim);

% initial state
x = zeros(nx,1);  % start at rest
x_log(:,1) = x;

% run the closed-loop simulation
for k = 1:nsim
    x_ref = [0; xr(k)];  % add a zero as the armature current reference
    u_opt = controller({x,x_ref});  % get the control input

    x = lsim(sysc,[u_opt u_opt],[0 Ts],x);  % simulate one timestep
    
    x_log(:,k+1) = x;       % log the states
    u_log(:,k) = u_opt;     % log the inputs
end

%% Plot the results
figure

% plot the state(s) of interest
subplot(2,1,1)
plot(t,C*x_log)  % plot the output
hold on
plot(t,xr,'r--')  % plot the reference
ylabel('$\omega$ [rad/s]','Interpreter','latex')
grid on
legend({'$\omega$','$\omega_{ref}$'},'Location','northeast','Interpreter','latex')
ylim padded
xlim tight
title('Simulation results')

% plot the control inputs
subplot(2,1,2)
stairs(t,[u_log u_log(end)])
hold on
yline([umin umax],'r--')  % plot the input limits
ylabel('$u_a$ [V]','Interpreter','latex')
xlabel('$t$ [s]','Interpreter','latex')
grid on
legend({'$u_a$','$u_{nom}$'},'Interpreter','latex')
ylim padded
xlim tight

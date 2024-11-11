%% Linear MPC formulation using YALMIP
clear;clc;close all;

%% Discrete time model
% form: x+ = A*x + B*u
% in this case, a quadcopter 
% more information: https://github.com/orgs/osqp/discussions/558
A = [1       0       0   0   0   0   0.1     0       0    0       0       0;
      0       1       0   0   0   0   0       0.1     0    0       0       0;
      0       0       1   0   0   0   0       0       0.1  0       0       0;
      0.0488  0       0   1   0   0   0.0016  0       0    0.0992  0       0;
      0      -0.0488  0   0   1   0   0      -0.0016  0    0       0.0992  0;
      0       0       0   0   0   1   0       0       0    0       0       0.0992;
      0       0       0   0   0   0   1       0       0    0       0       0;
      0       0       0   0   0   0   0       1       0    0       0       0;
      0       0       0   0   0   0   0       0       1    0       0       0;
      0.9734  0       0   0   0   0   0.0488  0       0    0.9846  0       0;
      0      -0.9734  0   0   0   0   0      -0.0488  0    0       0.9846  0;
      0       0       0   0   0   0   0       0       0    0       0       0.9846];
B = [0      -0.0726  0       0.0726;
     -0.0726  0       0.0726  0;
     -0.0152  0.0152 -0.0152  0.0152;
      0      -0.0006 -0.0000  0.0006;
      0.0006  0      -0.0006  0;
      0.0106  0.0106  0.0106  0.0106;
      0      -1.4512  0       1.4512;
     -1.4512  0       1.4512  0;
     -0.3049  0.3049 -0.3049  0.3049;
      0      -0.0236  0       0.0236;
      0.0236  0      -0.0236  0;
      0.2107  0.2107  0.2107  0.2107];
[nx, nu] = size(B);  % system dimensions

%% MPC parameters
% details: https://osqp.org/docs/examples/mpc.html

% objective function matrices
Q = diag([0 0 10 10 10 10 0 0 0 5 5 5]);
R = 0.1*eye(4);
[~,QN] = dlqr(A,B,Q,R);  % LQR solution as the terminal cost

% input constraints
u0 = 10.5916;  % hovering input
umin = [9.6; 9.6; 9.6; 9.6] - u0;
umax = [13; 13; 13; 13] - u0;

% state constraints (only on some states)
xmin = [-pi/6; -pi/6; -Inf; -Inf; -Inf; -1; -Inf(6,1)];
xmax = [ pi/6;  pi/6;  Inf;  Inf;  Inf; Inf; Inf(6,1)];

% prediction horizon
N = 10;  % steps, not seconds

%% Define the optimization problem for YALMIP
u = sdpvar(nu,N);
x = sdpvar(nx,N+1);
x0 = sdpvar(nx,1);  % initial state, will be set online
xr = sdpvar(nx,1);  % reference state, will be set online

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
options.verbose = 1;  % print some information

% define input and output data
input_data = {x0,xr};   % the solver requires an initial state and the reference
output_data = u(:,1);   % the solver returns the first optimal input

% define a controller object
controller = optimizer(constraints, objective, options, input_data, output_data);

%% Simulate in closed loop

% initial and reference states
x0 = zeros(12,1);  % start at rest
xr = [0; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0];  % lift the quad 1 m

nsim = 15;  % number of simulation steps

x_log = nan(nx,nsim+1);     % state logging
u_log = nan(nu,nsim);       % input logging

% run the simulation
x = x0;
x_log(:,1) = x;

for k = 1 : nsim
    u_opt = controller({x,xr});  % get the control input
    x = A*x + B*u_opt;      % simulate one timestep
    
    x_log(:,k+1) = x;       % log the states
    u_log(:,k) = u_opt;     % log the inputs
end

%% Plot the results
figure

% plot the state(s) of interest
subplot(2,1,1)
plot(x_log(3,:))  % third state should follow the reference
hold on
yline(xr(3),'r--')  % plot the reference
ylabel('x3')
ylim padded
xlim tight

% plot the control inputs
subplot(2,1,2)
stairs(u_log')
hold on
yline([umin(1) umax(1)],'r--')  % plot the input limits
legend({'u1','u2','u3','u4'})
ylabel('u')
xlabel('k')
ylim padded
xlim tight

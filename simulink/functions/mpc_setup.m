function controller = mpc_setup(N)
% This function creates an MPC object using YALMIP 
% input: prediction horizon

% ------------- Discrete-time model --------------
% form: x+ = A*x + B*u
[A,B] = plant_model();
[nx, nu] = size(B);  % system dimensions

% ------------- MPC parameters -------------------
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

% ------- Optimization problem definition --------
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
output_data = u(:,1);   % the solver returns the optimal input

% define a controller object
controller = optimizer(constraints, objective, options, input_data, output_data);
end

function [A,B,C,D] = plant_model(Ts)
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

if Ts > 0
    % discrete-time LTI system
    % form: x+ = A*x + B*u
    sysd = c2d(sysc,Ts);    % discretization using ZOH
    A = sysd.A; B = sysd.B; % discrete-time matrices (C and D stay the same)
else
    % output continuous-time matrices
    A = Ac; B = Bc;
end
end

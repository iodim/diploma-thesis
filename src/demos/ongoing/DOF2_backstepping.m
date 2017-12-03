clc; clear; close all;

%% Parameters
n = 4;
x0 = zeros(n, 1);
z0 = zeros(n, 1);
q0 = [x0; z0];

% Plant parameters
m1 = 0.5;
m2 = 0.2;
l1 = 0.3;
l2 = 0.25;
g = 9.81;
plant = @(t, x, u) manipulator_2dof(t, x, u, [m1 m2 l1 l2 g]);

% Reference trajectories
% yd = @(t) [sin(2*t) + 0.3*cos(5*t);
% 	        0.5*sin(3*t) + 2*cos(t)];
yd = @(t) [cos(2*t);
           sin(3*t)];

% Solver parameters
tmax = 15;
ode_options = odeset('AbsTol', 1e-14, 'RelTol', 1e-9, ...
                     'OutputFcn', @odeprog,'Events', @odeabort);

% PPC parameters
k = [1 1];
tol = 1e-2;
rbar = 3e0;
rho0 = 3;
rho = @(t) (rho0 - tol)*exp(-rbar*t) + tol;

%% State feedback
A = [1 2; 3 4];
controller = @(t, x, yd, w) ppc_backstepping(t, x, yd, rho, k);
MIMO_controller = @(t, x, w) MIMO_controller(t, x, A, controller, yd, w);
sys1 = @(t, q) control_loop(t, q, plant, [n 0 0], MIMO_controller);

[t, q] = ode15s(sys1, [0 tmax], x0, ode_options);

% Reconstruct state estimates
q1 = q(:, A(1, :));
q2 = q(:, A(2, :));
y = yd(t')';

% Plots
plotter('t', t, 'x', q1);
suptitle('Link 1 states');

plotter('t', t, 'x', q1(:, 1) - y(:, 1), 'rho', rho);
suptitle('Link 1 tracking error');

plotter('t', t, 'x', q2);
suptitle('Link 2 states');

plotter('t', t, 'x', q2(:, 1) - y(:, 2), 'rho', rho);
suptitle('Link 2 tracking error');

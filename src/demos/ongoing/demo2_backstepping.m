clc; clear; close all;

%% Parameters
n = 2;
x0 = ones(n, 1);
z0 = zeros(n, 1);
q0 = [x0; z0];

plant = @plant2;
yd = @(t) sin(2*t) + 0.3*cos(5*t);

tmax = 15;
ode_options = odeset('AbsTol', 1e-14, 'RelTol', 1e-9);%, ...
%                      'OutputFcn', @odeprog,'Events', @odeabort);


% HGO parameters
mu = 1e-3;
alpha = poly([-4 -5]);
alpha = alpha(2:end);

% PPC parameters
k = [4, 5];
tol = 1e-2;
rbar = 3e0;
rho0 = 5;
rho = @(t) (rho0 - tol)*exp(-rbar*t) + tol;

%% BME
e = 5;
bme = @(t, x) [x(1); 0.01*x(2)];
controller = @(t, x, w) ppc_backstepping(t, bme(t, x), yd, rho, k);
sys1 = @(t, q) control_loop(t, q, plant, [n 0 0], controller);

[t, q] = ode15s(sys1, [0 tmax], x0, ode_options);

% Reconstruct state estimates
x = q(:, 1:n);
xi1 = (x(:, 1) - yd(t))./rho(t);
eps1 = log((1+xi1)./(1-xi1))./(1-xi1.^2);
xi2 = (x(:, 2) + k(1)*eps1)./rho(t);
eps2 = log((1+xi2)./(1-xi2))./(1-xi2.^2);
u = -k(2)*eps2;

% Plots
plotter('t', t, 'x', x);
plotter('t', t, 's', x(:, 1) - yd(t), 'rho', rho)
plotter('t', t, 's', u);

%% Output Feedback
% satlvl = 25;
% observer = @(t, xhat, y) hgo(t, xhat, y, alpha, mu);
% controller = @(t, x, w) ppc_backstepping(t, x, yd, rho, k);
% sat_controller = @(t, x, w) sat_control(t, x, controller, satlvl);
% sys1 = @(t, q) control_loop(t, q, plant, [n 0 n], sat_controller, observer);
% 
% [t, q] = ode15s(sys1, [0 tmax], q0, ode_options);
% 
% % Reconstruct state estimates
% x = q(:, 1:n);
% xhat = q(:, n+1:end);
% e = x - xhat;
% xi1 = (x(:, 1) - yd(t))./rho(t);
% eps1 = log((1+xi1)./(1-xi1))./(1-xi1.^2);
% xi2 = (xhat(:, 2) + k(1)*eps1)./rho(t);
% eps2 = log((1+xi2)./(1-xi2))./(1-xi2.^2);
% u = sat(-k(2)*eps2, satlvl);
% 
% % Plots
% plotter('t', t, 'x', x, 'xhat', xhat);
% plotter('t', t, 's', x(:, 1) - yd(t), 'rho', rho)
% plotter('t', t, 's', xhat(:, 2) + k(1)*eps1, 'rho', rho)
% plotter('t', t, 's', u);

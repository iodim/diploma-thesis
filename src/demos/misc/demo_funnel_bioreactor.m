clc; clear; close all;

%% Parameters
n = 2;
x0 = [0.075; 0.03];
z0 = [0; 0];
q0 = [x0; z0];

% Funnel parameters
p = [1, 1/11];
q = [1, 0.2];
Gamma = 0;
phi = @(t) 0.5*exp(-t) + 100/pi*atan(t);

% PPD parameters
k = 50;
tol = 1e-3;
rbar = 1e1;
rho0 = 10;
tau = -1/rbar*log(1/((rho0-tol)*rbar));
rho = create_rho(rho0, tau, tol);


% Simulation paramaters
plant = @bioreactor;

tmax = 70;
ode_options = odeset('AbsTol', 1e-14, 'RelTol', 1e-10);%, ...
%                      'OutputFcn', @odeprog,'Events', @odeabort);

%% Funnel simulation
controller = @(t, x, w) nocontrol();
observer = @(t, z, y) funnel2(t, z, y, p, q, Gamma, phi);
sys = @(t, q) control_loop(t, q, plant, [n 0 n], controller, observer);

[t, q] = ode15s(sys, [0 tmax], q0, ode_options);

% Reconstructing state estimates
x = q(:, 1:n);
xhat = q(:, n+1:end);

% Plots
plotter('t', t, 'x', x, 'xhat', xhat, 'rho', phi);

%% PPD simulation
% controller = @(t, x, w) nocontrol();
% peak = tau;
% observer = @(t, z, y) ppd(t, z, y, rho, k, peak);
% sys = @(t, q) control_loop(t, q, plant, [n 0 n], controller, observer);
% 
% [t, q] = ode15s(sys, [0 tmax], q0, ode_options);
% 
% % Reconstructing state estimates
% x = q(:, 1:n);
% xhat = q(:, n+1:end);
% 
% % Plots
% plotter('t', t, 'x', x, 'xhat', xhat);
% plotter('t', t, 's', x-xhat, 'rho', @(t) rho(t-tau));

clc; clear; close all;

%% Parameters
n = 1;
x0 = ones(n, 1);
z0 = zeros(n, 1);
q0 = [x0; z0];

k = 500;

tol = 1e-6;
rbar = 1e3;
rho0 = 3;
rho = @(t) (rho0 - tol)*exp(-rbar*t) + tol;

plant = @plant1b;

tmax = 25;
ode_options = odeset('AbsTol', 1e-9, 'RelTol', 1e-6, ...
                     'OutputFcn', @odeprog,'Events', @odeabort);

%% Prescribed perfomance differentiator
peak = 5/rbar;
observer = @(t, z, y) ppd(t, z, y, rho, k, peak);
controller = @(t, x, w) nocontrol();
sys1 = @(t, q) control_loop(t, q, plant, [n 0 n], controller, observer);

[t, q] = ode15s(sys1, [0 tmax], q0, ode_options);
t_p = (t < peak);

% Reconstruct state estimates
x = q(:, 1:n);
z = q(:, n+1:end);

xhat = z;

% Reconstruct sliding surface, its estimate and the control input
s = x - xhat;

plotter('t', t, 'x', x, 'xhat', xhat);
% save2tikz('../tex/reports/ppd/figures/ppd1b.tikz');

plotter('t', t(t_p), 'x', x(t_p), 'xhat', xhat(t_p));
% save2tikz('../tex/reports/ppd/figures/ppd1b_zoom.tikz');

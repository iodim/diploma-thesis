clc; clear; close all;

%% Parameters
n = 2;
x0 = [1; 0];
z0 = zeros(n, 1);
q0 = [x0; z0];

k = 200;

tol = 1e-3;
rbar = 1e2;
rho0 = 10;
rho = @(t) (rho0 - tol)*exp(-rbar*t) + tol;

plant = @plant2b;

tmax = 25;
ode_options = odeset('AbsTol', 1e-9, 'RelTol', 1e-6, ...
                     'OutputFcn', @odeprog,'Events', @odeabort);

%% Prescribed perfomance differentiator
peak = 5/rbar;
observer = @(t, z, y) ppd(t, z, y, rho, k, peak);
controller = @(t, x, w) nocontrol();
sys1 = @(t, q) control_loop(t, q, plant, [n 0 n], controller, observer);

[t, q] = ode15s(sys1, [0 tmax], q0, ode_options);
t_p = (t < n*peak);

% Reconstruct state estimates
x = q(:, 1:n);
z = q(:, n+1:end);

xhat = z;

% Reconstruct sliding surface, its estimate and the control input
s = x - xhat;

plotter('t', t, 'x', x, 'xhat', xhat);
% save2tikz('../tex/reports/ppd/figures/ppd2b.tikz');

plotter('t', t(t_p), 'x', x(t_p, :), 'xhat', xhat(t_p, :));
% save2tikz('../tex/reports/ppd/figures/ppd2b_zoom.tikz');

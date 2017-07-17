clc; clear; close all;

%% Parameters
n = 4;
x0 = ones(n, 1);
z0 = zeros(n, 1);
q0 = [x0; z0];

k = 15;

tol = 1e-1;
rbar = 10;
rho0 = 10;
rho = @(t) (rho0 - tol)*exp(-rbar*t) + tol;

plant = @plant4b;

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

plotter('t', t(1:10:numel(t)), 'x', x(1:10:numel(t), :), 'xhat', xhat(1:10:numel(t), :));
cleanfigure;
save2tikz('../tex/reports/ppd/figures/ppd4b.tikz');

plotter('t', t(t_p), 'x', x(t_p, :), 'xhat', xhat(t_p, :));
save2tikz('../tex/reports/ppd/figures/ppd4b_zoom.tikz');

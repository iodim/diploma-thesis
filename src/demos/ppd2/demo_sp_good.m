clc; clear; close all;

%% Parameters
n = 2;
x0 = ones(n, 1);
z0 = zeros(n, 1);
q0 = [x0; z0];

%% PPD parameters
k_o = 200;
rho_inf_o = 1e-2;
rbar_o = 2e2;
rho_0_o = 10;
rho_o = @(t) (rho_0_o - rho_inf_o)*exp(-rbar_o*t) + rho_inf_o;

%% PPC parameters
r = 2;
Lambda = fliplr(poly(-r))';
k_c = 50;
rho_inf_c = 1e-1;
rbar_c = 1;
rho_0_c = 10;
rho_c = @(t) (rho_0_c - rho_inf_c)*exp(-rbar_c*t) + rho_inf_c;

plant = @plant2;

tmax = 5;
ode_options = odeset('AbsTol', 1e-12, 'RelTol', 1e-9, ...
                     'OutputFcn', @odeprog,'Events', @odeabort);

%% Prescribed perfomance differentiator
peak = -1/rbar_o*log(1/((rho_0_o-rho_inf_o)*rbar_o));
observer = @(t, z, y) ppd(t, z, y, rho_o, k_o, peak);
controller = @(t, x, w) ppc(t, x, Lambda, rho_c, k_c);
sys1 = @(t, q) control_loop(t, q, plant, [n 0 n], controller, observer);

[t, q] = ode15s(sys1, [0 tmax], q0, ode_options);
t_p = (t < 2*n*peak);

% Reconstruct state estimates
x = q(:, 1:n);
z = q(:, n+1:end);

xhat = z;

% Reconstruct sliding surface, its estimate and the control input
s = x*Lambda;
shat = xhat*Lambda;

plotter('t', t(1:5:end), 'x', x(1:5:end, :), 'xhat', xhat(1:5:end, :), 'peak', n*peak);
% save2tikz('../tex/reports/ppd2/figures/ppd2a.tikz');

plotter('t', t(1:5:end), 's', s(1:5:end, :), 'shat', shat(1:5:end, :), 'rho', rho_c, 'peak', n*peak);
% save2tikz('../tex/reports/ppd2/figures/ppd2a_zoom.tikz');

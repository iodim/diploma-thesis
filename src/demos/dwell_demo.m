%DWELL_DEMO Performs a simulation demo for a dwell time PPC.
   
%   Ioannis Dimanidis (2017)
clc; clear; close all;

%% Parameters
n = 4;
x0 = ones(n, 1);
z0 = zeros(n,1);
q0 = [x0; z0];

k = 12;
satlvl = inf;
mu = 0.001;
alpha = [2, 1, 1, 1];
% M = [1.6, 1.3, 3];
M =  [10, 10, 10];

r = [1.1, 1.2, 1.3];
Lambda = fliplr(poly(-r))';

rho = @(t) (20 - 0.05)*exp(-1*t) + 0.05;

plant = @plant4b;

td = 0.04;

tmax = 15;
ode_options = odeset('AbsTol', 1e-9, 'RelTol', 1e-6);

%% Cascade High-Gain Observer
peak = 0.05;
observer = @(t, z, y) chgo(t, z, y, alpha, mu, M);
controller = @(t, x, w) ppc_sat(t, x, Lambda, rho, k, satlvl);
dwell_controller = @(t, x, w) dwell_time_controller(t, x, controller, td);
sys1 = @(t, q) control_loop(t, q, plant, [n 0 n], dwell_controller, observer);

[t, q] = ode15s(sys1, [0 tmax], q0, ode_options);
t_p = (t < peak);

% Reconstruct state estimates
x = q(:, 1:n);
z = q(:, n+1:end);

xhat = zeros(size(x));
xhat(:, 1) = z(:, 1);
xhat(:, 2) = sat(z(:, 2)/mu, M(1));

for i = 3:n
   xhat(:, i) = sat(alpha(i)/mu*(z(:, i) + xhat(:, i-1)), M(i-1));
end

% Reconstruct sliding surface, its estimate and the control input
s = x*Lambda;
shat = xhat*Lambda;
u = dwell_controller(t, xhat');

plotter('t', t, 'x', x, 's', s, 'rho', rho, 'u', u, 'xhat', xhat, ...
        'shat', shat, 'peak', peak);


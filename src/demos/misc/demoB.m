%DEMOB Performs a simulation demo for plant4b using PPC and CHGO.
%
%   See more:
%   Bechlioulis, Charalampos P., Achilles Theodorakopoulos, and George A. 
%   Rovithakis. "Output feedback stabilization with prescribed performance 
%   for uncertain nonlinear systems in canonical form." Decision and Control
%   (CDC), 2013 IEEE 52nd Annual Conference on. IEEE, 2013.
   
%   Ioannis Dimanidis (2017)
clc; clear; close all;

%% Parameters
n = 4;
x0 = ones(n, 1);
z0 = zeros(n,1);
q0 = [x0; z0];

k = 12;
mu = 0.001;
alpha = [2, 1, 1, 1];
% M = [1.6, 1.3, 3];
M = [2, 2, 2];

r = [1.1, 1.2, 1.3];
Lambda = fliplr(poly(-r))';

rho = @(t) (20 - 0.05)*exp(-1*t) + 0.05;

plant = @plant4b;

tmax = 15;
ode_options = odeset('AbsTol', 1e-9, 'RelTol', 1e-6);

%% Cascade High-Gain Observer
peak = 0.05;
observer = @(t, z, y) chgo(t, z, y, alpha, mu, M);
controller = @(t, x, w) ppc_surf(t, x, Lambda, rho, k);
sys1 = @(t, q) control_loop(t, q, plant, [n 0 n], controller, observer);

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
u = controller(t, xhat');

plotter('t', t, 'x', x, 's', s, 'rho', rho, 'u', u, 'xhat', xhat, ...
        'shat', shat, 'peak', peak);

%% Regular High-Gain Observer
peak = 0.05;
satlvl = 15;
alpha = [4, 6, 4, 1];

observer = @(t, xhat, y) hgo(t, xhat, y, alpha, mu);
controller = @(t, x, w) ppc_surf(t, x, Lambda, rho, k);
sat_controller = @(t, x, w) sat_control(t, x, controller, satlvl);
sys2 = @(t, q) control_loop(t, q, plant, [n 0 n], sat_controller, observer);

[t, q] = ode15s(sys2, [0 tmax], q0, ode_options);
t_p = (t < peak);

% Reconstruct sliding surface, its estimate and the control input
x = q(:, 1:n);
xhat = q(:, n+1:end);

s = x*Lambda;
shat = xhat*Lambda;
u = zeros(size(t));
for i = 1:length(t)
    u(i) = sat_controller(t(i), xhat(i, :)');
end

plotter('t', t, 'x', x, 's', s, 'rho', rho, 'u', u, 'xhat', xhat, ...
        'shat', shat, 'peak', peak);

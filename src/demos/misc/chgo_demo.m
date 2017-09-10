%CHGO_DEMO Performs a simulation demo of a CHGO estimating a plant's states.
%
%   See more (Example 1):
%   Khalil, Hassan K. "Cascade high-gain observers in output feedback 
%   control." Automatica 80 (2017): 110-118.
   
%   Ioannis Dimanidis (2017)

clc; clear; close all;

%% Parameters
n = 4;
mu = 0.001;
M = [2, 1.5, 3];
alpha = [2, 1, 1, 1];

x0 = ones(n, 1);
z0 = zeros(n,1);
q0 = [x0; z0];

plant = @plant4b;
observer = @(t, z, y) chgo(t, z, y, alpha, mu, M);
controller = @(t, x, w) nocontrol();
sys = @(t, q) control_loop(t, q, plant, [n 0 n], controller, observer);

tmax = 0.15;
ode_options = odeset('AbsTol', 1e-9, 'RelTol', 1e-6);

%% Simulation
[t, q] = ode15s(sys, [0 tmax], q0, ode_options);

% Reconstruct state estimates
x = q(:, 1:n);
z = q(:, n+1:end);

xhat = zeros(size(x));
xhat(:, 1) = z(:, 1);
xhat(:, 2) = sat(z(:, 2)/mu, M(1));

for i = 3:n
   xhat(:, i) = sat(alpha(i)/mu*(z(:, i) + xhat(:, i-1)), M(i-1));
end

plotter('t', t, 'x', x, 'xhat', xhat) 

%   Ioannis Dimanidis (2017)
clc; clear; close all;

%% Parameters
n = 4;
x0 = ones(n, 1);
z0 = zeros(n,1);
q0 = [x0; z0];

k = 4;
mu = 0.001;
alpha = [4, 6, 4, 1];

% r = [1.1, 1.2, 1.3];
r = [2 2 2];
Lambda = fliplr(poly(-r))';

rho = @(t) (30 - 0.05)*exp(-2*t) + 0.05;

plant = @plant4c;

tmax = 10;
peak = 0.05;
ode_options = odeset('AbsTol', 1e-9, 'RelTol', 1e-6);

%% Full state feedback
controller = @(t, x, w) ppc_surf(t, x, Lambda, rho, k);
sys1 = @(t, q) control_loop(t, q, plant, [n 0 0], controller);

[t, q] = ode15s(sys1, [0 tmax], x0, ode_options);

% Reconstruct sliding surface, its estimate, and the control input
x = q(:, 1:n);

s = x*Lambda;
u = controller(t, x');

plotter('t', t, 'x', x, 's', s, 'rho', rho, 'u', u);

%% Output feedback (PPC-Sat w/ HGO)
satlvl = 46.61;
observer = @(t, xhat, y) hgo(t, xhat, y, alpha, mu);
controller = @(t, x, w) ppc_surf(t, x, Lambda, rho, k);
sat_controller = @(t, x, w) sat_control(t, x, controller, satlvl);
sys1 = @(t, q) control_loop(t, q, plant, [n 0 n], sat_controller, observer);

[t, q] = ode15s(sys1, [0 tmax], q0, ode_options);

% Reconstruct sliding surface, its estimate, and the control input
x = q(:, 1:n);
xhat = q(:, n+1:end);

s = x*Lambda;
shat = xhat*Lambda;
u = zeros(size(t));
for i = 1:length(t)
    u(i) = sat_controller(t(i), xhat(i, :)');
end

if any(find(abs(s./rho(t)) >= 1))
    fprintf('PPC-Sat w/ HGO:\n\t Surface violated perfomance.\n');
else
    fprintf('PPC-Sat w/ HGO:\n\t Surface did not violate perfomance.\n');
end

plotter('t', t, 'x', x, 's', s, 'rho', rho, 'u', u, 'xhat', xhat, ...
        'shat', shat, 'peak', peak);

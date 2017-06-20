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
peaking_time = 0.05;
ode_options = odeset('AbsTol', 1e-9, 'RelTol', 1e-6);

%% Full state feedback
controller = @(t, x, w) ppc(t, x, Lambda, rho, k);
sys1 = @(t, q) control_loop(t, q, plant, [n 0 0], controller);

[t, q] = ode15s(sys1, [0 tmax], x0, ode_options);

% Reconstruct sliding surface, its estimate, and the control input
x = q(:, 1:n);

s = x*Lambda;
u = controller(t, x');

plotter4(t, x, s, rho, u);

%% Output feedback (PPC-Sigma w/ HGO)
satlvl = 1 - 1e-4;
observer = @(t, xhat, y) hgo(t, xhat, y, alpha, mu);
controller = @(t, x, w) ppc_sigma(t, x, Lambda, rho, k, satlvl);
sys1 = @(t, q) control_loop(t, q, plant, [n 0 n], controller, observer);

[t, q] = ode15s(sys1, [0 tmax], q0, ode_options);
t_p = (t < peaking_time);

% Reconstruct sliding surface, its estimate, and the control input
x = q(:, 1:n);
xhat = q(:, n+1:end);

s = x*Lambda;
shat = xhat*Lambda;
u = controller(t, xhat');

plotter4(t, x, s, rho, u, xhat, shat, t_p);
if any(find(abs(s./rho(t)) >= 1))
    fprintf('PPC-Sigma w/ HGO:\n\t Surface violated perfomance.\n');
else
    fprintf('PPC-Sigma w/ HGO:\n\t Surface did not violate perfomance.\n');
end

%% Output feedback (PPC-Sat w/ HGO)
satlvl = 46.61;
observer = @(t, xhat, y) hgo(t, xhat, y, alpha, mu);
controller = @(t, x, w) ppc_sat(t, x, Lambda, rho, k, satlvl);
sys1 = @(t, q) control_loop(t, q, plant, [n 0 n], controller, observer);

[t, q] = ode15s(sys1, [0 tmax], q0, ode_options);
t_p = (t < peaking_time);

% Reconstruct sliding surface, its estimate, and the control input
x = q(:, 1:n);
xhat = q(:, n+1:end);

s = x*Lambda;
shat = xhat*Lambda;
u = controller(t, xhat');

plotter4(t, x, s, rho, u, xhat, shat, t_p);
if any(find(abs(s./rho(t)) >= 1))
    fprintf('PPC-Sat w/ HGO:\n\t Surface violated perfomance.\n');
else
    fprintf('PPC-Sat w/ HGO:\n\t Surface did not violate perfomance.\n');
end


close(figure(1));
close(figure(3));
close(figure(5));

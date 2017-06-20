%DEMOTEST
clc; clear; close all;

n = 2;
k = 1;
satlvl = 25;
mu = 0.001;
x0 = [1; 1];
z0 = [0; 0];
alpha = [10, 25];

r = 6;
Lambda = fliplr(poly(-r))';

rho = @(t) (10 - 0.01)*exp(-5*t) + 0.01;

tmax = 2;
peaking_time = 0.05;
ode_options = odeset('AbsTol', 1e-9, 'RelTol', 1e-6);

%% PPC-Sat (w/ HGO)
plant = @plant2;
controller = @(t, x, w) ppc_sat(t, x, Lambda, rho, k, satlvl);
observer = @(t, xhat, y) hgo(t, xhat, y, alpha, mu);
system = @(t, q) control_loop(t, q, plant, [n 0 n], controller, observer);

[t, q] = ode15s(system, [0 tmax], [x0; z0], ode_options);

x = q(:, 1:n);
xhat = q(:, n+1:end);

s = x*Lambda;
shat = xhat*Lambda;
u = controller(t, xhat');

if any(find(abs(s./rho(t)) >= 1))
    fprintf('PPC-Sat w/ HGO:\n\t Surface violated perfomance.\n');
else
    fprintf('PPC-Sat w/ HGO:\n\t Surface did not violate perfomance.\n');
end

plotter2(t, x, s, rho, u, xhat, shat)

%% PPC-Sigma (w/ HGO)
satlvl = 1 - 1e-8;
controller = @(t, x, w) ppc_sigma(t, x, Lambda, rho, k, satlvl);
system = @(t, q) control_loop(t, q, plant, [n 0 n], controller, observer);

[t, q] = ode15s(system, [0 tmax], [x0; z0], ode_options);

x = q(:, 1:n);
xhat = q(:, n+1:end);

s = x*Lambda;
shat = xhat*Lambda;
u = controller(t, xhat');

if any(find(abs(s./rho(t)) >= 1))
    fprintf('PPC-Sigma w/ HGO:\n\t Surface violated perfomance.\n');
else
    fprintf('PPC-Sigma w/ HGO:\n\t Surface did not violate perfomance.\n');
end

plotter2(t, x, s, rho, u, xhat, shat)

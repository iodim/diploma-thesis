clc; clear; close all;

%% Parameters
n = 4;
x0 = ones(n, 1);
z0 = zeros(n, 1);
q0 = [x0; z0];

k = 1;
r0 = 30;
r = poly(-r0*ones(n, 1));
r = r(2:end);

tol = 1e-1;
rbar = 1e1;
rho0 = 2;
tau = -1/rbar*log(1/((rho0-tol)*rbar));
% rho = create_rho(rho0, tau, tol);
rho = @(t) (rho0 - tol)*exp(-rbar*t) + tol;

plant = @plant4b;

tmax = 15;
ode_options = odeset('AbsTol', 1e-12, 'RelTol', 1e-9);%, ...
%                      'OutputFcn', @odeprog,'Events', @odeabort);

%% Prescribed perfomance differentiator
peak = tau;
satlvl = 3;
observer = @(t, z, y) ppd_hgo2b_sat(t, z, y, rho, r, satlvl);
controller = @(t, x, w) nocontrol();
sys1 = @(t, q) control_loop(t, q, plant, [n 0 n], controller, observer);

[t, q] = ode15s(sys1, [0 tmax], q0, ode_options);
t_p = (t < peak);

% Reconstruct state estimates
x = q(:, 1:n);
z = q(:, n+1:end);

e1 = x(:, 1) - z(:, 1);
e2 = x(:, 2) - z(:, 2);
xi = e1./rho(t);
eps1 = log((1 + xi)./(1 - xi));

drho = @(t) -rbar.*(rho(t) - tol);

de1 = e2 - k*e1;
deps1 = 2./(1-xi.^2)./rho(t).*(de1 - eps1 - xi.*drho(t));

k = eps1./e1;
dk = (deps1.*e1 - de1.*eps1)./e1.^2;

xhat = zeros(size(z));
xhat(:, 1) = z(:, 1);
xhat(:, 2:end) = sat(z(:, 2:end), satlvl);

% Reconstruct sliding surface, its estimate and the control input
s = x(:, 1) - xhat(:, 1);

plotter('t', t, 'x', x, 'xhat', xhat)

% plotter('t', t(t_p), 's', s(t_p, :), 'rho', rho);

plotter('t', t, 'x', k, 'xhat', 2./rho(t));
legend('k');

% plotter('t', t, 'x', xi);
% legend('xi');

plotter('t', t, 'x', dk./k, 'xhat', -2./rho(t));
legend('dk/k');

% plotter('t', t(t_p), 'x', x(t_p, :), 'xhat', xhat(t_p, :));

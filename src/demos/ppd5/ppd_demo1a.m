clc; clear; close all;

%% Parameters
n = 1;
x0 = zeros(n, 1);
z0 = ones(n, 1);
q0 = [x0; z0];

k = 2e3;

tol = 1e-3;
rho0 = 3;
T1 = 0.002;
rho = create_rho(rho0, T1, tol);

plant = @plant1a;
tmax = 0.008;
ode_options = odeset('AbsTol', 1e-11, 'RelTol', 1e-8, ...
                      'OutputFcn', @odeprog,'Events', @odeabort);

%% Prescribed perfomance differentiator
observer = @(t, z, y) ppd(t, z, y, rho, k, T1);
controller = @(t, x, w) nocontrol();
sys1 = @(t, q) control_loop(t, q, plant, [n 0 n], controller, observer);

[t, q] = ode15s(sys1, [0 tmax], q0, ode_options);

% Reconstruct state estimates
x = q(:, 1:n);
z = q(:, n+1:end);

% plotter('t', t, 's', x - z, 'rho', rho);
% save2tikz('../tex/reports/ppd5/figures/ppd1.tikz');

dx = 100*cos(1e4*t);

xi = (x-z)./rho(t);
dz = k*log((1+xi)./(1-xi));
e2 = dx - dz;

fbar = 1e6;
[~, lambda] = create_rho(rho0, T1, tol);
s = lambda.*rho0.*exp(-lambda.*T1);
drho = @(t) (t < T1).*(-lambda.*(rho(t) - s.*t) + s);
b = @(t) abs(drho(t)) + rho(t)*fbar/2/k;
find(abs(e2./b(t)) > 1)

plotter('t', t, 's', e2, 'rho', b);
% save2tikz('../tex/reports/ppd5/figures/ppd2.tikz');

plotter('t', t, 'x', e2./b(t));

e10 = x(1) - z(1);
mu1 = dx(1) - abs(drho(0)) - 1/2/k*fbar*rho(0);
mu2 = dx(1) + abs(drho(0)) + 1/2/k*fbar*rho(0);
rho0*tanh(mu2/2/k) > e10
rho0*tanh(mu1/2/k) < e10
all(-lambda + fbar*s./(2*k*b(t(t<T1))).*(lambda.*t(t<T1) + 1) > 0)

all(abs(1 + rho(t(t<T1)).*(1-xi(t<T1).^2)/2/k.*(-lambda + fbar*s./(2*k*b(t(t<T1))).*(lambda.*t(t<T1) + 1))) > 1)

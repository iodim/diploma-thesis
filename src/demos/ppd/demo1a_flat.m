clc; clear; close all;

%% Parameters
n = 1;
x0 = zeros(n, 1);
z0 = 2*ones(n, 1);
q0 = [x0; z0];

k = 2e3;

tol = 1e-3;
rho0 = 10;
T1 = 0.003;
rho = create_rho(rho0, T1, tol);

plant = @plant1a;
tmax = 0.008;
ode_options = odeset('AbsTol', 1e-14, 'RelTol', 1e-11, ...
                      'OutputFcn', @odeprog,'Events', @odeabort);

%% Prescribed perfomance differentiator
observer = @(t, z, y) ppd(t, z, y, rho, k, T1);
controller = @(t, x, w) nocontrol();
sys1 = @(t, q) control_loop(t, q, plant, [n 0 n], controller, observer);

[t, q] = ode15s(sys1, [0 tmax], q0, ode_options);

% Reconstruct state estimates
x = q(:, 1:n);
z = q(:, n+1:end);

plotter('t', t, 's', x - z, 'rho', rho);
% save2tikz('../tex/reports/ppd5/figures/ppd1.tikz');

dx = 100*cos(1e4*t);

xi = (x-z)./rho(t);
dz = k*log((1+xi)./(1-xi));
e2 = dx - dz;

fbar = 1e6;
drho = @(t) (t < T1).*2*T1^2./(t - T1).^3.*(rho(t) -tol);
ddrho = @(t) (t < T1).*(2*T1^2./(t - T1).^3 - 3./(t - T1)).*drho(t);

gamma = 5;
cinv =  (1 - xi.^2).*rho(t)/2/k;
rho2 = @(t) -gamma*drho(t) + rho(t)*fbar/2/k;
drho2 = @(t) -gamma*ddrho(t) + drho(t)*fbar/2/k;

find(abs(e2./rho2(t)) > 1)

plotter('t', t, 's', e2, 'rho', rho2);
% save2tikz('../tex/reports/ppd5/figures/ppd2.tikz');

plotter('t', t, 'x', e2./rho2(t));
hold on
plot([T1 T1], [-1.2, 1.2], ':k')

e10 = x(1) - z(1);
mu1 = dx(1) - abs(drho(0)) - 1/2/k*fbar*rho(0);
mu2 = dx(1) + abs(drho(0)) + 1/2/k*fbar*rho(0);
rho0*tanh(mu2/2/k) > e10
rho0*tanh(mu1/2/k) < e10

figure
% plot(t, rho2(t))
% hold on
% plot(t, -drho(t) + cinv*fbar)
plot(t, rho2(t) + cinv.*drho2(t))
hold on
plot(t, -drho(t) + cinv*fbar)

numel(find(abs(rho2(t) + cinv.*drho2(t)) < abs(-drho(t) - cinv*fbar)))

% tau = t(t<T1);
% asf= 2*T1^2./(tau-T1).^3.*(1 - fbar*tol/2/k./rho2(tau)) + 3*drho(tau)./rho2(tau)./(tau-T1);
% all(abs(1 + (1-xi(t<T1).^2).*rho(tau)/2/k.*asf) > 1)
% figure
% plot(tau, 1 + (1-xi(t<T1).^2).*rho(tau)/2/k.*asf);
clc; clear; close all;

%% Parameters
n = 2;
x0 = zeros(n, 1);
z0 = ones(1, 1);
q0 = [x0; z0];

k = 1000;

tol = 1e-4;
rho0 = 10;
lambda = 50;
rho = @(t) tol./(1 - exp(-t));

plant = @plant2;
tmax = 5;
ode_options = odeset('AbsTol', 1e-16, 'RelTol', 1e-13);%, ...
%                       'OutputFcn', @odeprog,'Events', @odeabort);

%% Prescribed perfomance differentiator
observer = @(t, z, y) ppd(t, z, y, rho, k, 100);
controller = @(t, x, w) nocontrol();
sys1 = @(t, q) control_loop(t, q, plant, [n 0 1], controller, observer);

[t, q] = ode15s(sys1, [0 tmax], q0, ode_options);

% Reconstruct state estimates
x = q(:, 1:n);
z = q(:, n+1:end);

plotter('t', t, 's', x(:, 1) - z, 'rho', rho);
title('Immediate observation error');

xi = (x(:, 1)-z)./rho(t);
dz = k*log((1+xi)./(1-xi));
e2 = x(:, 2) - dz;

f = -2*(x(:,1).^2 - 1).*x(:,2) - x(:,1) + 1;
fbar = max(abs(f));
drho = @(t) -tol*exp(-t)./(1-exp(-t)).^2;
ddrho = @(t) -lambda.*drho(t);

cinv =  (1 - xi.^2).*rho(t)/2/k;
rho2 = @(t) -drho(t) + rho(t)*fbar/2/k;
drho2 = @(t) -ddrho(t) + drho(t)*fbar/2/k;

find(abs(e2./rho2(t)) > 1)

plotter('t', t, 's', e2, 'rho', rho2);
title('Implied observation error');

plotter('t', t, 'x', e2./rho2(t));
title('\xi_2')

e10 = x(1, 1) - z(1);
mu1 = x(1, 2) - abs(drho(0)) - 1/2/k*fbar*rho(0);
mu2 = x(1, 2) + abs(drho(0)) + 1/2/k*fbar*rho(0);
rho0*tanh(mu2/2/k) > e10
rho0*tanh(mu1/2/k) < e10

figure
plot(t, rho2(t) + rho(t)/2/k.*drho2(t))
hold on
plot(t, xi.*drho(t) + cinv*fbar)
title('rho2 + rho/2k *drho2 over num');

figure
plot(t, rho(t)/2/k.*drho2(t))
% 
% 
% figure
% plot(t, abs(xi.*drho(t) + cinv.*f)./(abs(rho2(t) + cinv.*drho2(t))))
% 
% figure
% plot(t, drho2(t)./rho2(t))
% title('dhro2/rho2');
% 
% 
% figure
% plot(t, 1 + rho(t)/2/k.*drho2(t)./rho2(t));
% figure
% plot(t, abs(xi.*drho(t) + cinv.*f)./rho2(t))
% max(abs(xi.*drho(t) + cinv.*f)./rho2(t))
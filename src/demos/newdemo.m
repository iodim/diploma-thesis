%DEMOTEST
clc; clear; close all;

n = 2;
k = 1;
sat = 25;
mu = 0.01;
x0 = [1; 1];
z0 = [0; 0];
alpha = [10, 25];

r = 6;
Lambda = fliplr(poly(-r))';

rho = @(t) (10 - 0.01)*exp(-5*t) + 0.01;

tmax = 1.5;
peaking_time = 0.05;
ode_options = odeset('AbsTol', 1e-9, 'RelTol', 1e-6);

plant = @plant2;
controller = @(t, x, w) ppc_sat(t, x, Lambda, rho, k, sat);
observer = @(t, xhat, y) hgo(t, xhat, y, alpha, mu);
system = @(t, q) control_loop(t, q, plant, [n 0 n], controller, observer);

[t, q] = ode15s(system, [0 tmax], [x0; z0], ode_options);

x = q(:, 1:n);
xhat = q(:, n+1:end);

s = x*Lambda;
shat = xhat*Lambda;
u = controller(t, xhat');

figure();
subplot(2, 2, 1)
    box on; hold on; axis([0, max(t), 1.1*min(x(:, 1)), 1.1*max(x(:, 1))]);
    plot(t, x(:, 1), 'k');
    plot(t, xhat(:, 1), '--k');
    ylabel('$x_1(t), \hat{x}_1(t)$', 'Interpreter', 'Latex');
subplot(2, 2, 2)
    box on; hold on; axis([0, max(t), 1.1*min(x(:, 2)), 1.1*max(x(:, 2))]);
    plot(t, x(:, 2), 'k');
    plot(t, xhat(:, 2), '--k');
    ylabel('$x_2(t), \hat{x}_2(t)$', 'Interpreter', 'Latex');    
subplot(2, 2, 3)
    box on; hold on; axis([0, max(t), -1.1*rho(0), 1.1*rho(0)]);
    plot(t, s, 'k');
    plot(t, shat, '--k');
    plot([t, t], [rho(t), -rho(t)], ':k'); 
    ylabel('$s(x(t)), s(\hat{x}(t))$', 'Interpreter', 'Latex');
%     % peaking plot
%     axes('position', [.675 .675 .2 .2]); 
%     box on; hold on; axis tight;
%     plot(t(t_p), s(t_p), 'k');
%     plot(t(t_p), shat(t_p), '--k');
%     plot([t(t_p), t(t_p)], [rho(t(t_p)), -rho(t(t_p))], ':k'); 
subplot(2, 2, 4)
    box on; hold on; axis([0, tmax, 1.1*min(u), 1.1*max(u)]);
    plot(t, u, 'k')
    ylabel('$u(t)$', 'Interpreter', 'Latex');
    xlabel('$t$', 'Interpreter', 'Latex');
%     % peaking plot
%     axes('position', [.675 .175 .2 .2]); 
%     box on; hold on; axis([0, max(t(t_p)), -1.1*sat, 1.1*sat]);
%     plot(t(t_p), u(t_p), 'k')

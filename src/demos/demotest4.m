%   Ioannis Dimanidis (2017)
clc; clear; close all;

%% Parameters
n = 4;
x0 = ones(n, 1);
z0 = zeros(n,1);
q0 = [x0; z0];

k = 2;
satlvl = 1 - 1e-16;
mu = 0.01;
alpha = [4, 6, 4, 1];

r = [1.1, 1.2, 1.3];
Lambda = fliplr(poly(-r))';

rho = @(t) (20 - 0.05)*exp(-1*t) + 0.05;

plant = @plant4b;

tmax = 15;
peaking_time = 0.05;
ode_options = odeset('AbsTol', 1e-16, 'RelTol', 1e-13);

%% High-Gain Observer
observer = @(t, xhat, y) hgo(t, xhat, y, alpha, mu);
controller = @(t, x, w) ppc_sat2(t, x, Lambda, rho, k, satlvl);
sys1 = @(t, q) control_loop(t, q, plant, [n 0 n], controller, observer);

[t, q] = ode15s(sys1, [0 tmax], q0, ode_options);
t_p = (t < peaking_time);

% Reconstruct sliding surface, its estimate, and the control input
x = q(:, 1:n);
xhat = q(:, n+1:end);

s = x*Lambda;
shat = xhat*Lambda;
u = controller(t, xhat');

%% Plots
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
    box on; hold on; axis([0, max(t), 1.1*min(x(:, 3)), 1.1*max(x(:, 3))]);
    plot(t, x(:, 3), 'k');
    plot(t, xhat(:, 3), '--k');
    ylabel('$x_3(t), \hat{x}_3(t)$', 'Interpreter', 'Latex');
    xlabel('$t$', 'Interpreter', 'Latex');
subplot(2, 2, 4)
    box on; hold on; axis([0, max(t), 1.1*min(x(:, 4)), 1.1*max(x(:, 4))]);
    plot(t, x(:, 4), 'k');
    plot(t, xhat(:, 4), '--k');   
    ylabel('$x_4(t), \hat{x}_4(t)$', 'Interpreter', 'Latex');
    xlabel('$t$', 'Interpreter', 'Latex');

figure();
subplot(2, 1, 1)
    box on; hold on; axis([0, max(t), -1.1*rho(0), 1.1*rho(0)]);
    plot(t, s, 'k');
    plot(t, shat, '--k');
    plot([t, t], [rho(t), -rho(t)], ':k'); 
    ylabel('$s(x(t)), s(\hat{x}(t))$', 'Interpreter', 'Latex');
    % peaking plot
%     axes('position', [.675 .675 .2 .2]); 
%     box on; hold on; axis tight;
%     plot(t(t_p), s(t_p), 'k');
%     plot(t(t_p), shat(t_p), '--k');
%     plot([t(t_p), t(t_p)], [rho(t(t_p)), -rho(t(t_p))], ':k'); 

subplot(2, 1, 2)
    box on;
    hold on;
    plot(t, u, 'k')
    ylabel('$u(t)$', 'Interpreter', 'Latex');
    xlabel('$t$', 'Interpreter', 'Latex');
    % peaking plot
%     axes('position', [.675 .175 .2 .2]); 
%     box on; hold on; axis([0, max(t(t_p)), -1.1*sat, 1.1*sat]);
%     plot(t(t_p), u(t_p), 'k')

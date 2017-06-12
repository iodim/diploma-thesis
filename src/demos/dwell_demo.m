%DWELL_DEMO Performs a simulation demo for a dwell time PPC.
   
%   Ioannis Dimanidis (2017)
clc; clear; close all;

%% Parameters
n = 4;
x0 = ones(n, 1);
z0 = zeros(n,1);
q0 = [x0; z0];

k = 12;
satlvl = inf;
mu = 0.001;
alpha = [2, 1, 1, 1];
% M = [1.6, 1.3, 3];
M =  [10, 10, 10];

r = [1.1, 1.2, 1.3];
Lambda = fliplr(poly(-r))';

rho = @(t) (20 - 0.05)*exp(-1*t) + 0.05;

plant = @plant4b;

td = 0.04;

tmax = 15;
ode_options = odeset('AbsTol', 1e-9, 'RelTol', 1e-6);

%% Cascade High-Gain Observer
peak = 0.05;
observer = @(t, z, y) chgo(t, z, y, alpha, mu, M);
controller = @(t, x, w) ppc_sat(t, x, Lambda, rho, k, satlvl);
dwell_controller = @(t, x, w) dwell_time_controller(t, x, controller, td);
sys1 = @(t, q) control_loop(t, q, plant, [n 0 n], dwell_controller, observer);

[t, q] = ode15s(sys1, [0 tmax], q0, ode_options);
t_p = (t < peak);

% Reconstruct state estimates
x = q(:, 1:n);
z = q(:, n+1:end);

xhat = zeros(size(x));
xhat(:, 1) = z(:, 1);
xhat(:, 2) = sat(z(:, 2)/mu, M(1));

for i = 3:n
   xhat(:, i) = sat(alpha(i)/mu*(z(:, i) + xhat(:, i-1)), M(i-1));
end

% Reconstruct sliding surface, its estimate and the control input
s = x*Lambda;
shat = xhat*Lambda;
u = dwell_controller(t, xhat');

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
    axes('position', [.675 .675 .2 .2]); 
    box on; hold on; axis tight;
    plot(t(t_p), s(t_p), 'k');
    plot(t(t_p), shat(t_p), '--k');
    plot([t(t_p), t(t_p)], [rho(t(t_p)), -rho(t(t_p))], ':k'); 

subplot(2, 1, 2)
    box on;
    hold on;
    plot(t, u, 'k')
    ylabel('$u(t)$', 'Interpreter', 'Latex');
    xlabel('$t$', 'Interpreter', 'Latex');
    % peaking plot
    axes('position', [.675 .2 .2 .2]); 
    box on; hold on; axis([0, max(t(t_p)), -1.1*satlvl, max(1, max(u(t_p)))]);
    plot(t(t_p), u(t_p), 'k')

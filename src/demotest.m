%DEMOA Performs a simulation demo for plant2 using PPC and HGO.
%   See more:
%   Bechlioulis, Charalampos P., Achilles Theodorakopoulos, and George A. 
%   Rovithakis. "Output feedback stabilization with prescribed performance 
%   for uncertain nonlinear systems in canonical form." Decision and Control
%   (CDC), 2013 IEEE 52nd Annual Conference on. IEEE, 2013.
   
%   Ioannis Dimanidis (2017)
clc; clear; close all;

%% Parameters
n = 2;
k = 1;
sat = inf;
mu = 0.01;
q0 = [1; 1];
qhat0 = [0; 0];
alpha = [10, 25];

r = 6;
Lambda = fliplr(poly(-r))';

plant = @plant2;

rho = @(t) (10 - 0.01)*exp(-5*t) + 0.01;

tmax = 5;
peaking_time = 0.05;
ode_options = odeset('AbsTol', 1e-16, 'RelTol', 1e-13);

%% Full state feedback
sys = @(t, q) ppc(t, q, plant, Lambda, rho, k);

[t, q] = ode15s(sys, [0 tmax], q0, ode_options);

x = q(:, 1:n);

s = x*Lambda;
u = -k*log((1 + s./rho(t))./(1 - s./rho(t)));

% Plots
figure();
subplot(2, 2, 1)
    box on; hold on; axis([0, max(t), 1.1*min(x(:, 1)), 1.1*max(x(:, 1))]);
    plot(t, x(:, 1), 'k');
    ylabel('$x_1(t)$', 'Interpreter', 'Latex');
subplot(2, 2, 2)
    box on; hold on; axis([0, max(t), 1.1*min(x(:, 2)), 1.1*max(x(:, 2))]);
    plot(t, x(:, 2), 'k');
    ylabel('$x_2(t)$', 'Interpreter', 'Latex');    
subplot(2, 2, 3)
    box on; hold on; axis([0, max(t), -1.1*rho(0), 1.1*rho(0)]);
    plot(t, s, 'k');
    plot([t, t], [rho(t), -rho(t)], ':k'); 
    ylabel('$s(x(t))$', 'Interpreter', 'Latex');
subplot(2, 2, 4)
    box on; hold on; axis([0, tmax, 1.1*min(u), 1.1*max(u)]);
    plot(t, u, 'k')
    ylabel('$u(t)$', 'Interpreter', 'Latex');
    xlabel('$t$', 'Interpreter', 'Latex');

    
%% Output feedback (HGO)
observer = @(t, xhat, y) hgo(t, xhat, y, alpha, mu);
sys = @(t, q) ppc_observer2(t, q, plant, observer, Lambda, rho, k, sat);

[t, q] = ode15s(sys, [0 tmax], [q0; qhat0], ode_options);
t_p = (t < peaking_time);

% Reconstruct sliding surface, its estimate and the control input
x = q(:, 1:n);
xhat = q(:, n+1:end);

s = x*Lambda;
shat = xhat*Lambda;
u = -k*log((1 + sigma(shat./rho(t)))./(1 - sigma(shat./rho(t))));
% u(imag(u) ~= 0) = sign(real(u(imag(u) ~= 0)))*sat;
% u = min(sat, max(-sat, u));

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

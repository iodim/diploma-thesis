%DEMOC Performs a simulation demo for plant4b using PPC and CHGO.
%
%   See more:
%   Bechlioulis, Charalampos P., Achilles Theodorakopoulos, and George A. 
%   Rovithakis. "Output feedback stabilization with prescribed performance 
%   for uncertain nonlinear systems in canonical form." Decision and Control
%   (CDC), 2013 IEEE 52nd Annual Conference on. IEEE, 2013.
   
%   Ioannis Dimanidis (2017)
clc; clear; close all;

%% Parameters
n = 4;
x0 = ones(n, 1);
z0 = zeros(n,1);
q0 = [x0; z0];

k = 12;
sat = inf;
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
sys1 = @(t, q) dwell_ppc_observer(t, q, plant, observer, Lambda, rho, k, sat, td);

[t, q] = ode15s(sys1, [0 tmax], q0, ode_options);
t_p = (t < peak);

% Reconstruct state estimates
x = q(:, 1:n);
z = q(:, n+1:end);

xhat = zeros(size(x));
xhat(:, 1) = z(:, 1);
xhat(:, 2) = min(M(1), max(-M(1), z(:, 2)/mu));

for i = 3:n
   xhat(:, i) = min(M(i-1), max(-M(i-1), alpha(i)/mu*(z(:, i) + xhat(:, i-1))));
end

% Reconstruct sliding surface, its estimate and the control input
s = x*Lambda;
shat = xhat*Lambda;
u = -k*log((1 + shat./rho(t))./(1 - shat./rho(t)));
u(imag(u) ~= 0) = sign(real(u(imag(u) ~= 0)))*sat;
u = min(sat, max(-sat, u));
u(t < td) = 0;

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
    box on; hold on; axis([0, max(t(t_p)), -1.1*sat, max(1, max(u(t_p)))]);
    plot(t(t_p), u(t_p), 'k')


%% Regular High-Gain Observer
peak = 0.05;
sat = 15;
alpha = [4, 6, 4, 1];
observer = @(t, xhat, y) hgo(t, xhat, y, alpha, mu);

sys1 = @(t, q) ppc_observer(t, q, plant, observer, Lambda, rho, k, sat);

[t, q] = ode15s(sys1, [0 tmax], q0, ode_options);
t_p = (t < peak);

% Reconstruct sliding surface, its estimate and the control input
x = q(:, 1:n);
xhat = q(:, n+1:end);

s = x*Lambda;
shat = xhat*Lambda;
u = -k*log((1 + shat./rho(t))./(1 - shat./rho(t)));
u(imag(u) ~= 0) = sign(real(u(imag(u) ~= 0)))*sat;
u = min(sat, max(-sat, u));
u(t < td) = 0;


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
    box on; hold on; axis([0, max(t(t_p)), -1.1*sat, max(1, max(u(t_p)))]);
    plot(t(t_p), u(t_p), 'k')

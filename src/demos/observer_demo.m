%DEMOA Performs a simulation demo for plant2 using PPC and HGO.
%   See more:
%   Bechlioulis, Charalampos P., Achilles Theodorakopoulos, and George A. 
%   Rovithakis. "Output feedback stabilization with prescribed performance 
%   for uncertain nonlinear systems in canonical form." Decision and Control
%   (CDC), 2013 IEEE 52nd Annual Conference on. IEEE, 2013.
   
%   Ioannis Dimanidis (2017)
clc; clear; close all;

%% Parameters
% RED parameters
L = 5;
muV = [1 1 1 1];
lambda = [2 2 3 4];

% HGO parameters
mu = 0.001;
alpha = [4, 6, 4, 1];

% General parameters
n = 4;
q0 = [1; 1; 1; 1];
qhat0 = [1; 0; 0; 0];

plant = @plant4b;

tmax = 15;
ode_options = odeset('AbsTol', 1e-9, 'RelTol', 1e-6);

%% Robust Exact Differentiator (RED)
observer = @(t, xhat, y) gfed(t, xhat, y, lambda, L, muV);
% observer = @(t, xhat, y) hgo(t, xhat, y, alpha, mu);
controller = @(t, x, w) nocontrol();
sys = @(t, q) control_loop(t, q, plant, [n 0 n], controller, observer);

[t, q] = ode15s(sys, [0 tmax], [q0; qhat0], ode_options);

x = q(:, 1:n);
xhat = q(:, n+1:end);

% Plots
figure;
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

%% High-Gain Observer (HGO)
observer = @(t, xhat, y) hgo(t, xhat, y, alpha, mu);
controller = @(t, x, w) nocontrol();
sys = @(t, q) control_loop(t, q, plant, [n 0 n], controller, observer);

[t, q] = ode15s(sys, [0 tmax], [q0; qhat0], ode_options);

x = q(:, 1:n);
xhat = q(:, n+1:end);

% Plots
figure;
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

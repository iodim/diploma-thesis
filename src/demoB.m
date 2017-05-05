%DEMOB Performs a simulation demo for plant4b using PPC and CHGO.
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

k = 5;
sat = 10;
mu = 0.001;
alpha = [2, 1, 1, 1];
M = [1.6, 1.2, 3];

r = [10 10 10];
Lambda = fliplr(poly(-r))';

plant = @plant4b;

rho1 = @(t) (10 - 0.01)*exp(-5*t) + 0.01;

tmax = 50;
ode_options = odeset('AbsTol', 1e-9, 'RelTol', 1e-6);

%% Cascade High-Gain Observer
observer = @(t, z, y) chgo(t, z, y, alpha, mu, M);
sys1 = @(t, q) ppc_observer(t, q, plant, observer, Lambda, rho1, k, sat);

[t, q] = ode15s(sys1, [0 tmax], q0, ode_options);

% Reconstruct state estimates
x = q(:, 1:n);
z = q(:, n+1:end);

xhat = zeros(size(x));
xhat(:, 1) = z(:, 1);
xhat(:, 2) = min(M(1), max(-M(1), z(:, 2)/mu));

for i = 3:n
   xhat(:, i) = min(M(i-1), max(-M(i-1), alpha(i)/mu*(z(:, i) + xhat(:, i-1))));
end


%% Plots
figure();
subplot(2, 2, 1)
    box on;
    hold on;
    plot(t, x(:, 1), 'k');
    plot(t, xhat(:, 1), '--k');
subplot(2, 2, 2)
    box on;
    hold on;
    plot(t, x(:, 2), 'k');
    plot(t, xhat(:, 2), '--k');
subplot(2, 2, 3)
    box on;
    hold on;
    plot(t, x(:, 3), 'k');
    plot(t, xhat(:, 3), '--k');
subplot(2, 2, 4)
    box on;
    hold on;
    plot(t, x(:, 4), 'k');
    plot(t, xhat(:, 4), '--k');   

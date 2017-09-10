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
controller = @(t, x, w) nocontrol();
sys = @(t, q) control_loop(t, q, plant, [n 0 n], controller, observer);

[t, q] = ode15s(sys, [0 tmax], [q0; qhat0], ode_options);

x = q(:, 1:n);
xhat = q(:, n+1:end);

plotter('t', t, 'x', x, 'xhat', xhat);

%% High-Gain Observer (HGO)
observer = @(t, xhat, y) hgo(t, xhat, y, alpha, mu);
controller = @(t, x, w) nocontrol();
sys = @(t, q) control_loop(t, q, plant, [n 0 n], controller, observer);

[t, q] = ode15s(sys, [0 tmax], [q0; qhat0], ode_options);

x = q(:, 1:n);
xhat = q(:, n+1:end);

plotter('t', t, 'x', x, 'xhat', xhat);

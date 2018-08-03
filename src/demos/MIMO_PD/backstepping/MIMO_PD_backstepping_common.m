clc; clear; close all;

%% Parameters
n = 6;
randseed(1);
x0 = rand(n, 1);
z0 = zeros(n, 1);
q0 = [x0; z0];

plant = @(t, x, u) MIMO_pd(t, x, u);

% Reference trajectories
yd1 = @(t) cos(2.*t);
yd2 = @(t) sin(3.*t);
yd = @(t) [yd1(t); yd2(t)];

% Solver parameters
tmax = 15;
ode_options = odeset('AbsTol', 1e-11, 'RelTol', 1e-9, ...
                      'OutputFcn', @odeprog,'Events', @odeabort);

% PPC parameters
k = 2*[1 2 3];
tol = 1e-1;
rbar = 2;
rho0 = 50;
rho = @(t) (rho0 - tol)*exp(-rbar*t) + tol;

% HGO parameters
mu = 1e-3;
satlvl = 200;
alpha = [15, 75, 125];

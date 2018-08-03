clc; clear; close all;

%% Parameters
n = 6;
randseed(1);
x0 = rand(n, 1);
z0 = ones(n, 1);
q0 = [x0; z0];

% Reference trajectories
syms t
yd1 = @(t) cos(2.*t);
yd2 = @(t) sin(3.*t);
yd = @(t) [yd1(t); yd2(t)];

[psi1, dpsi1] = create_psi(t, yd1, n/2);
[psi2, dpsi2] = create_psi(t, yd2, n/2);

psi = @(t) [psi1(t); psi2(t)];
dpsi = @(t) [dpsi1(t); dpsi2(t)];

plant = @(t, x, u) MIMO_pd(t, x, u);
plant_e = @(t, e, u) plant(t, e + psi(t), u) - dpsi(t);

% Solver parameters
tmax = 15;
ode_options = odeset('AbsTol', 1e-11, 'RelTol', 1e-9, ...
                     'OutputFcn', @odeprog,'Events', @odeabort);

% PPC parameters
k = 1;
r = 3;
Lambda = fliplr(poly([-r -r]))';
tol = 1e-2;
rbar = 2;
rho0 = 50;
rho = @(t) (rho0 - tol)*exp(-rbar*t) + tol;

% HGO parameters
mu = 5e-4;
satlvl = 200;
alpha = [15, 75, 125];

clc; clear; close all;

%% Parameters
n = 8;
% randseed(1);
x0 = [ -0.0038
        0.8017
        0.1493
        0.6904
        0.4773
        0.1720
       -0.5065
        0.3328];

z0 = zeros(6, 1);

% Reference trajectories
syms t
yd1 = @(t) cos(2.*t);
yd2 = @(t) sin(3.*t);
yd = @(t) [yd1(t); yd2(t)];

[psi1, dpsi1] = create_psi(t, yd1, 3);
[psi2, dpsi2] = create_psi(t, yd2, 3);

psi = @(t) [zeros(2, 1); psi1(t); psi2(t)];
dpsi = @(t) [zeros(2, 1); dpsi1(t); dpsi2(t)];

q0 = [x0 - psi(0); z0];

plant = @(t, x, u) mimo_pd_zd(t, x, u);
plant_e = @(t, e, u) plant(t, e + psi(t), u) - dpsi(t);

% Solver parameters
tmax = 15;
ode_options = odeset('AbsTol', 1e-10, 'RelTol', 1e-6, ...
                     'OutputFcn', @odeprog,'Events', @odeabort);

% PPC parameters
k = 30;
r = 3;
Lambda = fliplr(poly([-r -r]))';
tol = 4.5e-2;
rbar = 2;
rho0 = 50;
rho = @(t) (rho0 - tol)*exp(-rbar*t) + tol;

% HGO parameters
mu = 5e-5;
satlvl = 100;
alpha = [15, 75, 125];

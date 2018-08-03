clc; clear; close all;

%% Parameters
n = 2;
e0 = [-5/2; 1];
z0 = zeros(n, 1);
q0 = [e0; z0];

% Reference trajectories
syms t
yd = @(t) 3/2*cos(2/3*t);

[psi, dpsi] = create_psi(t, yd, n);

% Plant parameters
l = 2;
plant = @(t, x, u) vdp(t, x, u, l);

% Transform plant to e = x-y coordinates
plant_e = @(t, e, u) plant(t, e + psi(t), u) - dpsi(t);

% Solver parameters
tmax = 15;
ode_options = odeset('AbsTol', 1e-11, 'RelTol', 1e-7, ...
                     'OutputFcn', @odeprog,'Events', @odeabort);

% PPC parameters
k = 1;
r = 2;
Lambda = fliplr(poly(-r))';
tol = 0.0350;
rbar = 1.5;
rho0 = 30;
rho = @(t) (rho0 - tol)*exp(-rbar*t) + tol;

% HGO parameters
mu = 1e-2;
satlvl = 10;
alpha = [10, 25];

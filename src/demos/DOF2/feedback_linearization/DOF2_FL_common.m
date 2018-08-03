clc; clear; close all;

%% Parameters
n = 4;
x0 = [80/100*pi; 130/180*pi; -pi/6; pi/3];
z0 = zeros(n, 1);
q0 = [x0; z0];

% Reference trajectories
syms t
yd1 = @(t) 90/180*pi + 30/180*pi*sin(t);
yd2 = @(t) 90/180*pi - 30/180*pi*cos(t);
yd = @(t) [yd1(t); yd2(t)];

[psi1, dpsi1] = create_psi(t, yd1, n/2);
[psi2, dpsi2] = create_psi(t, yd2, n/2);

psi = @(t) [psi1(t); psi2(t)];
dpsi = @(t) [dpsi1(t); dpsi2(t)];

% Plant parameters
m1 = 3.2;
l1 = 0.5;
Iz1 = 0.96;
m2 = 2.0;
l2 = 0.4;
Iz2 = 0.81;
g = 9.81;
plant = @(t, x, u) manipulator_2dof(t, x, u, [m1 m2 l1 l2 Iz1 Iz2 g], 1);

% Transform plant to e = x-y coordinates
% plant_e = @(t, e, u) plant(t, e + psi(t), u) - dpsi(t);

% Solver parameters
tmax = 15;
ode_options = odeset('AbsTol', 1e-11, 'RelTol', 1e-7, ...
                     'OutputFcn', @odeprog,'Events', @odeabort);

% Linear feedback
attenuation = 10;
zeta = 0.4;
wn = attenuation/zeta;

% HGO parameters
mu = 1e-2;
satlvl = 300;
alpha = [10, 25];

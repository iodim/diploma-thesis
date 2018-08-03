clc; clear; close all;

%% Parameters
n = 4;
x0 = zeros(n, 1);
x0 = [80/180*pi; 0;
      130/180*pi; 0];
  
z0 = ones(n, 1);
q0 = [x0; z0];

% Plant parameters
% m1 = 0.5;
% m2 = 0.2;
% l1 = 0.3;
% l2 = 0.25;
% Iz1 = 0;
% Iz2 = 0;

m1 = 3.2;
l1 = 0.5;
Iz1 = 0.96;
m2 = 2.0;
l2 = 0.4;
Iz2 = 0.81;
g = 9.81;
plant = @(t, x, u) manipulator_2dof(t, x, u, [m1 m2 l1 l2 Iz1 Iz2 g]);

% Reference trajectories
% yd = @(t) [sin(2*t) + 0.3*cos(5*t);
% 	        0.5*sin(3*t) + 2*cos(t)];
yd = @(t) [pi/2 + pi/6*cos(t);
           pi/2 - pi/6*cos(t)];
% yd = @(t) [cos(2.*t);
%            sin(3.*t)];

% Solver parameters
tmax = 15;
ode_options = odeset('AbsTol', 1e-14, 'RelTol', 1e-9, ...
                      'OutputFcn', @odeprog,'Events', @odeabort);

% PPC parameters
k = [2 3];
tol = 1e-2;
rbar = 3;
rho0 = 10;

rho1 = @(t) (rho0 - tol)*exp(-3*t) + tol;
rho2 = @(t) (rho0 - tol)*exp(-1*t) + tol;


% HGO parameters
mu = 1e-4;
satlvl1 = 500;
satlvl2 = 100;
alpha = [10, 25];
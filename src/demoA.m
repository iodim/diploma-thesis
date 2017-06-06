%DEMOA Performs a simulation demo for plant2 using PPC and HGO.
%   See more:
%   Bechlioulis, Charalampos P., Achilles Theodorakopoulos, and George A. 
%   Rovithakis. "Output feedback stabilization with prescribed performance 
%   for uncertain nonlinear systems in canonical form." Decision and Control
%   (CDC), 2013 IEEE 52nd Annual Conference on. IEEE, 2013.
   
%   Ioannis Dimanidis (2017)
clc; clear; close all;

%% Parameters
k = 5;
sat = 5;
mu = 0.01;
q0 = [1; -1];
qhat0 = [0.8; 0];
alpha = [10, 25];

r = 6;
Lambda = fliplr(poly(-r))';

plant = @plant2;

rho1 = @(t) (10 - 0.01)*exp(-0.5*t) + 0.01;
rho2 = @(t) (10 - 0.01)*exp(-1*t) + 0.01;
rho3 = @(t) (10 - 0.01)*exp(-2*t) + 0.01;

tmax = 5;
ode_options = odeset('AbsTol', 1e-9, 'RelTol', 1e-6);

%% Full state feedback
sys1 = @(t, q) ppc(t, q, plant, Lambda, rho1, k);
sys2 = @(t, q) ppc(t, q, plant, Lambda, rho2, k);
sys3 = @(t, q) ppc(t, q, plant, Lambda, rho3, k);

[t1, q1] = ode15s(sys1, [0 tmax], q0, ode_options);
[t2, q2] = ode15s(sys2, [0 tmax], q0, ode_options);
[t3, q3] = ode15s(sys3, [0 tmax], q0, ode_options);

% Surface and control input reconstruction
s1 = q1*Lambda;
u1 = -k*log((1 + s1./rho1(t1))./(1 - s1./rho1(t1)));
u1(imag(u1) ~= 0) = sign(real(u1(imag(u1) ~= 0)))*sat;
% u1 = min(sat, max(-sat, u1));

s2 = q2*Lambda;
u2 = -k*log((1 + s2./rho2(t2))./(1 - s2./rho2(t2)));
u2(imag(u2) ~= 0) = sign(real(u2(imag(u2) ~= 0)))*sat;
% u2 = min(sat, max(-sat, u2));

s3 = q3*Lambda;
u3 = -k*log((1 + s3./rho3(t3))./(1 - s3./rho3(t3)));
u3(imag(u3) ~= 0) = sign(real(u3(imag(u3) ~= 0)))*sat;
% u3 = min(sat, max(-sat, u3));


figure('Position', [50 300 400 400]);
% r_bar = 0.5
subplot(3, 2, 1);
    hold on; box on;
    plot(t1, q1(:, 1), 'k');
    plot(t1, q1(:, 2), '--k');
    ylabel('$\bar{r} = 0.5$', 'Interpreter', 'Latex');
    axis([0 tmax -5 5])
    title('$x_1(t), x_2(t)$', 'Interpreter', 'Latex')
subplot(3, 2, 2);
    hold on; box on;
    plot(t1, u1, 'k');
    axis([0 tmax 1.1*min(u3) 1.1*max(u1)])
    title('$u(t)$', 'Interpreter', 'Latex')
% r_bar = 1    
subplot(3, 2, 3);
    hold on; box on;
    plot(t2, q2(:, 1), 'k');
    plot(t2, q2(:, 2), '--k');
    ylabel('$\bar{r} = 1$', 'Interpreter', 'Latex');
    axis([0 tmax -5 5])
subplot(3, 2, 4);
    hold on; box on;
    plot(t2, u2, 'k');
    axis([0 tmax 1.1*min(u2) 1.1*max(u2)])
% r_bar = 2
subplot(3, 2, 5);
    hold on; box on;
    plot(t3, q3(:, 1), 'k');
    plot(t3, q3(:, 2), '--k');
    xlabel('$t$', 'Interpreter', 'Latex');
    ylabel('$\bar{r} = 2$', 'Interpreter', 'Latex');
    axis([0 tmax -5 5])
subplot(3, 2, 6);
    hold on; box on;
    plot(t3, u3, 'k');
    axis([0 tmax 1.1*min(u3) 1.1*max(u3)])
    xlabel('$t$', 'Interpreter', 'Latex');
suptitle('State feedback')    

%% High-Gain Observer
observer = @(t, xhat, y) hgo(t, xhat, y, alpha, mu);
sys1 = @(t, q) ppc_observer(t, q, plant, observer, Lambda, rho1, k, sat);
sys2 = @(t, q) ppc_observer(t, q, plant, observer, Lambda, rho2, k, sat);
sys3 = @(t, q) ppc_observer(t, q, plant, observer, Lambda, rho3, k, sat);

[t1, q1] = ode15s(sys1, [0 tmax], [q0; qhat0], ode_options);
[t2, q2] = ode15s(sys2, [0 tmax], [q0; qhat0], ode_options);
[t3, q3] = ode15s(sys3, [0 tmax], [q0; qhat0], ode_options);

% Control input reconstruction
s1 = q1(:, 3:4)*Lambda;
u1 = -k*log((1 + s1./rho1(t1))./(1 - s1./rho1(t1)));
u1(imag(u1) ~= 0) = sign(real(u1(imag(u1) ~= 0)))*sat;
u1 = min(sat, max(-sat, u1));

s2 = q2(:, 3:4)*Lambda;
u2 = -k*log((1 + s2./rho2(t2))./(1 - s2./rho2(t2)));
u2(imag(u2) ~= 0) = sign(real(u2(imag(u2) ~= 0)))*sat;
u2 = min(sat, max(-sat, u2));

s3 = q3(:, 3:4)*Lambda;
u3 = -k*log((1 + s3./rho3(t3))./(1 - s3./rho3(t3)));
u3(imag(u3) ~= 0) = sign(real(u3(imag(u3) ~= 0)))*sat;
u3 = min(sat, max(-sat, u3));

figure('Position', [450 300 800 400]);

% r_bar = 0.5
subplot(3, 3, 1);
    hold on; box on;
    plot(t1, q1(:, 1), 'k');
    plot(t1, q1(:, 2), '--k');
    ylabel('$\bar{r} = 0.5$', 'Interpreter', 'Latex');
    axis([0 tmax -5 5])
    title('$x_1(t), x_2(t)$', 'Interpreter', 'Latex')
subplot(3, 3, 2);
    hold on; box on;
    plot(t1, q1(:, 3), 'k');
    plot(t1, q1(:, 4), '--k');
    axis([0 tmax -5 5])
    title('$\hat{x}_1(t), \hat{x}_2(t)$', 'Interpreter', 'Latex')
subplot(3, 3, 3);
    hold on; box on;
    plot(t1, u1, 'k');
    axis([0 tmax -1.1*sat 1.1*sat])
    title('$u(t)$', 'Interpreter', 'Latex')   

% r_bar = 1
subplot(3, 3, 4);
    hold on; box on;
    plot(t2, q2(:, 1), 'k');
    plot(t2, q2(:, 2), '--k');
    ylabel('$\bar{r} = 1$', 'Interpreter', 'Latex');
    axis([0 tmax -5 5])
subplot(3, 3, 5);
    hold on; box on;
    plot(t2, q2(:, 3), 'k');
    plot(t2, q2(:, 4), '--k');
    axis([0 tmax -5 5])
subplot(3, 3, 6);
    hold on; box on;
    plot(t2, u2, 'k');
    axis([0 tmax -1.1*sat 1.1*sat])

% r_bar = 2
subplot(3, 3, 7);
    hold on; box on;
    plot(t3, q3(:, 1), 'k');
    plot(t3, q3(:, 2), '--k');
    xlabel('$t$', 'Interpreter', 'Latex');
    ylabel('$\bar{r} = 2$', 'Interpreter', 'Latex');
    axis([0 tmax -5 5])
subplot(3, 3, 8);
    hold on; box on;
    plot(t3, q3(:, 3), 'k');
    plot(t3, q3(:, 4), '--k');
    xlabel('$t$', 'Interpreter', 'Latex');
    axis([0 tmax -5 5])
subplot(3, 3, 9);
    hold on; box on;
    plot(t3, u3, 'k');
    axis([0 tmax -1.1*sat 1.1*sat])
    xlabel('$t$', 'Interpreter', 'Latex');
suptitle('Output feedback')    

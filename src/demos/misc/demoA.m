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
satlvl = 5;
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
controller1 = @(t, x, w) ppc_surf(t, x, Lambda, rho1, k);
controller2 = @(t, x, w) ppc_surf(t, x, Lambda, rho2, k);
controller3 = @(t, x, w) ppc_surf(t, x, Lambda, rho3, k);

sys1 = @(t, q) control_loop(t, q, plant, [2 0 0], controller1);
sys2 = @(t, q) control_loop(t, q, plant, [2 0 0], controller2);
sys3 = @(t, q) control_loop(t, q, plant, [2 0 0], controller3);

[t1, q1] = ode15s(sys1, [0 tmax], q0, ode_options);
[t2, q2] = ode15s(sys2, [0 tmax], q0, ode_options);
[t3, q3] = ode15s(sys3, [0 tmax], q0, ode_options);

% Surface and control input reconstruction
u1 = controller1(t1, q1');
u2 = controller2(t2, q2');
u3 = controller3(t3, q3');

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
    axis([0 tmax 1.1*min(u1) 1.1*max(u1)])
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

controller1 = @(t, x, w) ppc_surf(t, x, Lambda, rho1, k);
controller2 = @(t, x, w) ppc_surf(t, x, Lambda, rho2, k);
controller3 = @(t, x, w) ppc_surf(t, x, Lambda, rho3, k);

sat1 = @(t, x, w) sat_control(t, x, controller1, satlvl);
sat2 = @(t, x, w) sat_control(t, x, controller2, satlvl);
sat3 = @(t, x, w) sat_control(t, x, controller3, satlvl);

sys1 = @(t, q) control_loop(t, q, plant, [2 0 2], sat1, observer);
sys2 = @(t, q) control_loop(t, q, plant, [2 0 2], sat2, observer);
sys3 = @(t, q) control_loop(t, q, plant, [2 0 2], sat3, observer);

[t1, q1] = ode15s(sys1, [0 tmax], [q0; qhat0], ode_options);
[t2, q2] = ode15s(sys2, [0 tmax], [q0; qhat0], ode_options);
[t3, q3] = ode15s(sys3, [0 tmax], [q0; qhat0], ode_options);

% Control input reconstruction
u1 = zeros(size(t1));
u2 = zeros(size(t2));
u3 = zeros(size(t3));
for i = 1:length(t1)
    u1(i) = sat1(t1(i), q1(i, 3:4)');
end
for i = 1:length(t2)
    u2(i) = sat2(t2(i), q2(i, 3:4)');
end
for i = 1:length(t3)
    u3(i) = sat3(t3(i), q3(i, 3:4)');
end

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
    axis([0 tmax -1.1*satlvl 1.1*satlvl])
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
    axis([0 tmax -1.1*satlvl 1.1*satlvl])

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
    axis([0 tmax -1.1*satlvl 1.1*satlvl])
    xlabel('$t$', 'Interpreter', 'Latex');
suptitle('Output feedback')    

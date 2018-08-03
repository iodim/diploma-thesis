clc; clear; close all;

x0 = [1; 0];
z0 = [0; 0];

plant = @(t, x, u) [x(2); u];    
control = @(t, x) -[2 2]*x;

satlvl = 4;
eta = [3, 2];

tmax = 10;
ode_options = odeset('AbsTol', 1e-11, 'RelTol', 1e-9, ...
                     'OutputFcn', @odeprog,'Events', @odeabort);

figpath = '../tex/reports/thesis/figures/hgo/';
 
%% State feedback
sys = @(t, x, w) control_loop(t, x, plant, [2 0 0], control);
[t_SFB, x_SFB] = ode15s(sys, [0 tmax], x0, ode_options);

%% Output feedback (mu = 0.1, w/o sat)
mu = 0.1;
observer = @(t, z, y) hgo(t, z, y, eta, mu);

sys = @(t, x, w) control_loop(t, x, plant, [2 0 2], control, observer);
[t_OFB_1, x_OFB_1] = ode15s(sys, [0 tmax], [x0; z0], ode_options);

%% Output feedback (mi = 0.01, w/o sat)
mu = 0.01;
observer = @(t, z, y) hgo(t, z, y, eta, mu);

sys = @(t, x, w) control_loop(t, x, plant, [2 0 2], control, observer);
[t_OFB_2, x_OFB_2] = ode15s(sys, [0 tmax], [x0; z0], ode_options);

%% Output feedback (mu = 0.1, w/ sat)
mu = 0.1;
observer = @(t, z, y) hgo(t, z, y, eta, mu);
sat_controller = @(t, x) sat_control(t, x, control, satlvl);

sys = @(t, x, w) control_loop(t, x, plant, [2 0 2], sat_controller, observer);
[t_OFB_1_sat, x_OFB_1_sat] = ode15s(sys, [0 tmax], [x0; z0], ode_options);

%% Output feedback (mi = 0.01, w/ sat)
mu = 0.01;
observer = @(t, z, y) hgo(t, z, y, eta, mu);
sat_controller = @(t, x) sat_control(t, x, control, satlvl);

sys = @(t, x, w) control_loop(t, x, plant, [2 0 2], sat_controller, observer);
[t_OFB_2_sat, x_OFB_2_sat] = ode15s(sys, [0 tmax], [x0; z0], ode_options);
    
%% Output feedback (mi = 0.01, sat = 24)
sat_controller = @(t, x) sat_control(t, x, control, 24);
sys = @(t, x, w) control_loop(t, x, plant, [2 0 2], sat_controller, observer);
[t_OFB_sat_1, x_OFB_sat_1] = ode15s(sys, [0 tmax], [x0; z0], ode_options);

%% Output feedback (mi = 0.01, sat = 4)
sat_controller = @(t, x) sat_control(t, x, control, 4);
sys = @(t, x, w) control_loop(t, x, plant, [2 0 2], sat_controller, observer);
[t_OFB_sat_2, x_OFB_sat_2] = ode15s(sys, [0 tmax], [x0; z0], ode_options);

%% Plots
figure;
subplot(2, 1, 1);
    hold on;
    box on;
    axis([0 10 -0.32 1.05])
    plot(t_SFB, x_SFB(:, 1), 'k');
    plot(t_OFB_1, x_OFB_1(:, 1), '--k');
    plot(t_OFB_2, x_OFB_2(:, 1), ':k');
    ylabel('$x_1$', 'Interpreter', 'Latex');
    legend('SFB', 'OFB w/o sat, \mu = 0.1', 'OFB w/o, \mu = 0.01')
subplot(2, 1, 2);
    hold on;
    box on;
    axis([0 10 -2.1 0.30])
    plot(t_SFB, x_SFB(:, 2), 'k');
    plot(t_OFB_1, x_OFB_1(:, 2), '--k');
    plot(t_OFB_2, x_OFB_2(:, 2), ':k');
    ylabel('$x_2$', 'Interpreter', 'Latex');
    xlabel('Time, $t$', 'Interpreter', 'Latex');
% save2tikz([figpath 'sfb_ofb_mu_comparison_no_sat.tikz'])

figure;
subplot(2, 1, 1);
    hold on;
    box on;
    axis([0 10 -0.05 1.05])
    plot(t_SFB, x_SFB(:, 1), 'k');
    plot(t_OFB_1_sat, x_OFB_1_sat(:, 1), '--k');
    plot(t_OFB_2_sat, x_OFB_2_sat(:, 1), ':k');
    ylabel('$x_1$', 'Interpreter', 'Latex');
    legend('SFB', 'OFB w/ sat, \mu = 0.1', 'OFB w/ sat, \mu = 0.01')
subplot(2, 1, 2);
    hold on;
    box on;
    axis([0 10 -1.3 0.07])
    plot(t_SFB, x_SFB(:, 2), 'k');
    plot(t_OFB_1_sat, x_OFB_1_sat(:, 2), '--k');
    plot(t_OFB_2_sat, x_OFB_2_sat(:, 2), ':k');
    ylabel('$x_2$', 'Interpreter', 'Latex');
    xlabel('Time, $t$', 'Interpreter', 'Latex');
% save2tikz([figpath 'sfb_ofb_mu_comparison_sat.tikz'])

figure;
subplot(2, 1, 1);
    hold on;
    box on;
    axis([0 10 -0.1 1.05])
    plot(t_SFB, x_SFB(:, 1), 'k');
    plot(t_OFB_sat_1, x_OFB_sat_1(:, 1), '--k');
    plot(t_OFB_sat_2, x_OFB_sat_2(:, 1), ':k');
    ylabel('$x_1$', 'Interpreter', 'Latex');
    legend('SFB', 'OFB, sat = 24, \mu = 0.01', 'OFB, sat = 4, \mu = 0.01')
subplot(2, 1, 2);
    hold on;
    box on;
    axis([0 10 -0.95 0.08])
    plot(t_SFB, x_SFB(:, 2), 'k');
    plot(t_OFB_sat_1, x_OFB_sat_1(:, 2), '--k');
    plot(t_OFB_sat_2, x_OFB_sat_2(:, 2), ':k');
    ylabel('$x_2$', 'Interpreter', 'Latex');
    xlabel('Time, $t$', 'Interpreter', 'Latex');
% save2tikz([figpath 'sfb_ofb_sat_comparison.tikz'])

figure;
subplot(2, 1, 1);
    hold on;
    box on;
    axis([0 0.15 -110 10])
    plot(t_OFB_2, -x_OFB_2(:, 3:4)*[2; 2], 'k')
    ylabel('$u(\hat x)$', 'Interpreter', 'Latex');

subplot(2, 1, 2);
    hold on;
    box on;
    axis([0 0.15 -4.2 0.1])
    plot(t_OFB_2_sat, sat(-x_OFB_2_sat(:, 3:4)*[2; 2], 4), 'k')
    ylabel('$u_s(\hat x)$', 'Interpreter', 'Latex');
    xlabel('Time, $t$', 'Interpreter', 'Latex');   
% save2tikz([figpath 'u_comparison.tikz'])

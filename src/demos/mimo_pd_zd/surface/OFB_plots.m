clc; clear; close all;

load plots.mat

%% Output tracking
% figure('Position', [0 0 300 200]);
% subplot_tight(2, 1, 1);
%     hold on; box on;
%     axis([0 15 -1.5 1.5]);
%     plot(t, e1(:, 1) + yd1(t), 'k');
%     plot(t, yd1(t), ':k');
%     set(gca,'XTick',[])
%     ylabel('$x_1^1(t)$', 'Interpreter', 'Latex');
% subplot_tight(2, 1, 2);
%     hold on; box on;
%     axis([0 15 -1.2 1.8]);
%     plot(t, e2(:, 1) + yd2(t), 'k');
%     plot(t, yd2(t), ':k');
%     ylabel('$x_1^2(t)$', 'Interpreter', 'Latex');
%     xlabel('Time, $t$', 'Interpreter', 'Latex');
% tightfig;

%% Tracking errors
figure('Position', [0 0 300 200]);
subplot_tight(2, 1, 1);
    hold on; box on;
    axis([0 15 -1.5 0.85]);
    plot(t, e1(:, 1), 'k');
    set(gca, 'XTick', 0:3:15);
    set(gca,'XTickLabels',[]);
    ylabel('$e_1^1(t)$', 'Interpreter', 'Latex');
    % Zoom plot of steady-state
        axes('Position',[.55 .6 .3 .15]);
        hold on; box on;
        axis([2 15 -1.5*tol/r^2 1.5*tol/r^2]);
        plot(t, e1(:, 1), 'k');
        plot([0 15], [1 1]*tol/r^2, ':k');
        plot([0 15], -[1 1]*tol/r^2, ':k');
subplot_tight(2, 1, 2);
    hold on; box on;
    axis([0 15 -1.1 1.9]);
    plot(t, e2(:, 1), 'k');
    set(gca, 'XTick', 0:3:15);
    ylabel('$e_1^2(t)$', 'Interpreter', 'Latex');
    xlabel('Time, $t$', 'Interpreter', 'Latex');
    % Zoom plot of steady-state
        axes('Position',[.55 .28 .3 .15]);
        hold on; box on;
        axis([2 15 -1.5*tol/r^2 1.5*tol/r^2]);
        plot(t, e2(:, 1), 'k');
        plot([0 15], [1 1]*tol/r^2, ':k');
        plot([0 15], -[1 1]*tol/r^2, ':k');    
tightfig;
% save2tikz('../tex/reports/surface-tac/figures/fig-1.1-new.tex');

%% Rest of the states
figure('Position', [0 0 300 200]);
subplot_tight(2, 1, 1);
    hold on; box on;
    axis([0 15 -9.2 7.2]);
    plot(t, e1(:, 2)' + [0 1 0]*psi1(t'), 'k');
    plot(t, e1(:, 3)' + [0 0 1]*psi1(t'), 'r');
    set(gca, 'XTick', 0:3:15);
    set(gca,'XTickLabels',[])
    ylabel('$x_2^1(t),\ x_3^1(t)$', 'Interpreter', 'Latex');
subplot_tight(2, 1, 2);
    hold on; box on;
    axis([0 15 -10 10.2]);
    plot(t, e2(:, 2)' + [0 1 0]*psi2(t'), 'k');
    plot(t, e2(:, 3)' + [0 0 1]*psi2(t'), 'r');
    set(gca, 'XTick', 0:3:15);
    ylabel('$x_2^2(t),\ x_3^2(t)$', 'Interpreter', 'Latex');
    xlabel('Time, $t$', 'Interpreter', 'Latex');
tightfig;
% save2tikz('../tex/reports/surface-tac/figures/fig-1.2-new.tex');


%% Surfaces
figure('Position', [0 0 300 200]);
subplot_tight(2, 1, 1);
    hold on; box on;
    axis([0 15 -50 50]);
    plot(t, s1, 'k');
    plot(t, s1hat, '--k');
    plot(t, rho(t), ':k');
    plot(t, -rho(t), ':k');
    set(gca, 'XTick', 0:3:15);
    set(gca,'XTickLabels',[])
    ylabel('$\sigma_1(t),\ \hat \sigma_1(t)$', 'Interpreter', 'Latex'); 
    % Zoom plot of peaking
        axes('Position',[.6 .775 .3 .15]);
        axis([0 0.0002 -2.2e9 9e8]);
        hold on; box on;
        plot(t, s1, 'k');
        plot(t, s1hat, '--k');
subplot_tight(2, 1, 2);
    hold on; box on;
    axis([0 15 -50 50]);
    plot(t, s2, 'k');
    plot(t, s2hat, '--k');    
    plot(t, rho(t), ':k');
    plot(t, -rho(t), ':k');
    set(gca, 'XTick', 0:3:15);
    ylabel('$\sigma_2(t),\ \hat \sigma_2(t)$', 'Interpreter', 'Latex');    
    xlabel('Time, $t$', 'Interpreter', 'Latex');
    % Zoom plot of peaking
        axes('Position',[.6 .2925 .3 .15]);
        axis([0 0.0002 -1.8e8 4.5e8]);
        hold on; box on;
        plot(t, s2, 'k');
        plot(t, s2hat, '--k');
tightfig;
% save2tikz('../tex/reports/surface-tac/figures/fig-1.3-new.tex');


%% Controls
figure('Position', [0 0 300 200]);
subplot_tight(2, 1, 1);
    hold on; box on;
    axis([0 15 -70 125]);
    plot(t, u1, 'k');
    set(gca, 'XTick', 0:3:15);
    set(gca,'XTickLabels',[])
    ylabel('$u_1(t)$', 'Interpreter', 'Latex');
    % Zoom plot of steady-state
        axes('Position',[.6 .775 .3 .15]);
        axis([0 0.0005 -115 20]);
        hold on; box on;
        plot(t, u1, 'k');
subplot_tight(2, 1, 2);
    hold on; box on;
    axis([0 15 -45 110]);
    plot(t, u2, 'k');
    set(gca, 'XTick', 0:3:15);
    ylabel('$u_2(t)$', 'Interpreter', 'Latex');    
    xlabel('Time, $t$', 'Interpreter', 'Latex');
    % Zoom plot of peaking
        axes('Position',[.6 .2925 .3 .15]);
        axis([0 0.0005 -115 20]);
        hold on; box on;
        plot(t, u1, 'k');  
tightfig;
save2tikz('../tex/reports/surface-tac/figures/fig-1.4-new.tex');


DOF2_surface_common

%% Output feedback
A = [1 2; 3 4];
C = [1 0 0 0
     0 0 1 0];
 
observer = @(t, z, y) hgo(t, z, y, alpha, mu);
MIMO_observer = @(t, z, y) MIMO_observer(t, z, y, observer, A);

controller = @(t, e, yd) ppc_surf_robust(t, e, Lambda, rho, k);
sat_controller = @(t, e, yd) sat_control(t, e, controller, satlvl, yd);
MIMO_control = @(t, e) MIMO_controller(t, e, A, sat_controller, @(t) 0, []);

sys1 = @(t, q) control_loop(t, q, plant_e, [n 0 n], MIMO_control, MIMO_observer, C);

[t, q] = ode15s(sys1, [0 tmax], q0, ode_options);

% Reconstruct state estimates
e1 = q(:, A(1, :));
e2 = q(:, A(2, :));
z1 = q(:, n + A(1, :));
z2 = q(:, n + A(2, :));

y = yd(t')';

u1 = zeros(size(t));
u2 = zeros(size(t));

s1 = e1*Lambda;
s2 = e2*Lambda;
s1hat = z1*Lambda;
s2hat = z2*Lambda;

for i = 1:numel(t)
    qi = [z1(i, :)'; z2(i, :)'];
    u = MIMO_control(t(i), qi);
    u1(i) = u(1);
    u2(i) = u(2);
end

%% Plots
figure('Position', [0 0 300 200]);
subplot_tight(2, 1, 1);
    hold on; box on;
    axis([0 15 -0.5 4.5]);
    plot(t, e1(:, 1), 'k');
    set(gca,'XTickLabel',[])
    ylabel('$y^1(t) - y_d^1(t)$', 'Interpreter', 'Latex');
    % Zoom plot of steady-state
        axes('Position',[.6 .775 .3 .15]);
        hold on; box on;
        axis([2 15 -1.5*tol/r 1.5*tol/r]);
        plot(t, e1(:, 1), 'k');
        plot([0 15], [1 1]*tol/r^2, ':k');
        plot([0 15], -[1 1]*tol/r^2, ':k');
subplot_tight(2, 1, 2);
    hold on; box on;
    axis([0 15 -0.6 1.6]);
    plot(t, e2(:, 1), 'k');
    ylabel('$y^2(t) - y_d^2(t)$', 'Interpreter', 'Latex');
    xlabel('Time, $t$ [\si{\second}]', 'Interpreter', 'Latex');
    % Zoom plot of steady-state
        axes('Position',[.6 .2925 .3 .15]);
        hold on; box on;
        axis([2 15 -1.5*tol/r 1.5*tol/r]);
        plot(t, e2(:, 1), 'k');
        plot([0 15], [1 1]*tol/r^2, ':k');
        plot([0 15], -[1 1]*tol/r^2, ':k');    
tightfig;
% save2tikz('../tex/reports/surface-tac/figures/fig-2.1.tex');

%% Rest of the states
figure('Position', [0 0 300 200]);
subplot_tight(2, 1, 1);
    hold on; box on;
    axis([0 15 -4.3 4.8]);
    plot(t, e1(:, 2)' + [0 1]*psi1(t'), 'k');
    set(gca,'XTickLabel',[])
    ylabel('$x_2^1(t),\ x_3^1(t)$', 'Interpreter', 'Latex');
subplot_tight(2, 1, 2);
    hold on; box on;
    axis([0 15 -4.5 8.2]);
    plot(t, e2(:, 2)' + [0 1]*psi2(t'), 'k');
    ylabel('$x_2^2(t),\ x_3^2(t)$', 'Interpreter', 'Latex');
    xlabel('Time, $t$ [\si{\second}]', 'Interpreter', 'Latex');
tightfig;
% save2tikz('../tex/reports/surface-tac/figures/fig-2.2.tex');

%% Surfaces
figure('Position', [0 0 300 200]);
subplot_tight(2, 1, 1);
    hold on; box on;
    axis([0 15 -30 50]);
    plot(t, s1, 'k');
    plot(t, rho(t), ':k');
    plot(t, -rho(t), ':k');
    set(gca,'XTickLabel',[])
    ylabel('$\sigma_1(t), \hat \sigma_1(t)$', 'Interpreter', 'Latex');
    % Zoom plot of peaking
        axes('Position',[.6 .775 .3 .15]);
        axis([0 0.0002 -1e4 5e4]);
        hold on; box on;
        plot(t, s1, 'k');
        plot(t, s1hat, '--k');    
subplot_tight(2, 1, 2);
    hold on; box on;
    axis([0 15 -30 30]);
    plot(t, s2, 'k');
    plot(t, rho(t), ':k');
    plot(t, -rho(t), ':k');
    ylabel('$\sigma_2(t), \hat \sigma_2(t)$', 'Interpreter', 'Latex');    
    xlabel('Time, $t$ [\si{\second}]', 'Interpreter', 'Latex');
    % Zoom plot of peaking
        axes('Position',[.6 .2925 .3 .15]);
        axis([0 0.0002 -1e4 1.5e3]);
        hold on; box on;
        plot(t, s2, 'k');
        plot(t, s2hat, '--k');    
tightfig;
% save2tikz('../tex/reports/surface-tac/figures/fig-2.3.tex');

%% Controls
figure('Position', [0 0 300 200]);
subplot_tight(2, 1, 1);
    hold on; box on;
    axis([0 15 -30 60]);
    plot(t, u1, 'k');
    set(gca,'XTickLabel',[])
    ylabel('$u_1(t)$', 'Interpreter', 'Latex');
    % Zoom plot of steady-state
        axes('Position',[.6 .775 .3 .15]);
        axis([0 0.0005 -35 5]);
        hold on; box on;
        plot(t, u1, 'k');
subplot_tight(2, 1, 2);
    hold on; box on;
    axis([0 15 -18 27]);
    plot(t, u2, 'k');
    ylabel('$u_2(t)$', 'Interpreter', 'Latex');    
    xlabel('Time, $t$ [\si{\second}]', 'Interpreter', 'Latex');
    % Zoom plot of steady-state
        axes('Position',[.6 .2925 .3 .15]);
        axis([0 0.0005 -35 5]);
        hold on; box on;
        plot(t, u1, 'k');  
tightfig;
% save2tikz('../tex/reports/surface-tac/figures/fig-2.4.tex');


DOF2_FL_common

%% Output feedback
A = [1 2; 3 4];
C = [1 0 0 0
     0 0 1 0];
 
observer = @(t, z, y) hgo(t, z, y, alpha, mu);
MIMO_observer = @(t, z, y) MIMO_observer(t, z, y, observer, A);

controller = @(t, x) dof2_fl(t, x, psi, dpsi, zeta, wn, [m1 m2 l1 l2 Iz1 Iz2 g]);
sat_controller = @(t, x) sat_control(t, x, controller, satlvl);

sys1 = @(t, q) control_loop(t, q, plant, [n 0 n], sat_controller, MIMO_observer, C);

[t, q] = ode15s(sys1, [0 tmax], q0, ode_options);

% Reconstruct state estimates
e1 = q(:, A(1, :)) - psi1(t')';
e2 = q(:, A(2, :)) - psi2(t')';
z1 = q(:, n + A(1, :));
z2 = q(:, n + A(2, :));

y = yd(t')';

u1 = zeros(size(t));
u2 = zeros(size(t));

for i = 1:numel(t)
    qi = [z1(i, :)'; z2(i, :)'];
    u = sat_controller(t(i), qi);
    u1(i) = u(1);
    u2(i) = u(2);
end

%% Plots
figure('Position', [0 0 300 200]);
subplot_tight(2, 1, 1);
    hold on; box on;
    axis([0 15 -0.5 4.5]);
    plot(t, e1(:, 1), 'k');
    plot(t, z1(:, 1)' - [1 0]*psi1(t'), '--k');
    set(gca,'XTickLabel',[])
    ylabel('$q_1(t) - r_1(t)$', 'Interpreter', 'Latex');
subplot_tight(2, 1, 2);
    hold on; box on;
    axis([0 15 -0.6 1.6]);
    plot(t, e2(:, 1), 'k');
    plot(t, z2(:, 1)' - [1 0]*psi2(t'), '--k');
    ylabel('$q_2(t) - r_2(t)$', 'Interpreter', 'Latex');
    xlabel('Time, $t$', 'Interpreter', 'Latex');
tightfig;
% save2tikz('../tex/reports/surface-tac/figures/fig-2.1.tex');

%% Rest of the states
figure('Position', [0 0 300 200]);
subplot_tight(2, 1, 1);
    hold on; box on;
    axis([0 15 -4.3 4.8]);
    plot(t, e1(:, 2)' + [0 1]*psi1(t'), 'k');
    set(gca,'XTickLabel',[])
    ylabel('$\dot q_1(t)$', 'Interpreter', 'Latex');
subplot_tight(2, 1, 2);
    hold on; box on;
    axis([0 15 -4.5 8.2]);
    plot(t, e2(:, 2)' + [0 1]*psi2(t'), 'k');
    ylabel('$\dot q_2(t)$', 'Interpreter', 'Latex');
    xlabel('Time, $t$', 'Interpreter', 'Latex');
tightfig;
% save2tikz('../tex/reports/surface-tac/figures/fig-2.2.tex');

%% Controls
figure('Position', [0 0 300 200]);
subplot_tight(2, 1, 1);
    hold on; box on;
    plot(t, u1, 'k');
    set(gca,'XTickLabel',[])
    ylabel('$u_1(t)$', 'Interpreter', 'Latex');
subplot_tight(2, 1, 2);
    hold on; box on;
    plot(t, u2, 'k');
    ylabel('$u_2(t)$', 'Interpreter', 'Latex');    
    xlabel('Time, $t$', 'Interpreter', 'Latex');
tightfig;
% save2tikz('../tex/reports/surface-tac/figures/fig-2.4.tex');


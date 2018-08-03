%% Output feedback
DOF2_backstepping_common

A = [1 2; 3 4];
C = [1 0 0 0
     0 0 1 0];
 
observer = @(t, z, y) hgo(t, z, y, alpha, mu);
MIMO_observer = @(t, z, y) MIMO_observer(t, z, y, observer, A);

controller1 = @(t, x, yd) ppc_backstepping(t, x, yd, rho1, k);
sat_controller1 = @(t, x, yd) sat_control(t, x, controller1, satlvl1, yd);
controller2 = @(t, x, yd) ppc_backstepping(t, x, yd, rho2, k);
sat_controller2 = @(t, x, yd) sat_control(t, x, controller2, satlvl2, yd);

sat_controller = {sat_controller1, sat_controller2};

MIMO_control = @(t, x, w) MIMO_controller(t, x, A, sat_controller, yd, []);
sys1 = @(t, q) control_loop(t, q, plant, [n 0 n], MIMO_control, MIMO_observer, C);

[t, q] = ode15s(sys1, [0 tmax], q0, ode_options);

% Reconstruct state estimates
q1 = q(:, A(1, :));
q2 = q(:, A(2, :));
z1 = q(:, 4 + A(1, :));
z2 = q(:, 4 + A(2, :));

y = yd(t')';

% xi11 = (q1(:,1) - y(:,1))./rho(t);
% eps11 = log((1+xi11)./(1-xi11))./(1-xi11.^2);
% xi12 = (q1(:,2) + k(1)*eps11)./rho(t);
% eps12 = log((1+xi12)./(1-xi12))./(1-xi12.^2);
% % u1 = -k(2)*eps12;
% 
% xi21 = (q2(:,1) - y(:,2))./rho(t);
% eps21 = log((1+xi21)./(1-xi21))./(1-xi21.^2);
% xi22 = (q2(:,2) + k(1)*eps21)./rho(t);
% eps22 = log((1+xi22)./(1-xi22))./(1-xi22.^2);
% % u2 = -k(2)*eps22;

u1 = zeros(size(t));
u2 = zeros(size(t));

for i = 1:numel(t)
    qi = [z1(i,1); z1(i,2); z2(i,1); z2(i,2)];
    u = MIMO_control(t(i), qi, []);
    u1(i) = u(1);
    u2(i) = u(2);
end

%% Plots
plotter('t', t, 'x', q1, 'xhat', z1);
suptitle('Link 1 states');

plotter('t', t, 's', q1(:, 1) - y(:, 1), 'u', u1, 'rho', rho1, 'peak', 0.001);
% suptitle('Link 1 tracking error');
% save2tikz ../tex/reports/'backstepping sfb+ofb (MIMO)'/figures/dof2_link1.tex

plotter('t', t, 'x', q2, 'xhat', z2);
suptitle('Link 2 states');

plotter('t', t, 's', q2(:, 1) - y(:, 2), 'u', u2, 'rho', rho2, 'peak', 0.001);
% suptitle('Link 2 tracking error');
% save2tikz ../tex/reports/'backstepping sfb+ofb (MIMO)'/figures/dof2_link2.tex

% 
% plotter('t', t, 'x', [xi11 xi12]);
% suptitle('\xi_{1,1}, \xi_{1,2}');
% 
% plotter('t', t, 'x', [eps11 eps12]);
% suptitle('\epsilon_{1,1}, \epsilon_{1,2}')


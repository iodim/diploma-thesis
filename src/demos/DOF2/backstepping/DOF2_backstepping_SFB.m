%% State feedback
DOF2_backstepping_common

A = [1 2; 3 4];
controller1 = @(t, x, yd) ppc_backstepping(t, x, yd, rho1, k);
controller2 = @(t, x, yd) ppc_backstepping(t, x, yd, rho2, k);
controller = {controller1, controller2};
MIMO_control = @(t, x, w) MIMO_controller(t, x, A, controller, yd, []);
sys1 = @(t, q) control_loop(t, q, plant, [n 0 0], MIMO_control);

[t, q] = ode15s(sys1, [0 tmax], x0, ode_options);

% Reconstruct state estimates
q1 = q(:, A(1, :));
q2 = q(:, A(2, :));
y = yd(t')';

xi11 = (q1(:,1) - y(:,1))./rho1(t);
eps11 = log((1+xi11)./(1-xi11));%./(1-xi11.^2);
xi12 = (q1(:,2) + k(1)*eps11)./rho1(t);
eps12 = log((1+xi12)./(1-xi12));%./(1-xi12.^2);
u1 = -k(2)*eps12;

xi21 = (q2(:,1) - y(:,2))./rho2(t);
eps21 = log((1+xi21)./(1-xi21));%./(1-xi21.^2);
xi22 = (q2(:,2) + k(1)*eps21)./rho2(t);
eps22 = log((1+xi22)./(1-xi22));%./(1-xi22.^2);
u2 = -k(2)*eps22;

u = [u1 u2];

% Plots
plotter('t', t, 'x', q1);
suptitle('Link 1 states');

plotter('t', t, 's', q1(:, 1) - y(:, 1), 'rho', rho1, 'u', u1);
suptitle('Link 1 tracking error');

plotter('t', t, 'x', q2);
suptitle('Link 2 states');

plotter('t', t, 's', q2(:, 1) - y(:, 2), 'rho', rho2, 'u', u2);
suptitle('Link 2 tracking error');
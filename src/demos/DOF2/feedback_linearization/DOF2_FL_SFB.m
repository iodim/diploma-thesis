DOF2_FL_common

%% State feedback
A = [1 2; 3 4];
controller = @(t, x) dof2_fl(t, x, psi, dpsi, zeta, wn, [m1 m2 l1 l2 Iz1 Iz2 g]);
% MIMO_control = @(t, e, w) MIMO_controller(t, e, A, controller, @(t) 0, w);
sys1 = @(t, q) control_loop(t, q, plant, [n 0 0], controller);

[t, q] = ode15s(sys1, [0 tmax], x0, ode_options);

% Reconstruct state estimates
x1 = q(:, A(1, :));
x2 = q(:, A(2, :));
e1 = x1 - psi1(t')';
e2 = x2 - psi2(t')';

u = zeros(size(x1));
for i = 1:length(t)
    u(i, :) = controller(t(i), q(i, :)');
end

% Plots
plotter('t', t, 'x', e1);
suptitle('Link 1 states');

plotter('t', t, 'x', e2);
suptitle('Link 2 states');

plotter('t', t, 'u', u(:, 1));
suptitle('Link 1 torque');

plotter('t', t, 'u', u(:, 2));
suptitle('Link 2 torque');